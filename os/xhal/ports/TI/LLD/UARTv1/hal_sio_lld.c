/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    UARTv1/hal_sio_lld.c
 * @brief   AM67 SIO subsystem low level driver source.
 *
 * @addtogroup HAL_SIO
 * @{
 */

#include "hal.h"

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/**
 * @brief   Driver-private sticky bit for the character timeout condition.
 * @note    Deliberately above the eight hardware LSR bits: the 16550 signals
 *          a receiver gone quiet through the character timeout interrupt,
 *          not through a status bit, so the handler records it alongside the
 *          latched LSR bits.
 */
#define SIO_LSR_CTI                         (1U << 8)

/**
 * @brief   Error and status bits latched by the handler.
 */
#define SIO_LSR_STICKY                      (TI_UART_LSR_RX_ERRORS | SIO_LSR_CTI)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if (AM67_SIO_USE_UART1 == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   SIOD1 driver identifier.
 */
SIODriver SIOD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SIOConfig default_config = {
  .baud                 = SIO_DEFAULT_BITRATE,
  .lcr                  = TI_UART_LCR_8N1,
  .fcr                  = TI_UART_FCR_FIFOEN | TI_UART_FCR_RXTRIGGER_8
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Writes the IER register through the driver shadow.
 * @note    IER and DLH share an address, selected by LCR.DLAB. Going
 *          through the shadow keeps the driver from reading back whatever
 *          the divisor latch happens to expose.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] ier       new IER value
 */
static void uart_set_ier(SIODriver *siop, uint32_t ier) {

  /* IER and DLH share an address, selected by LCR.DLAB. The bit is only
     ever raised inside sio_lld_setcfg(), which runs with the interrupt
     line disabled, but the write is cheap and makes the register this
     function names the one it actually reaches.*/
  siop->uart->LCR &= ~TI_UART_LCR_DLAB;

  siop->ier = ier;
  siop->uart->IER_DLH = ier;
}

/**
 * @brief   Latches the volatile part of the line status.
 * @note    LSR clears its error bits on read, so any read that is not
 *          recorded here loses them for good. Every LSR read in this
 *          driver goes through this function for that reason, including
 *          the plain status checks.
 * @note    The accumulator is only written when the read actually carries
 *          an error, which keeps the common path free of a
 *          read-modify-write that the interrupt handler could interleave
 *          with. The window is not closed for the error case itself: this
 *          port has no @p X class locking, and paying for a critical
 *          section on every status check would be worse than the residual
 *          risk of losing a status bit that arrives in the same instant.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The raw LSR value.
 */
static uint32_t uart_latch_lsr(SIODriver *siop) {
  uint32_t lsr;

  lsr = siop->uart->LSR;
  if ((lsr & TI_UART_LSR_RX_ERRORS) != 0U) {
    siop->lsr |= lsr & TI_UART_LSR_RX_ERRORS;
  }

  return lsr;
}

/**
 * @brief   Translates latched status bits into SIO events.
 *
 * @param[in] lsr       latched status bits
 * @return              The SIO events mask.
 */
static sioevents_t uart_lsr2evt(uint32_t lsr) {
  sioevents_t evt = (sioevents_t)0;

  if ((lsr & TI_UART_LSR_PE) != 0U) {
    evt |= SIO_EV_PARITY_ERR;
  }
  if ((lsr & TI_UART_LSR_FE) != 0U) {
    evt |= SIO_EV_FRAMING_ERR;
  }
  if ((lsr & TI_UART_LSR_OE) != 0U) {
    evt |= SIO_EV_OVERRUN_ERR;
  }
  if ((lsr & TI_UART_LSR_BI) != 0U) {
    evt |= SIO_EV_RX_BREAK;
  }
  if ((lsr & SIO_LSR_CTI) != 0U) {
    evt |= SIO_EV_RX_IDLE;
  }

  return evt;
}

/**
 * @brief   Translates SIO events back into latched status bits.
 * @note    Only the conditions the driver latches have a counterpart, the
 *          live FIFO events clear themselves and map to nothing.
 *
 * @param[in] evt       SIO events mask
 * @return              The latched status bits.
 */
static uint32_t uart_evt2lsr(sioevents_t evt) {
  uint32_t lsr = 0U;

  if ((evt & SIO_EV_PARITY_ERR) != 0U) {
    lsr |= TI_UART_LSR_PE;
  }
  if ((evt & SIO_EV_FRAMING_ERR) != 0U) {
    lsr |= TI_UART_LSR_FE;
  }
  if ((evt & SIO_EV_OVERRUN_ERR) != 0U) {
    lsr |= TI_UART_LSR_OE;
  }
  if ((evt & SIO_EV_RX_BREAK) != 0U) {
    lsr |= TI_UART_LSR_BI;
  }
  if ((evt & SIO_EV_RX_IDLE) != 0U) {
    lsr |= SIO_LSR_CTI;
  }

  return lsr;
}

/**
 * @brief   Detects the physical end of transmission.
 * @details The generated event is latched by the caller through the usual
 *          wakeup path. A transmission that was never started, or whose
 *          end has already been signaled, is reported as finished so the
 *          polling timer is not armed for it.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The detection status.
 * @retval true         if nothing is outstanding, polling is not required
 * @retval false        if a transmission is still on the wire
 */
static bool uart_txend_process(SIODriver *siop) {

  if (!siop->txend_pending) {
    return true;
  }

  /* TEMT covers both the holding register and the shift register, it is
     the only honest end-of-transmission indication this peripheral has.*/
  if ((uart_latch_lsr(siop) & TI_UART_LSR_TEMT) == 0U) {
    return false;
  }

  siop->txend_pending = false;
  __sio_wakeup_txend(siop);

  return true;
}

#if defined(__CHIBIOS_RT__) || defined(__DOXYGEN__)
/**
 * @brief   TX-end polling timer callback.
 * @details ETBEI reports the holding register going empty, which happens
 *          one character before the wire goes idle, and re-arming it would
 *          only produce a level-sensitive interrupt storm. The remaining
 *          character time is waited out here instead.
 * @note    Virtual timer callbacks run in ISR context outside the kernel
 *          critical section; the driver callback is invoked outside it as
 *          its contract requires.
 *
 * @param[in] vtp       pointer to the virtual timer
 * @param[in] p         pointer to the @p SIODriver object
 */
static void uart_txend_timer_cb(virtual_timer_t *vtp, void *p) {
  SIODriver *siop = (SIODriver *)p;

  if (uart_txend_process(siop)) {
    __sio_callback(siop);
  }
  else {
    chSysLockFromISR();
    chVTSetI(vtp, siop->txend_step, uart_txend_timer_cb, p);
    chSysUnlockFromISR();
  }
}
#endif /* defined(__CHIBIOS_RT__) */

/**
 * @brief   UART deactivation.
 * @details Masks the sources, stops the TX-end machinery and leaves the
 *          peripheral in its reset mode. Shared by the stop path and by
 *          the start failure rollback.
 * @note    The vector is disabled before the timer is reset so that the
 *          handler cannot re-arm it behind this function.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 */
static void uart_deactivate(SIODriver *siop) {

  uart_set_ier(siop, 0U);

  vimDisableInterrupt(siop->irq);
  vimSetHandler(siop->irq, NULL, NULL);

#if defined(__CHIBIOS_RT__)
  chVTReset(&siop->txend_vt);
#endif
  siop->txend_pending = false;

  /* The peripheral is inert with the mode disabled, which is also its
     reset state, so a later start does not inherit half a setup.*/
  siop->uart->MDR1 = TI_UART_MDR1_MODE_DISABLE;
}

/**
 * @brief   Maximum handler entries without application-side progress.
 * @note    Sized well above any legitimate burst: the application answers a
 *          serviced source through the read, write or event paths, and those
 *          reset the counter.
 */
#define SIO_UNSERVICED_LIMIT                32U

/**
 * @brief   Common interrupt service routine.
 *
 * @param[in] arg       pointer to the @p SIODriver object
 * @return              The preemption-required flag.
 */
static bool uart_irq_handler(void *arg) {
  SIODriver *siop = (SIODriver *)arg;
  bool preemption_required;

  /* Called out of the lock: the __sio_wakeup_xxx() macros take the lock
     themselves and the driver callback must not run while holding it.*/
  sio_lld_serve_interrupt(siop);

  /* Runaway guard, see the note in the driver header. A source that cannot
     be silenced would otherwise re-enter this handler forever and starve
     the system tick, taking every timeout down with it.*/
  siop->unserviced++;
  if (siop->unserviced > SIO_UNSERVICED_LIMIT) {
    siop->runaway = true;
    vimDisableInterrupt(siop->irq);
  }

  chSysLockFromISR();
  preemption_required = chSchIsPreemptionRequired();
  chSysUnlockFromISR();

  return preemption_required;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level SIO driver initialization.
 *
 * @notapi
 */
void sio_lld_init(void) {

#if AM67_SIO_USE_UART1 == TRUE
  sioObjectInit(&SIOD1);
  SIOD1.uart  = (TI_UART_TypeDef *)(void *)AM67_MAIN_UART1_BASE;
  SIOD1.irq   = AM67_MAIN_UART1_IRQ;
  SIOD1.clock = AM67_MAIN_UART1_CLOCK;
  SIOD1.ier   = 0U;
  SIOD1.lsr   = 0U;
  SIOD1.txend_pending = false;
  SIOD1.unserviced    = 0U;
  SIOD1.runaway       = false;
#if defined(__CHIBIOS_RT__)
  chVTObjectInit(&SIOD1.txend_vt);
  SIOD1.txend_step = (sysinterval_t)1;
#endif
#endif
}

/**
 * @brief   Configures and activates the SIO peripheral.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The operation status.
 *
 * @notapi
 */
msg_t sio_lld_start(SIODriver *siop) {

  /* Activation is unconditional: drvStart() moves the driver to
     HAL_DRV_STATE_STARTING before invoking this method, so testing for
     HAL_DRV_STATE_STOP here would never be true.*/

  /* Starting the session with an idle TX-end machinery and a clean guard.*/
  siop->txend_pending = false;
  siop->unserviced    = 0U;
  siop->runaway       = false;

  /* Configures the peripheral before the interrupt line is enabled. The
     order matters: sio_lld_setcfg() disables the module and raises
     LCR.DLAB while it programs the divisor, and in that window the handler
     would read a disabled peripheral and write its IER update into the
     divisor latch instead, leaving the transmitter interrupt enabled in
     hardware for good.

     The returned pointer becomes the active configuration, the shared
     driver requires one to be set.*/
  siop->config = sio_lld_setcfg(siop, siop->config);
  if (siop->config == NULL) {
    return HAL_RET_CONFIG_ERROR;
  }

#if AM67_SIO_USE_UART1 == TRUE
  if (&SIOD1 == siop) {
    vimSetHandler(siop->irq, uart_irq_handler, (void *)siop);
    vimSetPriority(siop->irq, AM67_SIO_UART1_IRQ_PRIORITY);
    vimEnableInterrupt(siop->irq);
  }
#endif

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Deactivates the SIO peripheral.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
void sio_lld_stop(SIODriver *siop) {

  /* Unconditional, for the same reason as in sio_lld_start(): drvStop()
     moves the driver to HAL_DRV_STATE_STOPPING before invoking this
     method.*/
  uart_deactivate(siop);
}

/**
 * @brief   SIO configuration.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] config    pointer to the @p SIOConfig structure, @p NULL
 *                      selects the default configuration
 * @return              A pointer to the current configuration structure.
 * @retval NULL         if the configuration is not valid.
 *
 * @notapi
 */
const SIOConfig *sio_lld_setcfg(SIODriver *siop, const SIOConfig *config) {
  TI_UART_TypeDef *u = siop->uart;
  uint32_t divisor;

  if (config == NULL) {
    config = &default_config;
  }

  /* Checked before it reaches the divisor calculation below.*/
  if (config->baud == 0U) {
    return NULL;
  }

  /* The 16x oversampling divisor must be representable and non zero.*/
  divisor = siop->clock / (16U * config->baud);
  if ((divisor == 0U) || (divisor > 0xFFFFU)) {
    return NULL;
  }

  /* Held disabled while it is reprogrammed, the divisor latch must not be
     written with the transmitter live.*/
  u->MDR1 = TI_UART_MDR1_MODE_DISABLE;
  uart_set_ier(siop, 0U);

  u->LCR = TI_UART_LCR_DLAB;
  u->RBR_THR_DLL = divisor & 0xFFU;
  u->IER_DLH = (divisor >> 8) & 0xFFU;

  /* DLAB is masked out of the caller's line configuration: leaving it set
     would keep THR and IER pointing at the divisor latches.*/
  u->LCR = config->lcr & ~TI_UART_LCR_DLAB;

  /* The FIFO reset bits are self clearing, they are only meaningful in the
     same write that enables the FIFOs.*/
  u->IIR_FCR = config->fcr | TI_UART_FCR_RXRST | TI_UART_FCR_TXRST;

  /* Discards anything the previous owner left in the receiver.*/
  (void)uart_latch_lsr(siop);
  siop->lsr = 0U;

#if defined(__CHIBIOS_RT__)
  /* One character time at this baud rate, the granularity at which TEMT
     is worth re-testing. Rounded up to one tick so a fast link cannot ask
     for a zero interval.*/
  siop->txend_step = chTimeUS2I((10U * 1000000U) / config->baud);
  if (siop->txend_step == (sysinterval_t)0) {
    siop->txend_step = (sysinterval_t)1;
  }
#endif

  /* Written last, the peripheral does nothing at all until the mode is
     selected.*/
  u->MDR1 = TI_UART_MDR1_MODE_UART16X;

  return config;
}

/**
 * @brief   Selects one of the pre-defined SIO configurations.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] cfgnum    driver configuration number
 * @return              The configuration pointer.
 * @retval NULL         if the configuration is not valid.
 *
 * @notapi
 */
const hal_sio_config_t *sio_lld_selcfg(SIODriver *siop,
                                       unsigned cfgnum) {
#if SIO_USE_CONFIGURATIONS == TRUE
  extern const sio_configurations_t sio_configurations;

  if (cfgnum >= sio_configurations.cfgsnum) {
    return NULL;
  }

  return (const void *)sio_lld_setcfg(siop, &sio_configurations.cfgs[cfgnum]);
#else

  if (cfgnum > 0U) {
    return NULL;
  }

  return (const void *)sio_lld_setcfg(siop, NULL);
#endif
}

/**
 * @brief   Determines the state of the RX FIFO.
 * @note    A function rather than a register macro: LSR clears its error
 *          bits on read, so the read has to go through the latch helper or
 *          a status check would silently consume an overrun.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX FIFO state.
 * @retval false        if RX FIFO is not empty
 * @retval true         if RX FIFO is empty
 *
 * @notapi
 */
bool sio_lld_is_rx_empty(SIODriver *siop) {

  return (bool)((uart_latch_lsr(siop) & TI_UART_LSR_DR) == 0U);
}

/**
 * @brief   Determines the activity state of the receiver.
 * @note    A 16550 has no line-idle status bit. The closest honest answer
 *          is "nothing is waiting to be read"; the RX idle *event* comes
 *          from the character timeout interrupt instead.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX activity state.
 * @retval false        if RX is in active state.
 * @retval true         if RX is in idle state.
 *
 * @notapi
 */
bool sio_lld_is_rx_idle(SIODriver *siop) {

  return (bool)((uart_latch_lsr(siop) & TI_UART_LSR_DR) == 0U);
}

/**
 * @brief   Determines the transmission state.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The TX state.
 * @retval false        if transmission is idle
 * @retval true         if transmission is ongoing
 *
 * @notapi
 */
bool sio_lld_is_tx_ongoing(SIODriver *siop) {

  return (bool)((uart_latch_lsr(siop) & TI_UART_LSR_TEMT) == 0U);
}

/**
 * @brief   Enable flags change notification.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
void sio_lld_update_enable_flags(SIODriver *siop) {
  uint32_t ier = 0U;

  /* Reaching here is the application answering the handler, so the runaway
     counter starts again. A line taken down by the guard is left down: it
     is a fault indication, not a condition to paper over.*/
  siop->unserviced = 0U;

  /* The character timeout that carries the RX idle event is only reported
     while the receiver interrupt is enabled, so both events map to it.*/
  if ((siop->enabled & (SIO_EV_RX_NOTEMPTY | SIO_EV_RX_IDLE)) != 0U) {
    ier |= TI_UART_IER_ERBFI;
  }

  /* The 16550 has no transmitter-empty interrupt, the handler tests TEMT
     when the holding register goes empty, so both events map to ETBEI.*/
  if ((siop->enabled & (SIO_EV_TX_NOTFULL | SIO_EV_TX_END)) != 0U) {
    ier |= TI_UART_IER_ETBEI;
  }

  if ((siop->enabled & SIO_EV_ALL_ERRORS) != 0U) {
    ier |= TI_UART_IER_ELSI;
  }

  uart_set_ier(siop, ier);
}

/**
 * @brief   Get and clears SIO error event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @notapi
 */
sioevents_t sio_lld_get_and_clear_errors(SIODriver *siop) {
  uint32_t lsr;

  (void)uart_latch_lsr(siop);

  lsr = siop->lsr & TI_UART_LSR_RX_ERRORS;
  siop->lsr &= ~TI_UART_LSR_RX_ERRORS;

  /* Errors acknowledged, the RX sources masked by the handler can be
     armed again.*/
  sio_lld_update_enable_flags(siop);

  return uart_lsr2evt(lsr);
}

/**
 * @brief   Get and clears SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] events    events to be returned and cleared
 * @return              The pending event flags.
 *
 * @notapi
 */
sioevents_t sio_lld_get_and_clear_events(SIODriver *siop, sioevents_t events) {
  sioevents_t pending;

  pending = sio_lld_get_events(siop) & events;

  /* Only the latched conditions actually returned are cleared: the live
     FIFO conditions clear themselves when the FIFOs are drained or
     filled, and an error the caller did not ask for stays pending
     instead of being dropped here.*/
  siop->lsr &= ~(uart_evt2lsr(pending) & SIO_LSR_STICKY);

  sio_lld_update_enable_flags(siop);

  return pending;
}

/**
 * @brief   Returns the pending SIO event flags.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The pending event flags.
 *
 * @notapi
 */
sioevents_t sio_lld_get_events(SIODriver *siop) {
  sioevents_t evt;
  uint32_t lsr;

  lsr = uart_latch_lsr(siop);

  evt = uart_lsr2evt(siop->lsr);

  if ((lsr & TI_UART_LSR_DR) != 0U) {
    evt |= SIO_EV_RX_NOTEMPTY;
  }
  if ((lsr & TI_UART_LSR_TEMT) != 0U) {
    evt |= SIO_EV_TX_END;
  }
  if ((siop->uart->SSR & TI_UART_SSR_TXFIFOFULL) == 0U) {
    evt |= SIO_EV_TX_NOTFULL;
  }

  return evt;
}

/**
 * @brief   Reads data from the RX FIFO.
 * @details The function is not blocking, it reads frames until the FIFO is
 *          empty without waiting.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[out] buffer   pointer to the buffer for read frames
 * @param[in] n         maximum number of frames to be read
 * @return              The number of frames copied from the FIFO.
 * @retval 0            if the RX FIFO is empty.
 *
 * @notapi
 */
size_t sio_lld_read(SIODriver *siop, uint8_t *buffer, size_t n) {
  size_t rd = 0U;

  while (rd < n) {
    if (sio_lld_is_rx_empty(siop)) {
      break;
    }

    *buffer++ = (uint8_t)siop->uart->RBR_THR_DLL;
    rd++;
  }

  /* Re-arms what the handler masked, now that the FIFO has been drained.*/
  if (sio_lld_is_rx_empty(siop)) {
    sio_lld_update_enable_flags(siop);
  }

  return rd;
}

/**
 * @brief   Writes data into the TX FIFO.
 * @details The function is not blocking, it writes frames until there is
 *          space available without waiting.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] buffer    pointer to the buffer for frames to be written
 * @param[in] n         maximum number of frames to be written
 * @return              The number of frames copied into the FIFO.
 * @retval 0            if the TX FIFO is full.
 *
 * @notapi
 */
size_t sio_lld_write(SIODriver *siop, const uint8_t *buffer, size_t n) {
  size_t wr = 0U;

  while (wr < n) {
    if (sio_lld_is_tx_full(siop)) {
      break;
    }

    siop->uart->RBR_THR_DLL = (uint32_t)*buffer++;
    wr++;
  }

  /* A transmission is outstanding until TEMT says otherwise. The polling
     timer is deliberately not armed here, detection belongs to the
     handler, see the TX-end notes in the driver header.*/
  if (wr > 0U) {
    siop->txend_pending = true;
  }

  /* Re-arms the transmitter interrupt masked by the handler.*/
  sio_lld_update_enable_flags(siop);

  return wr;
}

/**
 * @brief   Returns one frame from the RX FIFO.
 * @note    Must be invoked with the RX FIFO known to be non empty.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The received frame.
 *
 * @notapi
 */
msg_t sio_lld_get(SIODriver *siop) {
  msg_t msg;

  msg = (msg_t)(siop->uart->RBR_THR_DLL & 0xFFU);

  if (sio_lld_is_rx_empty(siop)) {
    sio_lld_update_enable_flags(siop);
  }

  return msg;
}

/**
 * @brief   Pushes one frame into the TX FIFO.
 * @note    Must be invoked with the TX FIFO known not to be full.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] data      frame to be pushed
 *
 * @notapi
 */
void sio_lld_put(SIODriver *siop, uint_fast16_t data) {

  siop->uart->RBR_THR_DLL = (uint32_t)data;

  /* See the equivalent note in sio_lld_write().*/
  siop->txend_pending = true;

  sio_lld_update_enable_flags(siop);
}

/**
 * @brief   Control operation on a serial port.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @param[in] operation control operation code
 * @param[in,out] arg   operation argument
 * @return              The control operation status.
 *
 * @notapi
 */
msg_t sio_lld_control(SIODriver *siop, unsigned int operation, void *arg) {

  (void)siop;
  (void)operation;
  (void)arg;

  return HAL_RET_UNKNOWN_CTL;
}

/**
 * @brief   Serves an UART interrupt.
 * @details The interrupt identification register is read once per pass: on
 *          a 16550 that read is what deasserts the reported condition, so
 *          reading it twice loses an interrupt.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 *
 * @notapi
 */
void sio_lld_serve_interrupt(SIODriver *siop) {
  uint32_t iir, intid;

  iir = siop->uart->IIR_FCR;
  if ((iir & TI_UART_IIR_INTSTATUS) != 0U) {
    /* Active low, nothing is pending.*/
    return;
  }

  intid = iir & TI_UART_IIR_INTID_MASK;

  /* The VIM line is level sensitive and none of these conditions clear
     until the application moves data, so the serviced sources are masked
     here and re-armed by the read/write paths. Without that the handler
     re-enters until the application happens to drain the FIFO.*/
  switch (intid) {
  case TI_UART_IIR_INTID_RLS:
    (void)uart_latch_lsr(siop);
    uart_set_ier(siop, siop->ier & ~(TI_UART_IER_ERBFI | TI_UART_IER_ELSI));
    __sio_wakeup_errors(siop);
    break;

  case TI_UART_IIR_INTID_CTI:
    siop->lsr |= SIO_LSR_CTI;
    uart_set_ier(siop, siop->ier & ~(TI_UART_IER_ERBFI | TI_UART_IER_ELSI));
    __sio_wakeup_rxidle(siop);
    __sio_wakeup_rx(siop);
    break;

  case TI_UART_IIR_INTID_RDA:
    uart_set_ier(siop, siop->ier & ~(TI_UART_IER_ERBFI | TI_UART_IER_ELSI));
    __sio_wakeup_rx(siop);
    break;

  case TI_UART_IIR_INTID_THRE:
    /* Masked here and re-armed by the write path, the condition is level
       sensitive and would otherwise re-enter until the FIFO is refilled.*/
    uart_set_ier(siop, siop->ier & ~TI_UART_IER_ETBEI);
    __sio_wakeup_tx(siop);

    /* THRE only says the holding register is empty, the last character can
       still be in the shift register. TEMT decides, and when it is not set
       yet the remaining character time is waited out by the polling timer
       rather than by re-arming ETBEI.*/
#if defined(__CHIBIOS_RT__)
    if (uart_txend_process(siop)) {
      chSysLockFromISR();
      chVTResetI(&siop->txend_vt);
      chSysUnlockFromISR();
    }
    else {
      chSysLockFromISR();
      chVTSetI(&siop->txend_vt, siop->txend_step,
               uart_txend_timer_cb, (void *)siop);
      chSysUnlockFromISR();
    }
#else
    /* Without the RT kernel only this opportunistic detection is
       available, see the notes in the driver header.*/
    (void)uart_txend_process(siop);
#endif
    break;

  default:
    break;
  }

  /* The callback is invoked out of the lock, per the XHAL callback
     contract.*/
  __sio_callback(siop);
}

#endif /* HAL_USE_SIO == TRUE */

/** @} */
