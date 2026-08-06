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
 * @file    TI/AM67/hal_serial_lld.c
 * @brief   AM67 (J722S) serial subsystem low level driver source.
 * @details Interrupt-driven driver for the 16550-compatible main-domain
 *          UARTs. Init sequence derived from NuttX u16550_setup() with the
 *          TI MDR1 mode dance around it (mode select written last, the
 *          UART is inert until then). Interrupts are routed through the
 *          VIM, the handler drains by IIR interrupt identification until
 *          nothing is pending because the VIM line is level-sensitive.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART1 serial driver identifier.*/
#if (AM67_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/* Bounded low-level TX diagnostics (GemstoneO1R5F bring-up). See the
   declarations in hal_serial_lld.h for what each counter means. */
volatile uint32_t am67_uart1_notify_count;
volatile uint32_t am67_uart1_ier_after_notify;
volatile uint32_t am67_uart1_isr_count;
volatile uint32_t am67_uart1_thre_count;
volatile uint32_t am67_uart1_lsr_at_thre;
volatile uint32_t am67_uart1_load_fifo_count;
volatile uint32_t am67_uart1_bytes_dequeued;
volatile uint32_t am67_uart1_thr_writes;
volatile uint32_t am67_uart1_iir_last;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline uint32_t u_getreg(SerialDriver *sdp, uint32_t offset) {

  return *(volatile uint32_t *)(sdp->base + offset);
}

static inline void u_putreg(SerialDriver *sdp, uint32_t offset,
                            uint32_t value) {

  *(volatile uint32_t *)(sdp->base + offset) = value;
}

/**
 * @brief   UART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t divisor;

  /* Divisor for 16x mode, rounded to nearest.*/
  divisor = (sdp->clock + (8U * config->speed)) / (16U * config->speed);

  /* UART into disabled mode while reconfiguring (TI-specific).*/
  u_putreg(sdp, UART_MDR1_OFFSET, UART_MDR1_MODE_DISABLE);

  /* Clear the FIFOs.*/
  u_putreg(sdp, UART_FCR_OFFSET, UART_FCR_RXRST | UART_FCR_TXRST);

  /* Frame format 8N1, divisor latch open.*/
  u_putreg(sdp, UART_LCR_OFFSET, UART_LCR_8N1 | UART_LCR_DLAB);

  /* Baud divisor.*/
  u_putreg(sdp, UART_DLM_OFFSET, (divisor >> 8) & 0xFFU);
  u_putreg(sdp, UART_DLL_OFFSET, divisor & 0xFFU);

  /* Divisor latch closed, frame format stays 8N1.*/
  u_putreg(sdp, UART_LCR_OFFSET, UART_LCR_8N1);

  /* FIFOs enabled and cleared, RX trigger at 1 character.*/
  u_putreg(sdp, UART_FCR_OFFSET, UART_FCR_FIFOEN | UART_FCR_RXRST |
                                 UART_FCR_TXRST | UART_FCR_RXTRIGGER_1);

  /* RX and line status interrupts on, TX interrupt is enabled on demand
     by the output queue notification.*/
  u_putreg(sdp, UART_IER_OFFSET, UART_IER_ERBFI | UART_IER_ELSI);

  /* UART 16x mode last, this is what actually turns the UART on.*/
  u_putreg(sdp, UART_MDR1_OFFSET, UART_MDR1_MODE_UART16X);
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
static void uart_deinit(SerialDriver *sdp) {

  u_putreg(sdp, UART_IER_OFFSET, 0U);
  u_putreg(sdp, UART_MDR1_OFFSET, UART_MDR1_MODE_DISABLE);
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] lsr       UART LSR register value
 */
static void set_error(SerialDriver *sdp, uint32_t lsr) {
  eventflags_t sts = 0U;

  if ((lsr & UART_LSR_OE) != 0U) {
    sts |= SD_OVERRUN_ERROR;
  }
  if ((lsr & UART_LSR_PE) != 0U) {
    sts |= SD_PARITY_ERROR;
  }
  if ((lsr & UART_LSR_FE) != 0U) {
    sts |= SD_FRAMING_ERROR;
  }
  if ((lsr & UART_LSR_BI) != 0U) {
    sts |= SD_BREAK_DETECTED;
  }
  osalSysLockFromISR();
  chnAddFlagsI(sdp, sts);
  osalSysUnlockFromISR();
}

/**
 * @brief   Loads the TX FIFO from the output queue.
 * @details Must be called from within a lock zone. Fills whatever room the
 *          TX FIFO has (SSR reports full, TI extension) and disables the
 *          THR empty interrupt when the output queue runs dry. The THRE
 *          interrupt honors the TX FIFO trigger level, so it can fire with
 *          bytes still queued in the FIFO: assuming an empty FIFO here and
 *          writing a full 64 bytes would overflow it and drop characters.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
static void load_tx_fifo(SerialDriver *sdp) {
  uint32_t n;

  am67_uart1_load_fifo_count++;

  for (n = 0U; n < UART_TX_FIFO_DEPTH; n++) {
    msg_t b;

    if ((u_getreg(sdp, UART_SSR_OFFSET) & UART_SSR_TXFIFOFULL) != 0U) {
      break;
    }
    b = sdRequestDataI(sdp);
    if (b < MSG_OK) {
      u_putreg(sdp, UART_IER_OFFSET,
               u_getreg(sdp, UART_IER_OFFSET) & ~UART_IER_ETBEI);
      break;
    }
    am67_uart1_bytes_dequeued++;
    u_putreg(sdp, UART_THR_OFFSET, (uint32_t)b);
    am67_uart1_thr_writes++;
  }
}

#if (AM67_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;

  am67_uart1_notify_count++;

  u_putreg(&SD1, UART_IER_OFFSET,
           u_getreg(&SD1, UART_IER_OFFSET) | UART_IER_ETBEI);
  am67_uart1_ier_after_notify = u_getreg(&SD1, UART_IER_OFFSET);

  /* The THR empty interrupt fires on a level transition only: when the
     transmitter is already idle there is no transition and nothing would
     ever start, feed the FIFO directly (NuttX u16550_txint() applies the
     same workaround, "fake a TX interrupt"). Safe with the transmitter in
     any state, the fill loop checks for FIFO room per byte.*/
  load_tx_fifo(&SD1);
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (AM67_SERIAL_USE_UART1 == TRUE) || defined(__DOXYGEN__)
static bool uart1_irq_handler(void *arg) {
  bool preemption_required;

  (void)arg;

  sd_lld_serve_interrupt(&SD1);

  chSysLockFromISR();
  preemption_required = chSchIsPreemptionRequired();
  chSysUnlockFromISR();

  return preemption_required;
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Common IRQ handler.
 * @details Services pending conditions by IIR identification until the
 *          UART reports nothing pending, the VIM line is level-sensitive
 *          so returning with a condition still asserted would re-enter
 *          immediately.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 */
void sd_lld_serve_interrupt(SerialDriver *sdp) {
  uint32_t iir;

  am67_uart1_isr_count++;

  while (((iir = u_getreg(sdp, UART_IIR_OFFSET)) &
          UART_IIR_INTSTATUS) == 0U) {

    am67_uart1_iir_last = iir;

    switch (iir & UART_IIR_INTID_MASK) {
    case UART_IIR_INTID_RLS:
      /* Line status: reading LSR clears the condition.*/
      set_error(sdp, u_getreg(sdp, UART_LSR_OFFSET));
      break;

    case UART_IIR_INTID_RDA:
    case UART_IIR_INTID_CTI:
      /* Received data available / character timeout: drain the RX FIFO
         into the input queue.*/
      osalSysLockFromISR();
      while ((u_getreg(sdp, UART_LSR_OFFSET) & UART_LSR_DR) != 0U) {
        sdIncomingDataI(sdp, (uint8_t)(u_getreg(sdp, UART_RBR_OFFSET) &
                                       0xFFU));
      }
      osalSysUnlockFromISR();
      break;

    case UART_IIR_INTID_THRE:
      /* TX holding register empty: refill up to a FIFO worth of data,
         disable the TX interrupt when the output queue runs dry.*/
      am67_uart1_thre_count++;
      am67_uart1_lsr_at_thre = u_getreg(sdp, UART_LSR_OFFSET);
      osalSysLockFromISR();
      load_tx_fifo(sdp);
      osalSysUnlockFromISR();
      break;

    default:
      /* Modem status or unexpected identification: reading MSR clears
         the modem condition.*/
      (void)u_getreg(sdp, UART_MSR_OFFSET);
      break;
    }
  }
}

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if AM67_SERIAL_USE_UART1 == TRUE
  sdObjectInit(&SD1, NULL, notify1);
  SD1.base  = AM67_UART1_BASE;
  SD1.clock = AM67_UART1_CLOCK;
  vim_set_handler(AM67_UART1_IRQ, uart1_irq_handler, NULL);
  vim_set_priority(AM67_UART1_IRQ, AM67_SERIAL_UART1_IRQ_PRIORITY);
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL) {
    config = &default_config;
  }

  uart_init(sdp, config);

#if AM67_SERIAL_USE_UART1 == TRUE
  if (sdp == &SD1) {
    vim_enable_irq(AM67_UART1_IRQ);
  }
#endif
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
#if AM67_SERIAL_USE_UART1 == TRUE
    if (sdp == &SD1) {
      vim_disable_irq(AM67_UART1_IRQ);
    }
#endif
    uart_deinit(sdp);
  }
}

/**
 * @brief   One-shot polled TX test for UART1 (GemstoneO1R5F bring-up).
 * @details Bypasses the output queue and the TX interrupt entirely: waits
 *          on LSR THRE and writes THR directly, per byte. Isolates the
 *          physical TX path (base address, pinmux, baud divisor, MDR1
 *          mode) from the notify1()/ETBEI/ISR-driven path. Bounded wait per
 *          byte so a stuck THRE cannot hang the caller forever.
 *
 *          Temporary diagnostic. Not the production TX path -- normal
 *          traffic keeps using the interrupt-driven queue via sdWrite().
 *
 * @param[in] data      bytes to send
 * @param[in] len       number of bytes
 * @return              number of bytes actually written; < len only if a
 *                       THRE wait timed out (transmitter never went idle).
 *
 * @notapi
 */
uint32_t am67_uart1_poll_tx(const uint8_t *data, uint32_t len) {
  uint32_t i;

  for (i = 0U; i < len; i++) {
    uint32_t tries = 0U;

    while ((u_getreg(&SD1, UART_LSR_OFFSET) & UART_LSR_THRE) == 0U) {
      if (++tries > 1000000U) {
        return i;
      }
    }
    u_putreg(&SD1, UART_THR_OFFSET, (uint32_t)data[i]);
  }
  return len;
}

/**
 * @brief   Drain queued TX bytes into the UART FIFO.
 * @details See the comment on the declaration in hal_serial_lld.h. Safe to
 *          call at any time; does nothing when the output queue is empty.
 *
 * @return  number of bytes still queued after pumping.
 *
 * @notapi
 */
uint32_t am67_uart1_tx_pump(void) {
  size_t remaining;

  osalSysLock();
  if (oqGetFullI(&SD1.oqueue) > (size_t)0) {
    load_tx_fifo(&SD1);
  }
  remaining = oqGetFullI(&SD1.oqueue);
  osalSysUnlock();

  return (uint32_t)remaining;
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
