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
 * @file    UARTv1/hal_sio_lld.h
 * @brief   AM67 SIO subsystem low level driver header.
 *
 * @addtogroup HAL_SIO
 * @{
 */

#ifndef HAL_SIO_LLD_H
#define HAL_SIO_LLD_H

#if (HAL_USE_SIO == TRUE) || defined(__DOXYGEN__)

#include "ti_uart.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    AM67 configuration options
 * @{
 */
/**
 * @brief   SIO driver 1 enable switch.
 * @details If set to @p TRUE the support for MAIN_UART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(AM67_SIO_USE_UART1) || defined(__DOXYGEN__)
#define AM67_SIO_USE_UART1                  FALSE
#endif

/**
 * @brief   MAIN_UART1 interrupt priority level setting.
 */
#if !defined(AM67_SIO_UART1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AM67_SIO_UART1_IRQ_PRIORITY         8U
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (AM67_SIO_USE_UART1 == TRUE) && (AM67_HAS_MAIN_UART1 == FALSE)
#error "MAIN_UART1 not present in the selected device"
#endif

#if (AM67_SIO_USE_UART1 == FALSE)
#error "SIO driver activated but no UART peripheral assigned"
#endif

#if !defined(AM67_MAIN_UART1_CLOCK)
#error "AM67_MAIN_UART1_CLOCK not defined in board.h"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Low level fields of the SIO driver structure.
 * @note    Runaway guard: the VIM line is level sensitive, so a source the
 *          handler cannot actually silence re-enters it immediately and,
 *          because an ARMv7-R core takes interrupts with IRQ masked, starves
 *          every lower-priority activity including the system tick. Masking
 *          the source in IER is the normal mechanism and is kept, but it is
 *          not trusted as the only one: consecutive handler entries with no
 *          application-side progress are counted and the line itself is
 *          taken down when the count runs away. The condition is latched so
 *          that a driver which lost its interrupts says so instead of
 *          looking merely idle.
 * @note    TX-end machinery: the 16550 reports THR/FIFO empty through
 *          ETBEI, it has no transmission-complete interrupt. THRE becomes
 *          true while the last character is still in the shift register,
 *          so TEMT has to be polled to observe the physical end of
 *          transmission. The polling virtual timer is armed, re-armed and
 *          reset only from the interrupt handler, from the timer callback
 *          itself and from the stop path, as required by the virtual timer
 *          ownership rule. The write paths never touch it: they mark a
 *          transmission outstanding and let the handler do the detection.
 *          Virtual timers are an RT facility, so without the RT kernel
 *          only the opportunistic check in the handler is available and a
 *          transmission whose TEMT is still clear when THRE fires is not
 *          signaled.
 */
#define sio_lld_driver_fields                                               \
  /* Pointer to the UARTx registers block.*/                                \
  TI_UART_TypeDef           *uart;                                          \
  /* Interrupt line associated to the peripheral.*/                         \
  uint32_t                  irq;                                            \
  /* Functional clock frequency for the associated UART.*/                  \
  uint32_t                  clock;                                          \
  /* Shadow of the IER register, the peripheral overlays IER and DLH so a   \
     read-modify-write is only safe while DLAB is known to be zero.*/       \
  uint32_t                  ier;                                            \
  /* Sticky line status bits captured by the handler, LSR clears on read    \
     so the errors would otherwise be lost before the driver asks.*/        \
  uint32_t                  lsr;                                            \
  /* Set while a started transmission has not reached the wire yet, see    \
     the TX-end notes above.*/                                              \
  bool                      txend_pending;                                  \
  /* TX-end polling virtual timer, see the TX-end notes above.*/            \
  virtual_timer_t           txend_vt;                                       \
  /* TX-end polling interval, one character time at the current baud.*/     \
  sysinterval_t             txend_step;                                     \
  /* Handler entries not yet answered by an application-side path, see the  \
     runaway note above.*/                                                  \
  uint32_t                  unserviced;                                     \
  /* Set when the runaway guard had to take the interrupt line down.*/      \
  bool                      runaway

/**
 * @brief   Low level fields of the SIO configuration structure.
 */
#define sio_lld_config_fields                                               \
  /* Desired baud rate.*/                                                   \
  uint32_t                  baud;                                           \
  /* UART LCR register initialization data.*/                               \
  uint32_t                  lcr;                                            \
  /* UART FCR register initialization data.*/                               \
  uint32_t                  fcr

/**
 * @brief   Determines if RX has pending error events to be read and cleared.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The RX error events.
 * @retval false        if RX has no pending events
 * @retval true         if RX has pending events
 *
 * @notapi
 */
#define sio_lld_has_rx_errors(siop)                                         \
  (bool)(((siop)->lsr & TI_UART_LSR_RX_ERRORS) != 0U)

/**
 * @brief   Determines the state of the TX FIFO.
 *
 * @param[in] siop      pointer to the @p SIODriver object
 * @return              The TX FIFO state.
 * @retval false        if TX FIFO is not full
 * @retval true         if TX FIFO is full
 *
 * @notapi
 */
#define sio_lld_is_tx_full(siop)                                            \
  (bool)(((siop)->uart->SSR & TI_UART_SSR_TXFIFOFULL) != 0U)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (AM67_SIO_USE_UART1 == TRUE) && !defined(__DOXYGEN__)
extern SIODriver SIOD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sio_lld_init(void);
  msg_t sio_lld_start(SIODriver *siop);
  void sio_lld_stop(SIODriver *siop);
  const SIOConfig *sio_lld_setcfg(SIODriver *siop, const SIOConfig *config);
  const hal_sio_config_t *sio_lld_selcfg(SIODriver *siop,
                                         unsigned cfgnum);
  bool sio_lld_is_rx_empty(SIODriver *siop);
  bool sio_lld_is_rx_idle(SIODriver *siop);
  bool sio_lld_is_tx_ongoing(SIODriver *siop);
  void sio_lld_update_enable_flags(SIODriver *siop);
  sioevents_t sio_lld_get_and_clear_errors(SIODriver *siop);
  sioevents_t sio_lld_get_and_clear_events(SIODriver *siop, sioevents_t events);
  sioevents_t sio_lld_get_events(SIODriver *siop);
  size_t sio_lld_read(SIODriver *siop, uint8_t *buffer, size_t n);
  size_t sio_lld_write(SIODriver *siop, const uint8_t *buffer, size_t n);
  msg_t sio_lld_get(SIODriver *siop);
  void sio_lld_put(SIODriver *siop, uint_fast16_t data);
  msg_t sio_lld_control(SIODriver *siop, unsigned int operation, void *arg);
  void sio_lld_serve_interrupt(SIODriver *siop);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SIO == TRUE */

#endif /* HAL_SIO_LLD_H */

/** @} */
