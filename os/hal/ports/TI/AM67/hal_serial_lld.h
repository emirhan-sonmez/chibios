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
 * @file    TI/AM67/hal_serial_lld.h
 * @brief   AM67 (J722S) serial subsystem low level driver header.
 * @details Main-domain UARTs, 16550-compatible with TI extensions. All
 *          registers are 32-bit at stride 4 and must be accessed with full
 *          32-bit reads/writes (K3 interconnect requirement). Register map
 *          from the TI J722S TRM, bit definitions derived from NuttX
 *          include/nuttx/serial/uart_16550.h (Apache-2.0).
 *
 * @addtogroup SERIAL
 * @{
 */

#ifndef HAL_SERIAL_LLD_H
#define HAL_SERIAL_LLD_H

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    UART instances
 * @{
 */
#define AM67_UART1_BASE        0x02810000U
#define AM67_UART1_CLOCK       48000000U   /* Functional clock, Hz.          */
#define AM67_UART1_IRQ         211U        /* R5FSS0 core 0 VIM line.        */
/** @} */

/* Register offsets (byte offsets, 32-bit registers at stride 4) ************/

#define UART_RBR_OFFSET        0x00U  /* (DLAB=0) Receiver Buffer Register   */
#define UART_THR_OFFSET        0x00U  /* (DLAB=0) Transmit Holding Register  */
#define UART_DLL_OFFSET        0x00U  /* (DLAB=1) Divisor Latch LSB          */
#define UART_DLM_OFFSET        0x04U  /* (DLAB=1) Divisor Latch MSB (TI: DLH)*/
#define UART_IER_OFFSET        0x04U  /* (DLAB=0) Interrupt Enable Register  */
#define UART_IIR_OFFSET        0x08U  /* Interrupt ID Register (read)        */
#define UART_FCR_OFFSET        0x08U  /* FIFO Control Register (write)       */
#define UART_LCR_OFFSET        0x0CU  /* Line Control Register               */
#define UART_MCR_OFFSET        0x10U  /* Modem Control Register              */
#define UART_LSR_OFFSET        0x14U  /* Line Status Register                */
#define UART_MSR_OFFSET        0x18U  /* Modem Status Register               */
#define UART_SCR_OFFSET        0x1CU  /* Scratch Pad Register                */
#define UART_MDR1_OFFSET       0x20U  /* Mode Definition Register 1 (TI)     */
#define UART_SSR_OFFSET        0x44U  /* Supplementary Status Register (TI)  */
#define UART_SYSC_OFFSET       0x54U  /* System Configuration Register (TI)  */
#define UART_SYSS_OFFSET       0x58U  /* System Status Register (TI)         */

/* IER (DLAB=0) Interrupt Enable Register ***********************************/

#define UART_IER_ERBFI         (1U << 0)  /* RX data available interrupt     */
#define UART_IER_ETBEI         (1U << 1)  /* THR empty interrupt             */
#define UART_IER_ELSI          (1U << 2)  /* Receiver line status interrupt  */
#define UART_IER_EDSSI         (1U << 3)  /* Modem status interrupt          */

/* IIR Interrupt ID Register (read) *****************************************/

#define UART_IIR_INTSTATUS     (1U << 0)  /* Interrupt pending (active low)  */
#define UART_IIR_INTID_SHIFT   1U
#define UART_IIR_INTID_MASK    (7U << UART_IIR_INTID_SHIFT)
#define UART_IIR_INTID_MSI     (0U << UART_IIR_INTID_SHIFT)
#define UART_IIR_INTID_THRE    (1U << UART_IIR_INTID_SHIFT)
#define UART_IIR_INTID_RDA     (2U << UART_IIR_INTID_SHIFT)
#define UART_IIR_INTID_RLS     (3U << UART_IIR_INTID_SHIFT)
#define UART_IIR_INTID_CTI     (6U << UART_IIR_INTID_SHIFT)

/* FCR FIFO Control Register (write) ****************************************/

#define UART_FCR_FIFOEN        (1U << 0)  /* Enable FIFOs                    */
#define UART_FCR_RXRST         (1U << 1)  /* Clear RX FIFO                   */
#define UART_FCR_TXRST         (1U << 2)  /* Clear TX FIFO                   */
#define UART_FCR_RXTRIGGER_1   (0U << 6)  /* RX trigger level: 1 character   */
#define UART_FCR_RXTRIGGER_4   (1U << 6)
#define UART_FCR_RXTRIGGER_8   (2U << 6)
#define UART_FCR_RXTRIGGER_14  (3U << 6)

/* LCR Line Control Register ************************************************/

#define UART_LCR_WLS_5BIT      0U
#define UART_LCR_WLS_6BIT      1U
#define UART_LCR_WLS_7BIT      2U
#define UART_LCR_WLS_8BIT      3U
#define UART_LCR_STB           (1U << 2)  /* 2 stop bits when set            */
#define UART_LCR_PEN           (1U << 3)  /* Parity enable                   */
#define UART_LCR_EPS           (1U << 4)  /* Even parity select              */
#define UART_LCR_BRK           (1U << 6)  /* Break control                   */
#define UART_LCR_DLAB          (1U << 7)  /* Divisor Latch Access Bit        */

#define UART_LCR_8N1           UART_LCR_WLS_8BIT

/* MCR Modem Control Register ***********************************************/

#define UART_MCR_DTR           (1U << 0)
#define UART_MCR_RTS           (1U << 1)
#define UART_MCR_LPBK          (1U << 4)  /* Internal loopback mode          */

/* LSR Line Status Register *************************************************/

#define UART_LSR_DR            (1U << 0)  /* Data ready (RX FIFO not empty)  */
#define UART_LSR_OE            (1U << 1)  /* Overrun error                   */
#define UART_LSR_PE            (1U << 2)  /* Parity error                    */
#define UART_LSR_FE            (1U << 3)  /* Framing error                   */
#define UART_LSR_BI            (1U << 4)  /* Break indicator                 */
#define UART_LSR_THRE          (1U << 5)  /* TX holding register empty       */
#define UART_LSR_TEMT          (1U << 6)  /* Transmitter empty (shift reg)   */
#define UART_LSR_RXFE          (1U << 7)  /* Error in RX FIFO                */

/* SSR Supplementary Status Register (TI extension) *************************/

#define UART_SSR_TXFIFOFULL    (1U << 0)  /* TX FIFO is full                 */

/* MDR1 Mode Definition Register 1 (TI extension) ***************************/
/* The UART is inert until MDR1 selects an operating mode, regardless of
   the rest of the configuration. Written last on start, first on stop.    */

#define UART_MDR1_MODE_MASK    7U
#define UART_MDR1_MODE_UART16X 0U         /* UART 16x mode (normal)          */
#define UART_MDR1_MODE_DISABLE 7U         /* UART disabled (reset default)   */

/* TX FIFO depth, used to batch refills in the THRE interrupt handler.      */
#define UART_TX_FIFO_DEPTH     64U

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   SD1 driver enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 */
#if !defined(AM67_SERIAL_USE_UART1) || defined(__DOXYGEN__)
#define AM67_SERIAL_USE_UART1  FALSE
#endif

/**
 * @brief   UART1 interrupt priority level setting.
 */
#if !defined(AM67_SERIAL_UART1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AM67_SERIAL_UART1_IRQ_PRIORITY  0x8U
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if AM67_SERIAL_USE_UART1 == FALSE
#error "serial driver activated but no UART peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   AM67 Serial Driver configuration structure.
 * @details An instance of this structure must be passed to @p sdStart()
 *          in order to configure and start a serial driver operations.
 */
typedef struct hal_serial_config {
  /**
   * @brief Bit rate.
   */
  uint32_t                  speed;
  /* End of the mandatory fields.*/
} SerialConfig;

/**
 * @brief   @p SerialDriver specific data.
 */
#define _serial_driver_data                                                 \
  _base_asynchronous_channel_data                                           \
  /* Driver state.*/                                                        \
  sdstate_t                 state;                                          \
  /* Input queue.*/                                                         \
  input_queue_t             iqueue;                                         \
  /* Output queue.*/                                                        \
  output_queue_t            oqueue;                                         \
  /* Input circular buffer.*/                                               \
  uint8_t                   ib[SERIAL_BUFFERS_SIZE];                        \
  /* Output circular buffer.*/                                              \
  uint8_t                   ob[SERIAL_BUFFERS_SIZE];                        \
  /* End of the mandatory fields.*/                                        \
  /* UART registers base address.*/                                        \
  uint32_t                  base;                                          \
  /* Functional clock frequency for the associated UART.*/                 \
  uint32_t                  clock;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (AM67_SERIAL_USE_UART1 == TRUE) && !defined(__DOXYGEN__)
extern SerialDriver SD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sd_lld_init(void);
  void sd_lld_start(SerialDriver *sdp, const SerialConfig *config);
  void sd_lld_stop(SerialDriver *sdp);
  void sd_lld_serve_interrupt(SerialDriver *sdp);

  /*
    Bounded low-level TX-path diagnostics for UART1/SD1 (GemstoneO1R5F
    bring-up, tracking down the "bytes accepted into the queue but never
    reach the wire" fault). ISR-safe: incremented/snapshotted as plain
    volatile counters inside notify1()/sd_lld_serve_interrupt()/
    load_tx_fifo(), never via trace_printf from interrupt context. Read and
    printed from normal thread context after boot.
  */
  extern volatile uint32_t am67_uart1_notify_count;      /* notify1() calls */
  extern volatile uint32_t am67_uart1_ier_after_notify;   /* IER readback right after notify1 sets ETBEI */
  extern volatile uint32_t am67_uart1_isr_count;          /* sd_lld_serve_interrupt() entries (ISR fired) */
  extern volatile uint32_t am67_uart1_thre_count;         /* IIR THRE branch entries */
  extern volatile uint32_t am67_uart1_lsr_at_thre;        /* LSR snapshot at the THRE branch */
  extern volatile uint32_t am67_uart1_load_fifo_count;    /* load_tx_fifo() calls */
  extern volatile uint32_t am67_uart1_bytes_dequeued;     /* bytes pulled from the oqueue */
  extern volatile uint32_t am67_uart1_thr_writes;         /* THR register writes */
  extern volatile uint32_t am67_uart1_iir_last;           /* last IIR value seen in the ISR loop */

  /*
    One-shot polled TX test: bypasses the output queue and the TX interrupt
    entirely, waiting on LSR THRE and writing THR directly per byte. Proves
    (or disproves) the physical TX path -- base address, pinmux, baud
    divisor, MDR1 mode -- independent of notify1()/ETBEI/ISR. Returns the
    number of bytes actually written (< len only if a THRE wait timed out).
    Temporary GemstoneO1R5F bring-up diagnostic, NOT the production TX path.
  */
  uint32_t am67_uart1_poll_tx(const uint8_t *data, uint32_t len);

  /*
    Push any bytes sitting in the SD1 software output queue into the UART
    TX FIFO, and report how many bytes remain queued afterwards.

    Needed because the THR-empty (THRE) interrupt is not currently observed
    to fire on this UART (am67_uart1_thre_count stays 0): the only thing
    that moves bytes queue -> FIFO is notify1(), which runs on a write. So
    once a burst overflows the FIFO, the leftover bytes can only drain if
    something else pumps them -- otherwise the queue stays full, txspace()
    reads 0, the writer stops writing, and no further notify1() ever
    happens. Call this periodically (e.g. once per main loop) so the TX
    path makes progress independently of new writes.
  */
  uint32_t am67_uart1_tx_pump(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SERIAL == TRUE */

#endif /* HAL_SERIAL_LLD_H */

/** @} */
