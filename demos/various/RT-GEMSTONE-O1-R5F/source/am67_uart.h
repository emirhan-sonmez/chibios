/*
 * Author: Emirhan Sonmez
 * Date: 16.07.2026 (start)
 *
 * @Brief: T3 Gemstone O1 ChibiOS Port
 *  AM67 (J722S) main-domain UART1, 16550-compatible with TI extensions.
 *
 * @info: Register map from the TI J722S TRM (https://www.ti.com/lit/zip/sprujb3).
 * Register bit definitions derived from NuttX include/nuttx/serial/uart_16550.h
 * (Apache-2.0). All registers are 32-bit and must be accessed with full
 * 32-bit reads/writes (K3 interconnect requirement).
 */

#ifndef AM67_UART_H
#define AM67_UART_H

#include <stdint.h>

/* Instance parameters ******************************************************/

#define AM67_UART1_BASE        0x02810000U
#define AM67_UART1_CLOCK       48000000U   /* Functional clock, Hz.          */

/* Baud divisor for 16x mode: clock / (16 * baud). 115200 -> 26.             */
#define AM67_UART_DIV_115200   26U

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

/* MDR1 Mode Definition Register 1 (TI extension) ***************************/
/* The UART is inert until MDR1 selects an operating mode, regardless of
   the rest of the configuration. NuttX writes this last (open_uart()).     */

#define UART_MDR1_MODE_MASK    7U
#define UART_MDR1_MODE_UART16X 0U         /* UART 16x mode (normal)          */
#define UART_MDR1_MODE_DISABLE 7U         /* UART disabled (reset default)   */

/* SYSC/SYSS System Configuration/Status (TI extension) *********************/

#define UART_SYSC_SOFTRESET    (1U << 1)  /* Write 1 to start module reset   */
#define UART_SYSS_RESETDONE    (1U << 0)  /* Reads 1 when reset completed    */

/* Function prototypes ******************************************************/

void     am67_uart1_pinmux(void);
void     am67_uart1_init(uint32_t divisor);
uint32_t am67_uart_read_lsr(void);
void     am67_uart1_putc(char c);
void     am67_uart1_puts(const char *s);
int      am67_uart1_getc(void);

#endif /* AM67_UART_H */
