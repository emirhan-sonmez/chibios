/*
 * Author: Emirhan Sonmez
 * Date: 16.07.2026 (start)
 *
 * @Brief: T3 Gemstone O1 ChibiOS Port
 *  Minimal polling driver for AM67 main-domain UART1 (Stage A bring-up).
 *  Validates the full hardware path (pinmux, clock, UART config, header
 *  pins 8/10) before the real ChibiOS HAL serial driver gets written.
 *
 * @info: Init sequence derived from NuttX drivers/serial/uart_16550.c
 * u16550_setup() and arch/arm/src/am67/{am67_serial.c,am67_pinmux.c}
 * (Apache-2.0). All registers are 32-bit, accessed with full 32-bit
 * reads/writes (K3 interconnect requirement).
 */

#include <stdint.h>

#include "am67_uart.h"

/* Pinmux *******************************************************************/
/* The Gemstone O1 routes UART1 through MCASP0 pads (mux mode 2):
     UART1_RXD <- MCASP0_AFSR  (ball C27), UART1_TXD <- MCASP0_ACLKR (F24).
   Pad registers live in the main-domain PADCFG control module and are
   write-protected: the two KICK registers must be written with the unlock
   values first, or pad writes are silently ignored.*/

#define AM67_PADCFG_CTRL_BASE   0x000F0000U
#define AM67_PADCFG_KICK0       (AM67_PADCFG_CTRL_BASE + 0x1008U)
#define AM67_PADCFG_KICK1       (AM67_PADCFG_CTRL_BASE + 0x5008U)
#define AM67_KICK0_UNLOCK       0x68EF3490U
#define AM67_KICK1_UNLOCK       0xD172BC5AU

#define AM67_PADCFG_BASE        (AM67_PADCFG_CTRL_BASE + 0x4000U)
#define AM67_PAD_MCASP0_AFSR    (AM67_PADCFG_BASE + 0x01ACU)  /* UART1_RXD */
#define AM67_PAD_MCASP0_ACLKR   (AM67_PADCFG_BASE + 0x01B0U)  /* UART1_TXD */

#define AM67_PIN_MODE(m)        ((uint32_t)(m))
#define AM67_PIN_PULL_DISABLE   (1U << 16)
#define AM67_PIN_INPUT_ENABLE   (1U << 18)

/* Register access **********************************************************/

static inline uint32_t uart_getreg(uint32_t offset) {

  return *(volatile uint32_t *)(AM67_UART1_BASE + offset);
}

static inline void uart_putreg(uint32_t offset, uint32_t value) {

  *(volatile uint32_t *)(AM67_UART1_BASE + offset) = value;
}

static inline void reg_write(uint32_t address, uint32_t value) {

  *(volatile uint32_t *)address = value;
}

/* Public functions *********************************************************/

/*
 * Routes UART1 RX/TX to the 40-pin header (pins 10/8). Harmless if the
 * pads are already muxed by the Linux device tree, this just re-writes
 * the same configuration.
 */
void am67_uart1_pinmux(void) {

  reg_write(AM67_PADCFG_KICK0, AM67_KICK0_UNLOCK);
  reg_write(AM67_PADCFG_KICK1, AM67_KICK1_UNLOCK);

  reg_write(AM67_PAD_MCASP0_AFSR,
            AM67_PIN_MODE(2) | AM67_PIN_INPUT_ENABLE | AM67_PIN_PULL_DISABLE);
  reg_write(AM67_PAD_MCASP0_ACLKR,
            AM67_PIN_MODE(2) | AM67_PIN_PULL_DISABLE);
}

/*
 * Configures UART1 for 8N1 polling operation at the baud rate implied by
 * the divisor (clock / (16 * baud), 26 for 115200 @ 48MHz). Translation
 * of NuttX u16550_setup() with the TI MDR1 mode dance around it.
 */
void am67_uart1_init(uint32_t divisor) {

  /* UART into disabled mode while reconfiguring (TI-specific).*/
  uart_putreg(UART_MDR1_OFFSET, UART_MDR1_MODE_DISABLE);

  /* Clear the FIFOs.*/
  uart_putreg(UART_FCR_OFFSET, UART_FCR_RXRST | UART_FCR_TXRST);

  /* Frame format 8N1, divisor latch open.*/
  uart_putreg(UART_LCR_OFFSET, UART_LCR_8N1 | UART_LCR_DLAB);

  /* Baud divisor.*/
  uart_putreg(UART_DLM_OFFSET, (divisor >> 8) & 0xFFU);
  uart_putreg(UART_DLL_OFFSET, divisor & 0xFFU);

  /* Divisor latch closed, frame format stays 8N1.*/
  uart_putreg(UART_LCR_OFFSET, UART_LCR_8N1);

  /* FIFOs enabled and cleared, RX trigger at 1 character.*/
  uart_putreg(UART_FCR_OFFSET, UART_FCR_FIFOEN | UART_FCR_RXRST |
                               UART_FCR_TXRST | UART_FCR_RXTRIGGER_1);

  /* Polling only, all interrupts off.*/
  uart_putreg(UART_IER_OFFSET, 0U);

  /* UART 16x mode last, this is what actually turns the UART on
     (NuttX open_uart() does the same write after setup).*/
  uart_putreg(UART_MDR1_OFFSET, UART_MDR1_MODE_UART16X);
}

uint32_t am67_uart_read_lsr(void) {

  return uart_getreg(UART_LSR_OFFSET);
}

/*
 * Blocking polled transmit: spins until the TX holding register empties.
 */
void am67_uart1_putc(char c) {

  while ((uart_getreg(UART_LSR_OFFSET) & UART_LSR_THRE) == 0U) {
  }
  uart_putreg(UART_THR_OFFSET, (uint32_t)(uint8_t)c);
}

void am67_uart1_puts(const char *s) {

  while (*s != '\0') {
    if (*s == '\n') {
      am67_uart1_putc('\r');
    }
    am67_uart1_putc(*s++);
  }
}

/*
 * Non-blocking polled receive: -1 when the RX FIFO is empty.
 */
int am67_uart1_getc(void) {

  if ((uart_getreg(UART_LSR_OFFSET) & UART_LSR_DR) == 0U) {
    return -1;
  }
  return (int)(uart_getreg(UART_RBR_OFFSET) & 0xFFU);
}
