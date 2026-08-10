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

#include <string.h>

#include "ch.h"
#include "hal.h"

#include "trace.h"

/*
 * SIO configuration for the console. The default configuration would do,
 * it is stated here so the demo shows what a board actually selects.
 */
static const SIOConfig sio_config = {
  .baud                 = 115200U,
  .lcr                  = TI_UART_LCR_8N1,
  .fcr                  = TI_UART_FCR_FIFOEN | TI_UART_FCR_RXTRIGGER_8
};

/*
 * SPI configuration for the ICM-20948 on MCSPI0 channel 3 (SPI0_CS3). Mode 3
 * (CPOL=1, CPHA=1) and a conservative 1 MHz -- the datasheet allows up to
 * 7 MHz, but this demo is a WHO_AM_I smoke test, not a throughput test.
 * Confirmed on hardware 2026-08-10 (T3 Gemstone O1): the read below
 * returned 0xEA, the ICM-20948's documented WHO_AM_I.
 */
static const SPIConfig spi_icm20948_config = {
  .mode                 = 0U,
  .speed                = 1000000U,
  .clock_mode           = 3U,
  .cs_channel           = 3U
};

/*
 * Writes a string to the console, blocking until the last character has
 * left the transmitter.
 */
static void console_write(const char *s) {
  size_t n = strlen(s);

  while (n > 0U) {
    size_t wr = sioAsyncWriteX(&SIOD1, (const uint8_t *)s, n);

    s += wr;
    n -= wr;

    if (n > 0U) {
      /* TX FIFO full, let something else run before trying again.*/
      chThdSleepMilliseconds(1);
    }
  }
}

/*
 * Reads the ICM-20948 WHO_AM_I register over SPI channel 3. Proves the
 * select/polled-exchange/unselect path runs end to end without hanging or
 * faulting, and the value confirms the device really answered: 0xEA is the
 * ICM-20948's WHO_AM_I.
 */
static void spi_probe_icm20948(void) {
  uint8_t whoami;

  board_imu_enable();
  drvStart(&SPID1, &spi_icm20948_config);

  spiSelectX(&SPID1);
  (void)spi_lld_polled_exchange(&SPID1, 0x80U);   /* WHO_AM_I reg | read bit.*/
  whoami = (uint8_t)spi_lld_polled_exchange(&SPID1, 0xFFU);
  spiUnselectX(&SPID1);

  trace_printf("SPI WHO_AM_I = 0x%02x\n", whoami);
  console_write("SPI probe done\r\n");
}

/*
 * Blinker thread, proves the scheduler preempts and the tick advances.
 */
static THD_WORKING_AREA(waHeartbeat, 512);
static THD_FUNCTION(heartbeat, arg) {
  unsigned n = 0U;

  (void)arg;

  chRegSetThreadName("heartbeat");

  while (true) {
    trace_printf("heartbeat %u t=%u ms\n", n, (unsigned)chVTGetSystemTimeX());
    console_write("heartbeat\r\n");
    n++;
    chThdSleepMilliseconds(1000);
  }
}

/*
 * Echo loop, proves the receive path and the SIO synchronization API.
 */
static void echo_loop(void) {
  uint8_t c;

  while (true) {
    msg_t msg = sioSynchronizeRX(&SIOD1, TIME_MS2I(1000));

    if (msg == MSG_OK) {
      while (sioAsyncReadX(&SIOD1, &c, 1U) == 1U) {
        (void)sioAsyncWriteX(&SIOD1, &c, 1U);
      }
    }
  }
}

/*
 * Drains the receiver, bounded.
 *
 * The bound is the point of this helper: UART1 RX is muxed to header pin 10
 * with nothing attached to it, and a floating receiver produces a continuous
 * stream of framing errors and random bytes. An open-ended drain never
 * returns on such a line.
 *
 * @return  the number of frames discarded, capped by the bound.
 */
static size_t sio_drain_rx(void) {
  uint8_t d[8];
  size_t total = 0U;
  unsigned i;

  for (i = 0U; i < 64U; i++) {
    size_t k = sioAsyncReadX(&SIOD1, d, sizeof d);

    if (k == 0U) {
      break;
    }
    total += k;
  }
  (void)sioGetAndClearErrorsX(&SIOD1);

  return total;
}

/*
 * Interrupt-driven SIO synchronization check, over the UART's own internal
 * loopback so that it needs no cable and no free pad.
 *
 * This is the counterpart of a polled loopback: it goes through
 * sioSynchronizeTXEnd() and sioSynchronizeRX(), both of which are resumed
 * from the UART interrupt handler, so it fails by timing out if the VIM
 * line is not wired up, if the RX FIFO trigger is set higher than the
 * payload, or if the TX-end detection never observes TEMT.
 *
 * The counts of what was drained before and after enabling the loopback are
 * reported: on a board whose RX pad is left floating they say whether the
 * loopback really isolated the receiver, which decides whether the payload
 * comparison below means anything at all.
 */
static void sio_probe_loopback(void) {
  static const uint8_t tx[] = { 'P', 'I', 'N', 'G' };
  uint8_t rx[sizeof tx];
  size_t pre, post, n;
  msg_t msg;

  /* Anything written before this point has only been accepted by the TX
     FIFO, not put on the wire. Letting it go out first keeps it from
     looping back into the payload.*/
  (void)sioSynchronizeTXEnd(&SIOD1, TIME_MS2I(100));

  pre = sio_drain_rx();

  /* Internal loopback, MCR is not touched by the driver itself.*/
  SIOD1.uart->MCR |= TI_UART_MCR_LPBK;

  post = sio_drain_rx();

  n = sioAsyncWriteX(&SIOD1, tx, sizeof tx);
  if (n != sizeof tx) {
    trace_printf("SIO loopback: short write %u\n", (unsigned)n);
    goto done;
  }

  msg = sioSynchronizeTXEnd(&SIOD1, TIME_MS2I(100));
  if (msg != MSG_OK) {
    trace_printf("SIO loopback: TX end msg=%d\n", (int)msg);
    goto done;
  }

  msg = sioSynchronizeRX(&SIOD1, TIME_MS2I(100));
  if (msg != MSG_OK) {
    trace_printf("SIO loopback: RX msg=%d\n", (int)msg);
    goto done;
  }

  n = sioAsyncReadX(&SIOD1, rx, sizeof rx);
  trace_printf("SIO loopback: drained %u/%u read %u '%c%c%c%c' errors=0x%02x\n",
               (unsigned)pre, (unsigned)post, (unsigned)n,
               rx[0], rx[1], rx[2], rx[3],
               (unsigned)sioGetAndClearErrorsX(&SIOD1));

done:
  /* A driver whose interrupt line was taken down by the runaway guard is
     reported rather than left looking merely idle.*/
  if (SIOD1.runaway) {
    trace_printf("SIO loopback: RUNAWAY, interrupt line disabled\n");
  }

  SIOD1.uart->MCR &= ~TI_UART_MCR_LPBK;
  (void)sio_drain_rx();
  console_write("SIO loopback probe done\r\n");
}

int main(void) {

  /* System initializations:
     - HAL initialization, this also initializes the configured device
       drivers and performs the board-specific initializations.
     - Kernel initialization, the main() function becomes a thread and the
       RTOS is active.*/
  halInit();
  chSysInit();

  trace_init();
  trace_printf("RT-XHAL-T3-GEM-O1 starting\n");

  /* Console up.*/
  drvStart(&SIOD1, &sio_config);

  console_write("\r\n"
                "ChibiOS/RT on " PLATFORM_NAME "\r\n"
                "board: " BOARD_NAME "\r\n"
                "type characters to have them echoed back\r\n");

  sio_probe_loopback();
  spi_probe_icm20948();

  chThdCreateStatic(waHeartbeat, sizeof (waHeartbeat),
                    NORMALPRIO + 1, heartbeat, NULL);

  echo_loop();
}
