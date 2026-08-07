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
 * I2C configuration for MCU_I2C0, the 40-pin header instance (pins 3/5).
 * Standard mode: no sensor is wired to this bus on the Gemstone O1 (both
 * onboard sensors are on SPI), so this is a bus-level smoke test only.
 * Confirmed on hardware 2026-08-10 (T3 Gemstone O1): the probe below
 * returned MSG_RESET with I2C_ACK_FAILURE set, the expected clean NACK.
 */
static const I2CConfig i2c0_config = {
  .frequency            = 100000U
};

/*
 * PWM configuration for EPWM0. 1 MHz tick (1 tick = 1 us), 20 ms / 50 Hz
 * frame -- standard RC servo/ESC timing. Only channel A (output 0) is
 * used by this demo.
 * TEMP-DIAG: no scope or servo is attached to this XHAL port yet (PWM
 * conversion is compile-verified only, see the vault's hardware
 * validation backlog).
 * REMOVE-AFTER: a scope trace on EPWM0 output A confirms 1500 us @ 50 Hz.
 */
static const PWMConfig pwm0_config = {
  .frequency            = 1000000U,
  .period               = 20000U,
  .enabled_events       = 0U,
  .channels             = {
    {.mode = PWM_OUTPUT_ACTIVE_HIGH},
    {.mode = PWM_OUTPUT_DISABLED}
  }
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

  am67_spi0_imu_enable();
  drvStart(&SPID1, &spi_icm20948_config);

  spiSelectX(&SPID1);
  (void)spi_lld_polled_exchange(&SPID1, 0x80U);   /* WHO_AM_I reg | read bit.*/
  whoami = (uint8_t)spi_lld_polled_exchange(&SPID1, 0xFFU);
  spiUnselectX(&SPID1);

  trace_printf("SPI WHO_AM_I = 0x%02x\n", whoami);
  console_write("SPI probe done\r\n");
}

/*
 * Attempts a 1-byte read from address 0x50 (common EEPROM address) on
 * MCU_I2C0. Nothing is wired up, so a clean NACK (I2C_ACK_FAILURE) is the
 * expected -- and fine -- outcome; it still proves the async
 * start/interrupt/complete path runs end to end. A hang or an unexpected
 * MSG_RESET without I2C_ACK_FAILURE would flag a driver bug.
 */
static void i2c_probe_bus(void) {
  uint8_t data;
  msg_t msg;

  drvStart(&I2CD1, &i2c0_config);

  msg = i2cMasterReceiveTimeout(&I2CD1, 0x50U, &data, 1U, TIME_MS2I(50));
  if (msg == MSG_OK) {
    trace_printf("I2C probe: got 0x%02x from 0x50\n", data);
  }
  else {
    /* Not i2cGetAndClearErrorsX(): it calls chSysGetStatusAndLockX(), which
       chsys.c only compiles when CH_PORT_SUPPORTS_RECURSIVE_LOCKS == TRUE --
       this ARMv7-R port does not define it, so that call is an unresolved
       symbol here. No transfer is active at this point, so the plain,
       non-locking read is equally correct.*/
    trace_printf("I2C probe: msg=%d errors=0x%02x (NACK expected, nothing wired)\n",
                (int)msg, (unsigned)i2cGetErrorsX(&I2CD1));
  }
  console_write("I2C probe done\r\n");
}

/*
 * Starts EPWM0 channel A at a 1500 us pulse (servo center) and 50 Hz.
 * pwmEnableChannel() is called again every heartbeat tick from the
 * heartbeat thread below -- not redundant, see hal_pwm_lld.c's file
 * header on why this driver reasserts on every write.
 */
static void pwm_probe_servo(void) {

  drvStart(&PWMD1, &pwm0_config);
  pwmEnableChannel(&PWMD1, 0U, 1500U);
  trace_printf("PWM: EPWM0 ch A started, 1500 us @ 50 Hz\n");
  console_write("PWM probe done\r\n");
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
    /* Reasserts EPWM0's shared time base and channel A's action qualifier
       every tick, same as an RCOutput driver's per-frame output write
       would -- see hal_pwm_lld.c's file header.*/
    pwmEnableChannel(&PWMD1, 0U, 1500U);
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

int main(void) {

  /* System initializations:
     - HAL initialization, this also initializes the configured device
       drivers and performs the board-specific initializations.
     - Kernel initialization, the main() function becomes a thread and the
       RTOS is active.*/
  halInit();
  chSysInit();

  trace_init();
  trace_printf("RT-XHAL-GEMSTONE-O1-R5F starting\n");

  /* Console up.*/
  drvStart(&SIOD1, &sio_config);

  console_write("\r\n"
                "ChibiOS/RT on " PLATFORM_NAME "\r\n"
                "board: " BOARD_NAME "\r\n"
                "type characters to have them echoed back\r\n");

  spi_probe_icm20948();
  i2c_probe_bus();
  pwm_probe_servo();

  chThdCreateStatic(waHeartbeat, sizeof (waHeartbeat),
                    NORMALPRIO + 1, heartbeat, NULL);

  echo_loop();
}
