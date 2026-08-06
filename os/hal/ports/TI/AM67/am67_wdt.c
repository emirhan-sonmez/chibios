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
 * @file    TI/AM67/am67_wdt.c
 * @brief   MCU RTI windowed watchdog (DWWD) for the AM67 R5F.
 * @details M9. A crashed or livelocked R5F leaves the PWM peripherals emitting
 *          their last commanded pulse width indefinitely -- see Safety and
 *          Failsafes 3.1. Nothing downstream catches it: Linux cannot see the
 *          fault, and the Q-32 investigation showed the core can stop making
 *          progress while the kernel tick, the SPI thread and the log thread
 *          all keep running, so "the firmware is alive" is not the same
 *          question as "the flight loop is advancing".
 *
 *          Hardware: MCU_RTI at 0x04880000, compatible ti,j7-rti-wdt. Its
 *          device-tree status is "reserved", which on K3 means the peripheral
 *          is assigned to a remote core rather than to Linux -- no Linux
 *          driver binds it, so it is ours to take with no ownership handover
 *          and no dual-master hazard.
 *
 *          Register offsets and magic values are taken from the TI MCU+ SDK
 *          (source/drivers/watchdog/v0/cslr_rti.h), not from prose.
 *
 * @par Two-stage bring-up, on purpose
 *      The DWWD timeout is (RTIDWDPRLD + 1) * 2^13 / RTICLK, and RTICLK for
 *      this instance is set by device-tree clock parents that this firmware
 *      does not configure and cannot read back directly. A watchdog armed
 *      against a guessed clock either never fires or resets the board in a
 *      loop -- and a reset loop on a board whose only recovery path is a
 *      physical power cycle (Q-39) is an expensive mistake.
 *
 *      So am67_wdt_measure_clock() runs first and reports the real tick rate
 *      by watching the down-counter across a known interval. Arming is a
 *      separate call that takes a timeout in milliseconds and converts it
 *      using the measured value.
 *
 * @addtogroup AM67_WDT
 * @{
 */

#include "hal.h"
#include "am67_wdt.h"

#if (AM67_USE_WDT == TRUE) || defined(__DOXYGEN__)

/* MCU domain RTI. Same domain as MCU_MCSPI0 (0x04b00000) and MCU_GPIO0
   (0x04201000), so it is directly addressable from this core. */
#define AM67_MCU_RTI_BASE       0x04880000U

/* From cslr_rti.h. */
#define RTI_GCTRL               0x00U
#define RTI_DWDCTRL             0x90U
#define RTI_DWDPRLD             0x94U
#define RTI_WDSTATUS            0x98U
#define RTI_WDKEY               0x9CU
#define RTI_DWDCNTR             0xA0U
#define RTI_WWDRXNCTRL          0xA4U
#define RTI_WWDSIZECTRL         0xA8U

#define RTI_DWDCTRL_ENABLE      0xA98559DAU
#define RTI_DWDCTRL_DISABLE     0x5312ACEDU
#define RTI_WDKEY_FIRST         0x0000E51AU
#define RTI_WDKEY_SECOND        0x0000A35CU
#define RTI_WWDSIZE_100_PERCENT 0x00000005U
#define RTI_WWDRXN_RESET        0x00000005U

/* The down-counter decrements once per 2^13 RTICLK ticks. */
#define RTI_DWD_PRESCALE        8192U

static uint32_t wdt_clock_hz;
static bool     wdt_armed;

static inline uint32_t rti_get(uint32_t off) {

  return *(volatile uint32_t *)(AM67_MCU_RTI_BASE + off);
}

static inline void rti_put(uint32_t off, uint32_t value) {

  *(volatile uint32_t *)(AM67_MCU_RTI_BASE + off) = value;
}

uint32_t am67_wdt_measure_clock(void) {
  uint32_t first, second;
  const uint32_t window_ms = 200U;

  /* Preload the maximum so the counter cannot reach zero during the
     measurement, and keep the watchdog itself disabled throughout: this
     function must never be able to reset the board. */
  rti_put(RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);
  rti_put(RTI_DWDPRLD, 0xFFFU);
  rti_put(RTI_WWDSIZECTRL, RTI_WWDSIZE_100_PERCENT);

  /* Servicing reloads the down-counter from RTIDWDPRLD. */
  rti_put(RTI_WDKEY, RTI_WDKEY_FIRST);
  rti_put(RTI_WDKEY, RTI_WDKEY_SECOND);

  /* The counter only runs while the watchdog is enabled, so enable it with a
     preload far larger than the measurement window. 0xFFF preloads
     4096 * 8192 = 33.5M RTICLK ticks; even at 200 MHz that is 168 ms per...
     no: it is 33.5M ticks, which at any plausible RTICLK is far longer than
     the 200 ms window below. */
  rti_put(RTI_DWDCTRL, RTI_DWDCTRL_ENABLE);

  first = rti_get(RTI_DWDCNTR);
  chThdSleepMilliseconds(window_ms);
  second = rti_get(RTI_DWDCNTR);

  rti_put(RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);

  if (first <= second) {
    /* Counter not running, or it wrapped. Either way the result is not
       usable and arming must be refused rather than guessed at. */
    wdt_clock_hz = 0U;
    return 0U;
  }

  /* ticks = counts * prescale, over window_ms milliseconds. */
  wdt_clock_hz = ((first - second) * RTI_DWD_PRESCALE) * (1000U / window_ms);
  return wdt_clock_hz;
}

bool am67_wdt_start(uint32_t timeout_ms) {
  uint32_t preload;

  if (wdt_clock_hz == 0U) {
    return false;
  }

  /* timeout = (PRLD + 1) * 2^13 / RTICLK, so PRLD = timeout * RTICLK / 2^13 - 1.
     Computed in 64-bit: timeout_ms * clock overflows 32 bits for any
     realistic clock beyond about a second. */
  {
    const uint64_t ticks = ((uint64_t)timeout_ms * (uint64_t)wdt_clock_hz) / 1000ULL;
    const uint64_t prld  = (ticks / RTI_DWD_PRESCALE);

    if ((prld == 0ULL) || (prld > 0x1000ULL)) {
      return false;                     /* outside what DWDPRLD can express */
    }
    preload = (uint32_t)(prld - 1ULL);
  }

  rti_put(RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);
  rti_put(RTI_DWDPRLD, preload);

  /* 100% window: servicing at any point in the period is accepted. A narrower
     window also catches a loop running too FAST, which is not a failure this
     aircraft has, and would turn ordinary jitter into a reset. */
  rti_put(RTI_WWDSIZECTRL, RTI_WWDSIZE_100_PERCENT);

  /* Reaction: reset. An NMI handler that parked the outputs would react in
     microseconds instead of a reboot, and is the better long-term answer --
     but it has to run inside the firmware that just failed, and the Q-32
     livelock showed that state is not always trustworthy. A reset is
     unconditional. Revisit once there is a proven safe-state handler. */
  rti_put(RTI_WWDRXNCTRL, RTI_WWDRXN_RESET);

  rti_put(RTI_WDKEY, RTI_WDKEY_FIRST);
  rti_put(RTI_WDKEY, RTI_WDKEY_SECOND);

  rti_put(RTI_DWDCTRL, RTI_DWDCTRL_ENABLE);
  wdt_armed = true;
  return true;
}

void am67_wdt_service(void) {

  if (!wdt_armed) {
    return;
  }
  rti_put(RTI_WDKEY, RTI_WDKEY_FIRST);
  rti_put(RTI_WDKEY, RTI_WDKEY_SECOND);
}

void am67_wdt_stop(void) {

  rti_put(RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);
  wdt_armed = false;
}

bool am67_wdt_is_armed(void) {

  return wdt_armed;
}

uint32_t am67_wdt_status(void) {

  return rti_get(RTI_WDSTATUS);
}

uint32_t am67_wdt_counter(void) {

  return rti_get(RTI_DWDCNTR);
}

#endif /* AM67_USE_WDT */

/** @} */
