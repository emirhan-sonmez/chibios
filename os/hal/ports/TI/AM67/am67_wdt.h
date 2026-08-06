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
 * @file    TI/AM67/am67_wdt.h
 * @brief   MCU RTI windowed watchdog (DWWD). See am67_wdt.c for why arming is
 *          split from clock measurement.
 */

#ifndef AM67_WDT_H
#define AM67_WDT_H

#include <stdint.h>
#include <stdbool.h>

#if !defined(AM67_USE_WDT)
#define AM67_USE_WDT TRUE
#endif

#ifdef __cplusplus
extern "C" {
#endif

  /**
   * @brief   Measure RTICLK by watching the DWWD down-counter.
   * @details Leaves the watchdog DISABLED. Must be called before
   *          am67_wdt_start(), which refuses to arm without it: the DWWD
   *          timeout depends on a clock this firmware does not configure, and
   *          arming against a guessed rate risks a reset loop on a board whose
   *          only recovery is a physical power cycle (Q-39).
   * @return  Measured RTICLK in Hz, or 0 if the counter did not advance.
   */
  uint32_t am67_wdt_measure_clock(void);

  /**
   * @brief   Arm the watchdog with a timeout in milliseconds.
   * @return  False if the clock is unknown or the timeout cannot be expressed.
   */
  bool am67_wdt_start(uint32_t timeout_ms);

  /** @brief  Reload the counter. No-op unless armed. */
  void am67_wdt_service(void);

  /** @brief  Disarm. */
  void am67_wdt_stop(void);

  bool     am67_wdt_is_armed(void);
  uint32_t am67_wdt_status(void);
  uint32_t am67_wdt_counter(void);

#ifdef __cplusplus
}
#endif

#endif /* AM67_WDT_H */
