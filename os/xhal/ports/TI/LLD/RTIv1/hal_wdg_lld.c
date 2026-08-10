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
 * @file    RTIv1/hal_wdg_lld.c
 * @brief   AM67 (J722S) MCU RTI windowed watchdog (DWWD) low level driver
 *          source.
 * @details A crashed or livelocked R5F leaves the PWM peripherals emitting
 *          their last commanded pulse width indefinitely, and nothing
 *          downstream catches it: Linux cannot see the fault, and this
 *          port's own livelock investigation (Q-32) showed the core can
 *          stop making progress while the kernel tick, the SPI thread and
 *          the log thread all keep running -- "the firmware is alive" is
 *          not the same question as "the flight loop is advancing".
 *
 *          Hardware: MCU_RTI at @p AM67_MCU_RTI0_BASE, compatible
 *          ti,j7-rti-wdt. Its device-tree status is "reserved", which on
 *          K3 means the peripheral is assigned to a remote core rather
 *          than Linux -- no Linux driver binds it, so it is ours to take
 *          with no ownership handover and no dual-master hazard.
 *
 * @par Two-stage bring-up, on purpose
 *      The DWWD timeout is (RTIDWDPRLD + 1) * 2^13 / RTICLK, and RTICLK is
 *      set by device-tree clock parents this firmware does not configure
 *      and cannot read back directly. Arming against a guessed clock
 *      either never fires or resets the board in a loop -- and a reset
 *      loop on a board whose only recovery path is a physical power cycle
 *      is an expensive mistake. So @p wdg_lld_start() measures the real
 *      tick rate by watching the down-counter across a known interval
 *      before ever touching the arming registers, and refuses to arm if
 *      the measurement is not usable.
 *
 * @addtogroup WDG
 * @{
 */

#include "hal.h"

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* Length of the clock-measurement window. Long enough that the down-counter
   (decrementing once per RTICLK tick) moves a countable amount even at a
   slow RTICLK, short enough not to stall driver start noticeably.*/
#define WDT_MEASURE_WINDOW_MS    200U

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

#if (AM67_WDG_USE_RTI0 == TRUE) || defined(__DOXYGEN__)
hal_wdg_driver_c WDGD1;
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline uint32_t rti_get(hal_wdg_driver_c *wdgp, uint32_t off) {

  return *(volatile uint32_t *)(wdgp->base + off);
}

static inline void rti_put(hal_wdg_driver_c *wdgp, uint32_t off,
                           uint32_t value) {

  *(volatile uint32_t *)(wdgp->base + off) = value;
}

/**
 * @brief   Measures RTICLK by watching the DWWD down-counter.
 * @details Leaves the watchdog DISABLED throughout -- this function must
 *          never be able to reset the board. Servicing (the key sequence)
 *          reloads the down-counter from RTIDWDPRLD, and the counter only
 *          runs while the watchdog is enabled, so it is enabled here with
 *          a preload far larger than the measurement window: 0xFFF
 *          preloads 4096 * 8192 = ~33.5M RTICLK ticks, which outlasts
 *          @p WDT_MEASURE_WINDOW_MS at any plausible RTICLK.
 *
 * @param[in] wdgp      pointer to the @p hal_wdg_driver_c object
 * @return              Measured RTICLK in Hz, or 0 if the counter did not
 *                      advance (not running, or it wrapped).
 */
static uint32_t wdt_measure_clock(hal_wdg_driver_c *wdgp) {
  uint32_t first, second;

  rti_put(wdgp, RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);
  rti_put(wdgp, RTI_DWDPRLD, 0xFFFU);
  rti_put(wdgp, RTI_WWDSIZECTRL, RTI_WWDSIZE_100_PERCENT);

  rti_put(wdgp, RTI_WDKEY, RTI_WDKEY_FIRST);
  rti_put(wdgp, RTI_WDKEY, RTI_WDKEY_SECOND);

  rti_put(wdgp, RTI_DWDCTRL, RTI_DWDCTRL_ENABLE);

  first = rti_get(wdgp, RTI_DWDCNTR);
  chThdSleepMilliseconds(WDT_MEASURE_WINDOW_MS);
  second = rti_get(wdgp, RTI_DWDCNTR);

  rti_put(wdgp, RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);

  if (first <= second) {
    return 0U;
  }

  /* DWDCNTR counts RTICLK directly, so the delta IS the tick count over the
     window -- no prescale factor. See the note on the preload above.*/
  return (first - second) * (1000U / WDT_MEASURE_WINDOW_MS);
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level WDG driver initialization.
 *
 * @notapi
 */
void wdg_lld_init(void) {

#if AM67_WDG_USE_RTI0 == TRUE
  wdgObjectInit(&WDGD1);
  WDGD1.base     = AM67_MCU_RTI0_BASE;
  WDGD1.clock_hz = 0U;
#endif
}

/**
 * @brief   Measures RTICLK and arms the watchdog.
 * @details Blocks for @p WDT_MEASURE_WINDOW_MS (~200 ms) measuring the
 *          clock before arming -- see the file header. Requires the
 *          scheduler to already be running (chThdSleepMilliseconds()),
 *          same requirement every other driver in this port that blocks
 *          during start has.
 *
 * @param[in] wdgp      pointer to the @p hal_wdg_driver_c object
 * @return              The operation status.
 *
 * @notapi
 */
msg_t wdg_lld_start(hal_wdg_driver_c *wdgp) {
  const hal_wdg_config_t *config = (const hal_wdg_config_t *)wdgp->config;
  uint64_t ticks, prld;

  chDbgAssert(config != NULL, "config missing");

  wdgp->clock_hz = wdt_measure_clock(wdgp);
  if (wdgp->clock_hz == 0U) {
    return HAL_RET_HW_FAILURE;
  }

  /* Timeout = (PRLD + 1) * 2^13 / RTICLK, so PRLD = timeout * RTICLK / 2^13 - 1.
     Computed in 64-bit: timeout_ms * clock overflows 32 bits for any
     realistic clock beyond about a second.*/
  ticks = ((uint64_t)config->timeout_ms * (uint64_t)wdgp->clock_hz) / 1000ULL;
  prld  = ticks / RTI_DWD_PRESCALE;
  if ((prld == 0ULL) || (prld > 0x1000ULL)) {
    return HAL_RET_CONFIG_ERROR;         /* Outside what DWDPRLD can express.*/
  }

  rti_put(wdgp, RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);
  rti_put(wdgp, RTI_DWDPRLD, (uint32_t)(prld - 1ULL));

  /* 100% window: servicing at any point in the period is accepted. A
     narrower window also catches a loop running too FAST, which is not a
     failure mode this driver is targeting, and would turn ordinary
     scheduling jitter into a spurious reset.*/
  rti_put(wdgp, RTI_WWDSIZECTRL, RTI_WWDSIZE_100_PERCENT);

  /* Reaction: reset. An NMI handler that parked the outputs would react in
     microseconds instead of a reboot, and is the better long-term answer
     -- but it has to run inside the firmware that just failed, and a
     livelock has already shown that state is not always trustworthy at
     that point. A reset is unconditional.*/
  rti_put(wdgp, RTI_WWDRXNCTRL, RTI_WWDRXN_RESET);

  rti_put(wdgp, RTI_WDKEY, RTI_WDKEY_FIRST);
  rti_put(wdgp, RTI_WDKEY, RTI_WDKEY_SECOND);

  rti_put(wdgp, RTI_DWDCTRL, RTI_DWDCTRL_ENABLE);

  return HAL_RET_SUCCESS;
}

/**
 * @brief   Disarms the watchdog.
 * @details Unlike STM32's IWDG, this hardware can be disabled once armed.
 *
 * @param[in] wdgp      pointer to the @p hal_wdg_driver_c object
 *
 * @notapi
 */
void wdg_lld_stop(hal_wdg_driver_c *wdgp) {

  rti_put(wdgp, RTI_DWDCTRL, RTI_DWDCTRL_DISABLE);
}

/**
 * @brief   WDG configuration.
 * @details No board-independent default exists for a watchdog timeout,
 *          same reasoning as SPI/I2C's configuration.
 *
 * @param[in] wdgp      pointer to the @p hal_wdg_driver_c object
 * @param[in] config    pointer to the @p hal_wdg_config_t structure
 * @return              The configuration pointer, or @p NULL if invalid.
 *
 * @notapi
 */
const hal_wdg_config_t *wdg_lld_setcfg(hal_wdg_driver_c *wdgp,
                                       const hal_wdg_config_t *config) {
  (void)wdgp;

  if ((config == NULL) || (config->timeout_ms == 0U)) {
    return NULL;
  }

  return config;
}

/**
 * @brief   Selects one of the pre-defined WDG configurations.
 * @details Always fails: this class has no configurations-array feature
 *          (matches every existing XHAL WDG LLD, STM32's xWDGv1 included)
 *          and, per @p wdg_lld_setcfg(), no board-independent default.
 *
 * @param[in] wdgp      pointer to the @p hal_wdg_driver_c object
 * @param[in] cfgnum    driver configuration number
 * @return              @p NULL, always.
 *
 * @notapi
 */
const hal_wdg_config_t *wdg_lld_selcfg(hal_wdg_driver_c *wdgp,
                                       unsigned cfgnum) {
  (void)wdgp;
  (void)cfgnum;

  return NULL;
}

/**
 * @brief   Reloads the watchdog counter.
 *
 * @param[in] wdgp      pointer to the @p hal_wdg_driver_c object
 *
 * @notapi
 */
void wdg_lld_reset(hal_wdg_driver_c *wdgp) {

  rti_put(wdgp, RTI_WDKEY, RTI_WDKEY_FIRST);
  rti_put(wdgp, RTI_WDKEY, RTI_WDKEY_SECOND);
}

/**
 * @brief   Reads the raw WDSTATUS register.
 * @details Safety-relevant, not a convenience: this is how firmware finds
 *          out on the next boot that the previous run was terminated by
 *          the watchdog rather than a clean restart.
 *
 * @param[in] wdgp      pointer to the @p hal_wdg_driver_c object
 * @return              The raw WDSTATUS register value.
 *
 * @notapi
 */
uint32_t wdg_lld_status(hal_wdg_driver_c *wdgp) {

  return rti_get(wdgp, RTI_WDSTATUS);
}

#endif /* HAL_USE_WDG == TRUE */

/** @} */
