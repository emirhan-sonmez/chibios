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
 * @file    RTIv1/hal_wdg_lld.h
 * @brief   AM67 (J722S) MCU RTI windowed watchdog (DWWD) low level driver
 *          header.
 * @details Unlike a typical MCU watchdog, this one's timeout depends on a
 *          clock (RTICLK) this firmware does not configure and cannot read
 *          back directly -- it is set by device-tree clock parents on the
 *          Linux side. Arming against a guessed rate risks a reset loop on
 *          a board whose only recovery is a physical power cycle, so
 *          @p wdg_lld_start() measures RTICLK at runtime (a ~200 ms blocking
 *          wait, see hal_wdg_lld.c) before arming, and refuses to arm if the
 *          measurement is not usable. No configurations array or default
 *          configuration exists for the same reason SPI/I2C have none: a
 *          timeout has no board-independent sane value.
 *
 * @addtogroup WDG
 * @{
 */

#ifndef HAL_WDG_LLD_H
#define HAL_WDG_LLD_H

#if (HAL_USE_WDG == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/* Register offsets, from TI's MCU+ SDK (cslr_rti.h), not from prose.*/
#define RTI_GCTRL                0x00U
#define RTI_DWDCTRL              0x90U
#define RTI_DWDPRLD              0x94U
#define RTI_WDSTATUS             0x98U
#define RTI_WDKEY                0x9CU
#define RTI_DWDCNTR              0xA0U
#define RTI_WWDRXNCTRL           0xA4U
#define RTI_WWDSIZECTRL          0xA8U

#define RTI_DWDCTRL_ENABLE       0xA98559DAU
#define RTI_DWDCTRL_DISABLE      0x5312ACEDU
#define RTI_WDKEY_FIRST          0x0000E51AU
#define RTI_WDKEY_SECOND         0x0000A35CU
#define RTI_WWDSIZE_100_PERCENT  0x00000005U
#define RTI_WWDRXN_RESET         0x00000005U

/* The down-counter decrements once per 2^13 RTICLK ticks.*/
#define RTI_DWD_PRESCALE         8192U

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   WDGD1 driver enable switch.
 * @details If set to @p TRUE the support for MCU_RTI0 is included.
 */
#if !defined(AM67_WDG_USE_RTI0) || defined(__DOXYGEN__)
#define AM67_WDG_USE_RTI0    FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (AM67_WDG_USE_RTI0 == TRUE) && (AM67_HAS_MCU_RTI0 == FALSE)
#error "MCU_RTI0 not present in the selected device"
#endif

#if AM67_WDG_USE_RTI0 == FALSE
#error "WDG driver activated but no RTI peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Low level fields of the WDG driver structure.
 */
#define wdg_lld_driver_fields                                               \
  /* RTI registers base address.*/                                          \
  uint32_t                  base;                                           \
  /* Measured RTICLK in Hz, set by wdg_lld_start(). Zero means unmeasured   \
     (or the measurement failed) -- arming never proceeds past that.*/      \
  uint32_t                  clock_hz

/**
 * @brief   Low level fields of the WDG configuration structure.
 */
#define wdg_lld_config_fields                                               \
  /* Watchdog timeout in milliseconds. Converted against the runtime-       \
     measured RTICLK, not a compile-time constant.*/                        \
  uint32_t                  timeout_ms

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (AM67_WDG_USE_RTI0 == TRUE) && !defined(__DOXYGEN__)
extern hal_wdg_driver_c WDGD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void wdg_lld_init(void);
  msg_t wdg_lld_start(hal_wdg_driver_c *wdgp);
  void wdg_lld_stop(hal_wdg_driver_c *wdgp);
  const hal_wdg_config_t *wdg_lld_setcfg(hal_wdg_driver_c *wdgp,
                                         const hal_wdg_config_t *config);
  const hal_wdg_config_t *wdg_lld_selcfg(hal_wdg_driver_c *wdgp,
                                         unsigned cfgnum);
  void wdg_lld_reset(hal_wdg_driver_c *wdgp);
  /* Diagnostics, not part of the generic WDG class contract. WDSTATUS in
     particular is safety-relevant: it is how firmware finds out on the
     next boot that the previous run was terminated by the watchdog rather
     than a clean restart.*/
  uint32_t wdg_lld_status(hal_wdg_driver_c *wdgp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_WDG == TRUE */

#endif /* HAL_WDG_LLD_H */

/** @} */
