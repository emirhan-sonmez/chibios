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
 * @file    TI/AM67/am67_ecap.h
 * @brief   Minimal eCAP APWM output driver for the AM67/J722S.
 * @details Distinct IP and register map from eHRPWM. In APWM mode the 32-bit
 *          time-stamp counter (TSCTR) free-runs and resets at the period; the
 *          active period/compare live in CAP1/CAP2 and are shadowed by
 *          CAP3/CAP4 (which load at the period boundary). No prescale is used
 *          (32-bit counter clocked directly by fck). Register map and the APWM
 *          sequence follow the Linux pwm-tiecap driver and the J722S register
 *          spreadsheet. Pinmux, module clock and power are Linux-owned.
 */

#ifndef AM67_ECAP_H
#define AM67_ECAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
  /* Put the eCAP into APWM mode (active high, free-running) and program the
     period for frame_hz via the shadow registers; 0% duty (output low). The
     counter is expected to already be free-running (Linux enabled it). */
  void ecap_start(uint32_t base, uint32_t frame_hz);

  /* Set the high-time (pulse width) in microseconds (shadow compare -> loads
     at the next period boundary). */
  void ecap_set_pulse_us(uint32_t base, uint32_t pulse_us);

  /* Rest the output low (compare shadow = 0). */
  void ecap_low(uint32_t base);

  /* Register readbacks (diagnostics / running-detection). The active period/
     compare (CAP1/CAP2) are what the hardware is currently using; the shadow
     period/compare (CAP3/CAP4) are what was last written and load into the
     active pair at the next period boundary. They differ for up to one frame
     after a write, so both are exposed to make that explicit. */
  uint32_t ecap_read_tsctr(uint32_t base);           /* free-running counter  */
  uint32_t ecap_read_period(uint32_t base);          /* active  period  (CAP1) */
  uint32_t ecap_read_compare(uint32_t base);         /* active  compare (CAP2) */
  uint32_t ecap_read_period_shadow(uint32_t base);   /* shadow  period  (CAP3) */
  uint32_t ecap_read_compare_shadow(uint32_t base);  /* shadow  compare (CAP4) */
#ifdef __cplusplus
}
#endif

#endif /* AM67_ECAP_H */
