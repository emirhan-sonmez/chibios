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
 * @file    TI/AM67/am67_ecap.c
 * @brief   Minimal eCAP APWM output driver for the AM67/J722S.
 * @details Register offsets and the APWM programming sequence are taken from
 *          the Linux pwm-tiecap driver and confirmed against the J722S register
 *          spreadsheet (58_ECAP0). Access widths differ within the block: the
 *          counter and CAP1..CAP4 are 32-bit (readl/writel), ECCTL2 is 16-bit
 *          (readw/writew).
 *
 *          APWM mode: TSCTR free-runs 0..CAP1(period) and resets. With active-
 *          high polarity the output is HIGH while TSCTR < CAP2(compare), so the
 *          high time = CAP2 / fck and the period = CAP1 / fck. CAP3/CAP4 are the
 *          shadow period/compare and load into CAP1/CAP2 at the period boundary.
 */

#include "hal.h"
#include "am67_ecap.h"

/*===========================================================================*/
/* eCAP register offsets.                                                    */
/*===========================================================================*/

#define ECAP_TSCTR              0x00U  /* Time-stamp counter (32-bit).        */
#define ECAP_CAP1               0x08U  /* APWM active period  (32-bit).       */
#define ECAP_CAP2               0x0CU  /* APWM active compare (32-bit).       */
#define ECAP_CAP3               0x10U  /* APWM shadow period  (32-bit).       */
#define ECAP_CAP4               0x14U  /* APWM shadow compare (32-bit).       */
#define ECAP_ECCTL2             0x2AU  /* Capture control 2 (16-bit).         */

/* ECCTL2 fields. */
#define ECCTL2_TSCTR_FREERUN    (1U << 4)
#define ECCTL2_SYNC_SEL_DISA    ((1U << 6) | (1U << 7))
#define ECCTL2_APWM_MODE        (1U << 9)
#define ECCTL2_APWM_POL_LOW     (1U << 10)

/*===========================================================================*/
/* Local helpers.                                                            */
/*===========================================================================*/

static inline void ecap_wr32(uint32_t base, uint32_t off, uint32_t v) {

  *(volatile uint32_t *)(base + off) = v;
}

static inline uint32_t ecap_rd32(uint32_t base, uint32_t off) {

  return *(volatile uint32_t *)(base + off);
}

static inline void ecap_wr16(uint32_t base, uint32_t off, uint16_t v) {

  *(volatile uint16_t *)(base + off) = v;
}

static inline uint16_t ecap_rd16(uint32_t base, uint32_t off) {

  return *(volatile uint16_t *)(base + off);
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void ecap_start(uint32_t base, uint32_t frame_hz) {
  uint32_t period = (frame_hz != 0U) ? (AM67_ECAP_CLK_HZ / frame_hz) : 0U;
  uint16_t ecctl2 = ecap_rd16(base, ECAP_ECCTL2);

  /* APWM mode, sync disabled, active high, free-running counter. */
  ecctl2 |= (ECCTL2_APWM_MODE | ECCTL2_SYNC_SEL_DISA | ECCTL2_TSCTR_FREERUN);
  ecctl2 &= ~ECCTL2_APWM_POL_LOW;
  ecap_wr16(base, ECAP_ECCTL2, ecctl2);

  /* Program the ACTIVE period/compare directly, then the shadows.
     2026-07-30: writing only the shadows (the previous behaviour) deadlocks if
     Linux left CAP1 = 0. The shadow pair loads into the active pair on the
     period boundary, i.e. on TSCTR reaching CAP1 -- with CAP1 = 0 there is no
     period event to load on, TSCTR free-runs to 2^32 (~34 s at 125 MHz) and the
     active compare stays 0 forever, so the pin emits nothing and every readback
     shows shadow and active permanently disagreeing. Observed on ch3/ECAP0
     (pin 32) as shadow[cmp=125000] active[cmp=0] sustained across many ticks,
     and as a dead pin in the isolation test. Writing the active registers here
     makes the very first period valid regardless of what Linux left behind;
     the shadows then keep subsequent updates glitch-free. Start at 0% duty
     (output low). */
  ecap_wr32(base, ECAP_CAP1, period);
  ecap_wr32(base, ECAP_CAP2, 0U);
  ecap_wr32(base, ECAP_CAP3, period);
  ecap_wr32(base, ECAP_CAP4, 0U);
}

void ecap_set_pulse_us(uint32_t base, uint32_t pulse_us) {
  uint32_t period = ecap_rd32(base, ECAP_CAP1);
  uint32_t cmp = (uint32_t)(((uint64_t)pulse_us * AM67_ECAP_CLK_HZ) / 1000000ULL);

  if ((period != 0U) && (cmp > period)) {
    cmp = period;
  }
  ecap_wr32(base, ECAP_CAP4, cmp);   /* shadow compare -> loads at boundary */
}

void ecap_low(uint32_t base) {

  ecap_wr32(base, ECAP_CAP4, 0U);
}

uint32_t ecap_read_tsctr(uint32_t base)          { return ecap_rd32(base, ECAP_TSCTR); }
uint32_t ecap_read_period(uint32_t base)         { return ecap_rd32(base, ECAP_CAP1); }
uint32_t ecap_read_compare(uint32_t base)        { return ecap_rd32(base, ECAP_CAP2); }
uint32_t ecap_read_period_shadow(uint32_t base)  { return ecap_rd32(base, ECAP_CAP3); }
uint32_t ecap_read_compare_shadow(uint32_t base) { return ecap_rd32(base, ECAP_CAP4); }
