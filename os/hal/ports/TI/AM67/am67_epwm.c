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
 * @file    TI/AM67/am67_epwm.c
 * @brief   Minimal eHRPWM output driver for the AM67/J722S.
 * @details Classic eHRPWM (ti,am3352-ehrpwm) 16-bit register map, confirmed
 *          against the J722S register spreadsheet (62_EPWM0) and the Linux
 *          pwm-tiehrpwm driver (readw/writew). Up-count PWM: an output is set
 *          HIGH at counter zero and cleared LOW at its compare, so the high
 *          time = CMPx / TBCLK and the frame period = TBPRD / TBCLK, with
 *          TBCLK = fck / (CLKDIV * HSPCLKDIV). Outputs A and B of one instance
 *          share the time base but use CMPA/CMPB independently.
 */

#include "hal.h"
#include "am67_epwm.h"

/*===========================================================================*/
/* eHRPWM register offsets (16-bit registers).                               */
/*===========================================================================*/

#define EPWM_TBCTL              0x00U  /* Time-base control.                  */
#define EPWM_TBCTR              0x08U  /* Time-base counter.                  */
#define EPWM_TBPRD              0x0AU  /* Time-base period.                   */
#define EPWM_CMPCTL             0x0EU  /* Compare control (reset = shadow).   */
#define EPWM_CMPA               0x12U  /* Counter-compare A (15:0).           */
#define EPWM_CMPB               0x14U  /* Counter-compare B (15:0).           */
#define EPWM_AQCTLA             0x16U  /* Action qualifier, output A.         */
#define EPWM_AQCTLB             0x18U  /* Action qualifier, output B.         */
#define EPWM_AQCSFRC            0x1CU  /* Continuous software force.          */

/* CMPCTL fields: LOADAMODE[1:0], LOADBMODE[3:2], SHDWAMODE[4], SHDWBMODE[6].
   All-zero = shadow mode for both compares, loading into the active register at
   CTR=ZERO. That is also the reset value, but this driver programs it
   explicitly (see ehrpwm_start) rather than inheriting whatever the Linux
   pwm-tiehrpwm driver last left in it. */
#define CMPCTL_SHADOW_LOAD_ZERO 0x0000U

/* TBCTL fields. */
#define TBCTL_CTRMODE_UP        (0U << 0)   /* Count up.                      */
#define TBCTL_CTRMODE_STOP      (3U << 0)   /* Stop-freeze.                   */
#define TBCTL_HSPCLKDIV_DIV10   (5U << 7)   /* High-speed prescale /10 (0b101).*/
#define TBCTL_CLKDIV_DIV8       (3U << 10)  /* Prescale /8 (2^3, 0b011).      */

/* Fixed prescale = HSPCLKDIV(/10) * CLKDIV(/8) = 80. Sized for the confirmed
   250 MHz fck so a 50 Hz frame fits the 16-bit TBPRD: 250e6/80/50 = 62500. */
#define EPWM_PRESCALE           80U
#define TBCTL_PRESCALE          (TBCTL_HSPCLKDIV_DIV10 | TBCTL_CLKDIV_DIV8)

/* Action qualifier value for up-count PWM on output A: set HIGH at ZERO
   (ZRO[1:0]=2), clear LOW on the up-count CMPA match (CAU[5:4]=1) -> 0x0012. */
#define AQCTLA_UP_PWM           ((2U << 0) | (1U << 4))

/* Same for output B: set HIGH at ZERO (ZRO[1:0]=2), clear LOW on the up-count
   CMPB match (CBU[9:8]=1) -> 0x0102. */
#define AQCTLB_UP_PWM           ((2U << 0) | (1U << 8))

/* AQCSFRC continuous software force fields: CSFA[1:0], CSFB[3:2]. */
#define AQCSFRC_CSFA_MASK       0x0003U
#define AQCSFRC_CSFA_LOW        0x0001U
#define AQCSFRC_CSFB_MASK       0x000CU
#define AQCSFRC_CSFB_LOW        0x0004U

/* Effective time-base clock after the fixed prescale (same for all EPWM
   instances -- they share the 250 MHz PWMSS fck). */
#define EPWM_TBCLK_HZ           (AM67_EPWM0_CLK_HZ / EPWM_PRESCALE)

/*===========================================================================*/
/* Local helpers (base-address parameterised).                               */
/*===========================================================================*/

static inline void epwm_wr16(uint32_t base, uint32_t off, uint16_t v) {

  *(volatile uint16_t *)(base + off) = v;
}

static inline uint16_t epwm_rd16(uint32_t base, uint32_t off) {

  return *(volatile uint16_t *)(base + off);
}

static uint16_t epwm_tbprd_for(uint32_t frame_hz) {
  uint32_t prd = (frame_hz != 0U) ? (EPWM_TBCLK_HZ / frame_hz) : 0xFFFFU;

  if (prd > 0xFFFFU) {
    prd = 0xFFFFU;
  }
  return (uint16_t)prd;
}

/*===========================================================================*/
/* Generic eHRPWM API.                                                       */
/*===========================================================================*/

void ehrpwm_start(uint32_t base, uint32_t frame_hz) {

  /* Do not inherit CMPCTL. Linux owns the clock gate for this peripheral and
     its own pwm-tiehrpwm driver may have touched this register before we ever
     ran, so the reset value cannot be assumed. Both compares must be in shadow
     mode with load at CTR=ZERO: in immediate mode a compare write takes effect
     part-way through a period and truncates the pulse being emitted, which on
     these pins means handing an ESC a short pulse it never asked for. */
  epwm_wr16(base, EPWM_CMPCTL, CMPCTL_SHADOW_LOAD_ZERO);

  epwm_wr16(base, EPWM_TBPRD, epwm_tbprd_for(frame_hz));
  epwm_wr16(base, EPWM_TBCTR, 0U);
  epwm_wr16(base, EPWM_TBCTL, TBCTL_CTRMODE_UP | TBCTL_PRESCALE);
}

void ehrpwm_out_enable(uint32_t base, bool output_b) {
  uint16_t force = epwm_rd16(base, EPWM_AQCSFRC);

  if (!output_b) {
    epwm_wr16(base, EPWM_CMPA, 0U);
    epwm_wr16(base, EPWM_AQCTLA, AQCTLA_UP_PWM);
    epwm_wr16(base, EPWM_AQCSFRC, force & ~AQCSFRC_CSFA_MASK);  /* release A */
  }
  else {
    epwm_wr16(base, EPWM_CMPB, 0U);
    epwm_wr16(base, EPWM_AQCTLB, AQCTLB_UP_PWM);
    epwm_wr16(base, EPWM_AQCSFRC, force & ~AQCSFRC_CSFB_MASK);  /* release B */
  }
}

void ehrpwm_out_set_pulse_us(uint32_t base, bool output_b, uint32_t pulse_us) {
  uint32_t cmp = (uint32_t)(((uint64_t)pulse_us * EPWM_TBCLK_HZ) / 1000000ULL);
  uint16_t prd = epwm_rd16(base, EPWM_TBPRD);

  if (cmp > prd) {
    cmp = prd;
  }
  epwm_wr16(base, output_b ? EPWM_CMPB : EPWM_CMPA, (uint16_t)cmp);
}

void ehrpwm_out_reassert(uint32_t base, bool output_b, uint32_t frame_hz) {
  uint16_t force = epwm_rd16(base, EPWM_AQCSFRC);

  /* Time-base and compare-load configuration, reasserted alongside the action
     qualifier. These were previously written once in ehrpwm_start() and never
     revisited, which left the last writer at boot owning them permanently --
     and Linux is a writer: gemstone-r5f-setup.service brings each peripheral's
     clock up by exporting its sysfs pwm0 and writing period/duty/enable, and
     pwm0 is channel A. Its pwm-tiehrpwm driver programs TBCTL, TBPRD and
     CMPCTL with its own values, racing this firmware during the same startup.

     TBCTR is deliberately NOT written here. Zeroing the counter mid-period
     stretches or truncates the period being emitted while the pulse width
     stays put -- the exact glitch ehrpwm_start()'s idempotence guard exists to
     avoid. Everything below is idempotent and glitch-free when the values
     already match, which is the normal case.

     A TBPRD restored from under a stale CMPA leaves that one channel's pulse
     scaled to the wrong period for at most one frame: ehrpwm_out_set_pulse_us()
     recomputes the compare from the live TBPRD on the next write, and the
     caller writes every tick. */
  epwm_wr16(base, EPWM_CMPCTL, CMPCTL_SHADOW_LOAD_ZERO);
  epwm_wr16(base, EPWM_TBPRD, epwm_tbprd_for(frame_hz));
  epwm_wr16(base, EPWM_TBCTL, TBCTL_CTRMODE_UP | TBCTL_PRESCALE);

  if (!output_b) {
    epwm_wr16(base, EPWM_AQCTLA, AQCTLA_UP_PWM);
    epwm_wr16(base, EPWM_AQCSFRC, force & ~AQCSFRC_CSFA_MASK);
  }
  else {
    epwm_wr16(base, EPWM_AQCTLB, AQCTLB_UP_PWM);
    epwm_wr16(base, EPWM_AQCSFRC, force & ~AQCSFRC_CSFB_MASK);
  }
}

void ehrpwm_out_low(uint32_t base, bool output_b) {
  uint16_t force = epwm_rd16(base, EPWM_AQCSFRC);

  if (!output_b) {
    force = (force & ~AQCSFRC_CSFA_MASK) | AQCSFRC_CSFA_LOW;
  }
  else {
    force = (force & ~AQCSFRC_CSFB_MASK) | AQCSFRC_CSFB_LOW;
  }
  epwm_wr16(base, EPWM_AQCSFRC, force);
}

uint16_t ehrpwm_read_tbctr(uint32_t base) { return epwm_rd16(base, EPWM_TBCTR); }
uint16_t ehrpwm_read_tbprd(uint32_t base) { return epwm_rd16(base, EPWM_TBPRD); }
uint16_t ehrpwm_read_cmp(uint32_t base, bool output_b) {

  return epwm_rd16(base, output_b ? EPWM_CMPB : EPWM_CMPA);
}
uint16_t ehrpwm_read_aqctl(uint32_t base, bool output_b) {

  return epwm_rd16(base, output_b ? EPWM_AQCTLB : EPWM_AQCTLA);
}

/*===========================================================================*/
/* EPWM0 output-A convenience wrappers (bring-up demo).                      */
/*===========================================================================*/

void epwm0a_init(void) {

  /* Safe default: force the pin LOW and freeze the counter. */
  ehrpwm_out_low(AM67_EPWM0_BASE, false);
  epwm_wr16(AM67_EPWM0_BASE, EPWM_TBCTL, TBCTL_CTRMODE_STOP | TBCTL_PRESCALE);
  epwm_wr16(AM67_EPWM0_BASE, EPWM_TBPRD, 0U);
  epwm_wr16(AM67_EPWM0_BASE, EPWM_CMPA, 0U);
  epwm_wr16(AM67_EPWM0_BASE, EPWM_AQCTLA, AQCTLA_UP_PWM);
}

void epwm0a_start(uint32_t frame_hz) {

  ehrpwm_start(AM67_EPWM0_BASE, frame_hz);
  ehrpwm_out_enable(AM67_EPWM0_BASE, false);
}

void epwm0a_set_pulse_us(uint32_t pulse_us) {

  ehrpwm_out_set_pulse_us(AM67_EPWM0_BASE, false, pulse_us);
}

void epwm0a_stop(void) {

  ehrpwm_out_low(AM67_EPWM0_BASE, false);
  epwm_wr16(AM67_EPWM0_BASE, EPWM_TBCTL, TBCTL_CTRMODE_STOP | TBCTL_PRESCALE);
}

uint16_t epwm0a_read_tbprd(void)   { return epwm_rd16(AM67_EPWM0_BASE, EPWM_TBPRD); }
uint16_t epwm0a_read_tbctr(void)   { return epwm_rd16(AM67_EPWM0_BASE, EPWM_TBCTR); }
uint16_t epwm0a_read_tbctl(void)   { return epwm_rd16(AM67_EPWM0_BASE, EPWM_TBCTL); }
uint16_t epwm0a_read_aqctla(void)  { return epwm_rd16(AM67_EPWM0_BASE, EPWM_AQCTLA); }
uint16_t epwm0a_read_aqcsfrc(void) { return epwm_rd16(AM67_EPWM0_BASE, EPWM_AQCSFRC); }
uint16_t epwm0a_read_cmpa(void)    { return epwm_rd16(AM67_EPWM0_BASE, EPWM_CMPA); }
