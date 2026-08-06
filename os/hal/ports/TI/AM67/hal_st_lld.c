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
 * @file    TI/AM67/hal_st_lld.c
 * @brief   ST Driver subsystem low level driver source.
 * @details DMTIMER0 counts up from a preloaded value and interrupts on
 *          overflow, auto-reloading for a periodic tick. Register layout
 *          from the TI J722S TRM, matches the NuttX am67 tick timer.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

#define TIMER_IRQ_EOI           0x20U
#define TIMER_IRQSTATUS         0x28U
#define TIMER_IRQSTATUS_SET     0x2CU
#define TIMER_IRQSTATUS_CLR     0x30U
#define TIMER_TCLR              0x38U
#define TIMER_TCRR              0x3CU
#define TIMER_TLDR              0x40U

#define TIMER_TCLR_ST           (1U << 0)
#define TIMER_TCLR_AR           (1U << 1)
#define TIMER_IRQ_OVF           (1U << 1)

static inline volatile uint32_t *tmr_reg(uint32_t offset) {

  return (volatile uint32_t *)(AM67_TIMER0_BASE + offset);
}

static void tick_clear_irq(void) {

  *tmr_reg(TIMER_IRQSTATUS) = TIMER_IRQ_OVF;

  /* Read back to make sure the write reached the peripheral, the interrupt
     is level-sensitive at the VIM.*/
  if ((*tmr_reg(TIMER_IRQSTATUS) & TIMER_IRQ_OVF) != 0U) {
    *tmr_reg(TIMER_IRQSTATUS) = TIMER_IRQ_OVF;
  }
}

static bool tick_irq_handler(void *arg) {
  bool preemption_required;

  (void)arg;

  tick_clear_irq();

  chSysLockFromISR();
  chSysTimerHandlerI();
  preemption_required = chSchIsPreemptionRequired();
  chSysUnlockFromISR();

  return preemption_required;
}

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {
  uint32_t reload;

  /* Counter value producing OSAL_ST_FREQUENCY overflows per second.*/
  reload = 0xFFFFFFFFU - (AM67_TIMER0_CLK_HZ / OSAL_ST_FREQUENCY) + 1U;

  /* Stop the timer and clear any pending overflow interrupt.*/
  *tmr_reg(TIMER_TCLR) = 0U;
  tick_clear_irq();

  /* Auto-reload mode, counter and reload value set for the tick period.*/
  *tmr_reg(TIMER_TCRR) = reload;
  *tmr_reg(TIMER_TLDR) = reload;
  *tmr_reg(TIMER_IRQSTATUS_SET) = TIMER_IRQ_OVF;

  /* Highest priority (0 = most urgent, VIM_LOWEST_PRIORITY = 0xF = least):
     the system tick must always be able to preempt any peripheral ISR, or
     that peripheral's interrupt load can stall the scheduler's own
     timekeeping. Previously 0x8, the same level as UART1
     (AM67_SERIAL_UART1_IRQ_PRIORITY in mcuconf.h) and every other
     peripheral in this port -- same-priority VIM interrupts cannot preempt
     each other, so a UART1 RX burst busy inside its own drain loop
     (sd_lld_serve_interrupt(), up to ~2.5ms for a 32-byte iBus frame at
     115200 baud, 130 times/sec) could delay the 1kHz tick for that entire
     window. Root-caused on hardware: intermittent hangs (during boot and
     during steady-state operation, never reproduced without a receiver
     actively transmitting) that required a full power cycle to recover
     from, with the R5F core apparently still "running" per Linux but never
     producing another trace line. See the ArduPilot Gemstone-QuadPlane
     vault, Session Notes 2026-07-29, for the full investigation. */
  vim_set_handler(AM67_TIMER0_IRQ, tick_irq_handler, NULL);
  vim_set_priority(AM67_TIMER0_IRQ, 0x0U);
  vim_enable_irq(AM67_TIMER0_IRQ);

  /* Start counting.*/
  *tmr_reg(TIMER_TCLR) = TIMER_TCLR_ST | TIMER_TCLR_AR;
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
