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
 * @file    am67_vim.c
 * @brief   TI VIM interrupt controller driver for the AM67A/J722S R5F.
 * @details Register layout and dispatch sequence follow the TI J722S TRM,
 *          the handling order matches the proven NuttX am67 implementation:
 *          read IRQVEC (latches priority), read ACTIRQ, dispatch, clear the
 *          status bit, then write IRQVEC to signal end of interrupt.
 */

#include "ch.h"
#include "board.h"
#include "am67_vim.h"

#define VIM_IRQVEC              0x018U
#define VIM_ACTIRQ              0x020U
#define VIM_GROUP_RAW(j)        (0x400U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_GROUP_STS(j)        (0x404U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_GROUP_INT_EN(j)     (0x408U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_GROUP_INT_DIS(j)    (0x40CU + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_GROUP_INT_MAP(j)    (0x418U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_GROUP_INT_TYPE(j)   (0x41CU + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_INT_PRI(j)          (0x1000U + ((j) * 4U))
#define VIM_INT_VEC(j)          (0x2000U + ((j) * 4U))

#define VIM_ACTIRQ_VALID        0x80000000U
#define VIM_BIT(j)              (1U << ((j) & 0x1FU))
#define VIM_LOWEST_PRIORITY     0xFU

static struct {
  vim_handler_t         handler;
  void                  *arg;
} vim_handlers[AM67_VIM_NUM_IRQS];

static inline volatile uint32_t *vim_reg(uint32_t offset) {

  return (volatile uint32_t *)(AM67_VIM_BASE + offset);
}

static inline void vim_barrier(void) {

  __asm volatile ("isb" ::: "memory");
  __asm volatile ("dsb" ::: "memory");
}

void vim_init(void) {
  uint32_t i;

  for (i = 0U; i < AM67_VIM_NUM_IRQS; i++) {
    *vim_reg(VIM_INT_PRI(i)) = VIM_LOWEST_PRIORITY;
    *vim_reg(VIM_INT_VEC(i)) = 0U;
  }

  for (i = 0U; i < AM67_VIM_NUM_IRQS; i += 32U) {
    /* Disable and clear everything, all interrupts level-sensitive and
       routed to IRQ (not FIQ).*/
    *vim_reg(VIM_GROUP_INT_DIS(i))  = 0xFFFFFFFFU;
    *vim_reg(VIM_GROUP_STS(i))      = 0xFFFFFFFFU;
    *vim_reg(VIM_GROUP_INT_TYPE(i)) = 0U;
    *vim_reg(VIM_GROUP_INT_MAP(i))  = 0U;
  }

  /* Reading IRQVEC latches the prioritization logic, the write releases
     any stale in-service state left by the previous owner of the core.*/
  (void)*vim_reg(VIM_IRQVEC);
  *vim_reg(VIM_IRQVEC) = 0U;
}

void vim_set_handler(uint32_t irq, vim_handler_t handler, void *arg) {

  chDbgAssert(irq < AM67_VIM_NUM_IRQS, "invalid IRQ number");

  vim_handlers[irq].handler = handler;
  vim_handlers[irq].arg     = arg;
}

void vim_set_priority(uint32_t irq, uint32_t priority) {

  chDbgAssert(irq < AM67_VIM_NUM_IRQS, "invalid IRQ number");

  *vim_reg(VIM_INT_PRI(irq)) = priority & VIM_LOWEST_PRIORITY;
}

void vim_enable_irq(uint32_t irq) {

  chDbgAssert(irq < AM67_VIM_NUM_IRQS, "invalid IRQ number");

  vim_barrier();
  *vim_reg(VIM_GROUP_INT_EN(irq)) = VIM_BIT(irq);
}

void vim_disable_irq(uint32_t irq) {

  chDbgAssert(irq < AM67_VIM_NUM_IRQS, "invalid IRQ number");

  *vim_reg(VIM_GROUP_INT_DIS(irq)) = VIM_BIT(irq);
  vim_barrier();
}

/**
 * @brief   Samples the controller state for one line, for diagnostics.
 * @details Distinguishes "the peripheral never raised its line" from "the
 *          VIM latched it but never dispatched it", which look identical
 *          from a driver that simply timed out.
 *
 * @param[in] irq       IRQ line number
 * @return              bit 0 raw (line asserted), bit 1 status (pending),
 *                      bit 2 enabled.
 */
uint32_t vim_line_state(uint32_t irq) {
  uint32_t state = 0U;

  chDbgAssert(irq < AM67_VIM_NUM_IRQS, "invalid IRQ number");

  if ((*vim_reg(VIM_GROUP_RAW(irq)) & VIM_BIT(irq)) != 0U) {
    state |= 1U;
  }
  if ((*vim_reg(VIM_GROUP_STS(irq)) & VIM_BIT(irq)) != 0U) {
    state |= 2U;
  }
  if ((*vim_reg(VIM_GROUP_INT_EN(irq)) & VIM_BIT(irq)) != 0U) {
    state |= 4U;
  }
  return state;
}

/*
 * IRQ dispatcher called by the ARMv7-R port IRQ entry code.
 */
bool __port_irq_dispatch(void) {
  uint32_t actirq, irq;
  bool preemption_required = false;

  /* The IRQVEC read is mandatory, it latches the active interrupt and
     raises the internal priority mask.*/
  (void)*vim_reg(VIM_IRQVEC);

  actirq = *vim_reg(VIM_ACTIRQ);
  irq = actirq & (AM67_VIM_NUM_IRQS - 1U);
  if ((actirq & VIM_ACTIRQ_VALID) != 0U) {

    if (vim_handlers[irq].handler != NULL) {
      preemption_required = vim_handlers[irq].handler(vim_handlers[irq].arg);
    }

    *vim_reg(VIM_GROUP_STS(irq)) = VIM_BIT(irq);
  }

  /* The end-of-interrupt write must happen even for a spurious activation,
     it pops the priority mask raised by the IRQVEC read above.*/
  *vim_reg(VIM_IRQVEC) = irq;

  return preemption_required;
}
