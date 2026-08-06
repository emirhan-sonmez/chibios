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
 * @file    am67_vim.h
 * @brief   TI VIM interrupt controller driver for the AM67A/J722S R5F.
 */

#ifndef AM67_VIM_H
#define AM67_VIM_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief   IRQ handler type.
 * @return  True if a reschedule is required on ISR exit.
 */
typedef bool (*vim_handler_t)(void *arg);

#ifdef __cplusplus
extern "C" {
#endif
  void vim_init(void);
  void vim_set_handler(uint32_t irq, vim_handler_t handler, void *arg);
  void vim_set_priority(uint32_t irq, uint32_t priority);
  void vim_enable_irq(uint32_t irq);
  void vim_disable_irq(uint32_t irq);
  uint32_t vim_line_state(uint32_t irq);
#ifdef __cplusplus
}
#endif

#endif /* AM67_VIM_H */
