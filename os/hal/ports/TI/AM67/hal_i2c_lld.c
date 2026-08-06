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
 * @file    TI/AM67/hal_i2c_lld.c
 * @brief   AM67 (J722S) I2C subsystem low level driver source.
 * @details Interrupt-driven master-mode driver for the TI OMAP-style I2C
 *          controller. Init sequence, transfer state machine and the
 *          STOP/bus-free dance derived from NuttX
 *          arch/arm/src/am67/am67_i2c.c (Apache-2.0): STP is never
 *          pre-armed with a segment (writing it in the restart window
 *          fires an immediate STOP), every segment ends with the bus held
 *          and the final STOP is issued explicitly from the ARDY window.
 *
 * @addtogroup I2C
 * @{
 */

#include "hal.h"

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#include "am67_padcfg.h"

/* MCU-domain pad offsets for the MCU_I2C0 signals (header pins 3/5).*/
#define AM67_PAD_I2C0_SCL           0x0044U
#define AM67_PAD_I2C0_SDA           0x0048U

/* Internal reference clock the prescaler divides down to, per the TRM
   recommendation for standard and fast mode.*/
#define I2C_INTERNAL_CLOCK          12000000U

/* Bound for the reset-done busy wait, avoids a silent hard hang if the
   module clock is gated (i2c_lld_start() runs in a lock zone).*/
#define I2C_RESET_WAIT_LOOPS        1000000U

/* Bound for the wait that observes the reset actually starting. Short: it
   is a race guard, not a completion wait, and falling through it is a
   legitimate outcome (see i2c_hw_init).*/
#define I2C_RESET_START_LOOPS       10000U

/* Bound for the wait that lets a STOP release a bus left held by a
   previous transaction.*/
#define I2C_BUS_FREE_LOOPS          100000U

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief MCU_I2C0 I2C driver identifier.*/
#if (AM67_I2C_USE_MCU_I2C0 == TRUE) || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline uint32_t i2c_getreg(I2CDriver *i2cp, uint32_t offset) {

  return *(volatile uint32_t *)(i2cp->base + offset);
}

static inline void i2c_putreg(I2CDriver *i2cp, uint32_t offset,
                              uint32_t value) {

  *(volatile uint32_t *)(i2cp->base + offset) = value;
}

/*
 * Routes the MCU_I2C0 SCL/SDA pads to the I2C controller (mux mode 0)
 * with receivers and internal pull-ups enabled (matches the Linux device
 * tree pin configuration, PIN_INPUT_PULLUP).
 */
static void i2c0_pinmux(void) {

  am67_mcu_padcfg_unlock();

  am67_mcu_pad_config(AM67_PAD_I2C0_SCL,
                      AM67_PIN_MODE(0) | AM67_PIN_INPUT_ENABLE |
                      AM67_PIN_PULLUP);
  am67_mcu_pad_config(AM67_PAD_I2C0_SDA,
                      AM67_PIN_MODE(0) | AM67_PIN_INPUT_ENABLE |
                      AM67_PIN_PULLUP);
}

/**
 * @brief   Controller reset and timing configuration.
 * @details Sequence from NuttX am67_i2c_init(): the TI I2C requires
 *          CON.EN=1 for the internal reset to complete, and no-idle mode
 *          with both clocks kept active so the module is not gated
 *          between transfers.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 */
static void i2c_hw_init(I2CDriver *i2cp) {
  uint32_t psc, d, scll, sclh, i;

  i2cp->ready      = false;
  i2cp->init_error = I2C_INIT_OK;

  /* Soft reset, completes only with the module enabled.

     RST_DONE still reads 1 for a short while after SRST is asserted, so
     polling it immediately can fall straight through while the reset has
     not even started. Everything written afterwards would then be wiped
     by the reset completing underneath us, leaving an enabled module with
     zeroed SCL timing: no clock is ever generated, START never completes
     and every transfer times out. Wait for RST_DONE to drop first, then
     for it to come back. Falling through the first wait is legitimate (a
     reset fast enough to miss), the readback below is the real guard.*/
  i2c_putreg(i2cp, I2C_CON_OFFSET, 0U);
  i2c_putreg(i2cp, I2C_SYSC_OFFSET, I2C_SYSC_SRST);
  i2c_putreg(i2cp, I2C_CON_OFFSET, I2C_CON_EN);
  for (i = 0U; i < I2C_RESET_START_LOOPS; i++) {
    if ((i2c_getreg(i2cp, I2C_SYSS_OFFSET) & I2C_SYSS_RST_DONE) == 0U) {
      break;
    }
  }
  for (i = 0U; i < I2C_RESET_WAIT_LOOPS; i++) {
    if ((i2c_getreg(i2cp, I2C_SYSS_OFFSET) & I2C_SYSS_RST_DONE) != 0U) {
      break;
    }
  }
  if (i >= I2C_RESET_WAIT_LOOPS) {
    i2cp->init_error = I2C_INIT_RESET_TIMEOUT;
    return;
  }

  i2c_putreg(i2cp, I2C_SYSC_OFFSET, I2C_SYSC_IDLE_NO | I2C_SYSC_CLK_BOTH);

  /* SCL timing, written with the module disabled. The prescaler divides
     the functional clock to the 12 MHz internal reference, then:
     SCL = 12 MHz / ((SCLL + 7) + (SCLH + 5)).*/
  i2c_putreg(i2cp, I2C_CON_OFFSET, 0U);

  psc = (i2cp->clock / I2C_INTERNAL_CLOCK) - 1U;
  d   = I2C_INTERNAL_CLOCK / i2cp->config->frequency;
  scll = (d / 2U) - 7U;
  sclh = (d / 2U) - 5U;
  if ((scll < 1U) || (scll > 255U) || (sclh > 255U)) {
    i2cp->init_error = I2C_INIT_BAD_FREQUENCY;
    return;
  }
  i2c_putreg(i2cp, I2C_PSC_OFFSET, psc);
  i2c_putreg(i2cp, I2C_SCLL_OFFSET, scll);
  i2c_putreg(i2cp, I2C_SCLH_OFFSET, sclh);

  /* Readback guard for the reset race described above: if the reset was
     still in flight these registers now read back as zero rather than
     what was just written.*/
  if ((i2c_getreg(i2cp, I2C_SCLL_OFFSET) != scll) ||
      (i2c_getreg(i2cp, I2C_SCLH_OFFSET) != sclh) ||
      (i2c_getreg(i2cp, I2C_PSC_OFFSET) != psc)) {
    i2cp->init_error = I2C_INIT_TIMING_LOST;
    return;
  }

  i2c_putreg(i2cp, I2C_CON_OFFSET, I2C_CON_EN);

  /* All wakeup sources on (the TI Linux driver does the same to stop the
     I2C freezing across WFI on AM6x-class devices).*/
  i2c_putreg(i2cp, I2C_WE_OFFSET, I2C_WE_ALLMASK);

  /* Clean slate: interrupts off, pending flags and FIFOs cleared.*/
  i2c_putreg(i2cp, I2C_IRQENABLE_CLR_OFFSET, I2C_IRQ_ALLMASK);
  i2c_putreg(i2cp, I2C_IRQSTATUS_OFFSET, I2C_IRQ_ALLMASK);
  i2c_putreg(i2cp, I2C_BUF_OFFSET, I2C_BUF_TXFIFO_CLR | I2C_BUF_RXFIFO_CLR);

  /* A soft reset resets this master but not the bus. If the previous
     transaction ended without a STOP (the NACK path releases the engine
     but leaves the bus held) the slave is still mid-transfer and BB stays
     set, so the next START would never be granted. Issue one STOP to
     release it before declaring the driver ready.*/
  if ((i2c_getreg(i2cp, I2C_IRQSTATUS_RAW_OFFSET) & I2C_IRQ_BB) != 0U) {
    i2c_putreg(i2cp, I2C_CON_OFFSET,
               I2C_CON_EN | I2C_CON_MST | I2C_CON_STP);
    for (i = 0U; i < I2C_BUS_FREE_LOOPS; i++) {
      if ((i2c_getreg(i2cp, I2C_IRQSTATUS_RAW_OFFSET) & I2C_IRQ_BB) == 0U) {
        break;
      }
    }
    i2c_putreg(i2cp, I2C_IRQSTATUS_OFFSET, I2C_IRQ_ALLMASK);
    i2c_putreg(i2cp, I2C_CON_OFFSET, I2C_CON_EN);

    if ((i2c_getreg(i2cp, I2C_IRQSTATUS_RAW_OFFSET) & I2C_IRQ_BB) != 0U) {
      /* Still held: a slave is clock stretching or holding SDA and only
         bit-banged clock pulses would recover it. Report rather than let
         every later transfer time out with no explanation.*/
      i2cp->init_error = I2C_INIT_BUS_STUCK;
      return;
    }
  }

  i2cp->ready = true;
}

/**
 * @brief   Programs and starts one transfer segment.
 * @details Issues a START (or repeated START when the bus is already
 *          held). STP is deliberately not pre-armed, see the file header.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] tx        true for the transmit segment, false for receive
 */
static void i2c_start_segment(I2CDriver *i2cp, bool tx) {
  uint32_t con = I2C_CON_EN | I2C_CON_MST;
  size_t n = tx ? i2cp->txbytes : i2cp->rxbytes;

  i2c_putreg(i2cp, I2C_SA_OFFSET, (uint32_t)i2cp->addr);
  i2c_putreg(i2cp, I2C_CNT_OFFSET, (uint32_t)n);

  if (tx) {
    con |= I2C_CON_TRX;
  }
  if (i2cp->addr > 0x7FU) {
    con |= I2C_CON_XSA;
  }
  i2c_putreg(i2cp, I2C_CON_OFFSET, con);

  /* Errors, segment-done and the data direction in use; BF stays off
     until the final STOP is issued.*/
  i2c_putreg(i2cp, I2C_IRQENABLE_CLR_OFFSET,
             I2C_IRQ_XRDY | I2C_IRQ_RRDY | I2C_IRQ_BF);
  i2c_putreg(i2cp, I2C_IRQENABLE_SET_OFFSET,
             I2C_IRQ_FATALMASK | I2C_IRQ_ARDY |
             (tx ? I2C_IRQ_XRDY : I2C_IRQ_RRDY));

  i2c_putreg(i2cp, I2C_CON_OFFSET, con | I2C_CON_STT);
}

/**
 * @brief   Quiesces the controller after an error or a timeout.
 * @details Mirrors the NuttX fatal-error path: release the bus and reset
 *          the transaction engine, otherwise the master is left
 *          mid-transaction and every later transfer times out.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 */
static void i2c_abort(I2CDriver *i2cp) {

  i2c_putreg(i2cp, I2C_CON_OFFSET,
             i2c_getreg(i2cp, I2C_CON_OFFSET) | I2C_CON_STP);
  i2c_putreg(i2cp, I2C_IRQENABLE_CLR_OFFSET, I2C_IRQ_ALLMASK);
  i2c_putreg(i2cp, I2C_IRQSTATUS_OFFSET, I2C_IRQ_ALLMASK);
  i2c_putreg(i2cp, I2C_BUF_OFFSET, I2C_BUF_TXFIFO_CLR | I2C_BUF_RXFIFO_CLR);
  i2c_putreg(i2cp, I2C_CNT_OFFSET, 0U);
  i2cp->txbytes = 0U;
  i2cp->rxbytes = 0U;
}

/**
 * @brief   Shared interrupt service.
 * @details Ordered like the NuttX ISR: fatal errors first (checked in
 *          the raw status so they cannot hide behind masking), then data
 *          movement, then the segment/stop bookkeeping.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 */
static void i2c_serve_interrupt(I2CDriver *i2cp) {
  uint32_t status = i2c_getreg(i2cp, I2C_IRQSTATUS_OFFSET);
  uint32_t raw    = i2c_getreg(i2cp, I2C_IRQSTATUS_RAW_OFFSET);
  uint32_t fatal  = raw & I2C_IRQ_FATALMASK;

  if (fatal != 0U) {
    if ((fatal & I2C_IRQ_NACK) != 0U) {
      i2cp->errors |= I2C_ACK_FAILURE;
    }
    if ((fatal & I2C_IRQ_AL) != 0U) {
      i2cp->errors |= I2C_ARBITRATION_LOST;
    }
    if ((fatal & I2C_IRQ_AERR) != 0U) {
      i2cp->errors |= I2C_BUS_ERROR;
    }
    i2c_abort(i2cp);

    osalSysLockFromISR();
    osalThreadResumeI(&i2cp->thread, MSG_RESET);
    osalSysUnlockFromISR();
    return;
  }

  /* A byte arrived: drain it before any phase decision so a pending
     restart/stop cannot strand it in the FIFO.*/
  if ((status & I2C_IRQ_RRDY) != 0U) {
    uint32_t data = i2c_getreg(i2cp, I2C_DATA_OFFSET);

    if (i2cp->rxbytes > 0U) {
      *i2cp->rxptr++ = (uint8_t)data;
      i2cp->rxbytes--;
    }
    i2c_putreg(i2cp, I2C_IRQSTATUS_OFFSET, I2C_IRQ_RRDY);
  }

  /* The transmitter wants a byte.*/
  if ((status & I2C_IRQ_XRDY) != 0U) {
    if (i2cp->txbytes > 0U) {
      i2c_putreg(i2cp, I2C_DATA_OFFSET, (uint32_t)*i2cp->txptr++);
      i2cp->txbytes--;
    }
    else {
      i2c_putreg(i2cp, I2C_IRQENABLE_CLR_OFFSET, I2C_IRQ_XRDY);
    }
    i2c_putreg(i2cp, I2C_IRQSTATUS_OFFSET, I2C_IRQ_XRDY);
  }

  /* Segment finished (ARDY) or bus released (BF).*/
  if ((status & (I2C_IRQ_ARDY | I2C_IRQ_BF)) != 0U) {
    i2c_putreg(i2cp, I2C_IRQSTATUS_OFFSET,
               status & (I2C_IRQ_ARDY | I2C_IRQ_BF));

    if ((i2cp->txbytes == 0U) && !i2cp->rx_started &&
        (i2cp->rxbytes > 0U)) {
      /* Transmit segment done, the receive segment follows with a
         repeated START, ARDY means the hardware is holding the bus.*/
      i2cp->rx_started = true;
      i2c_start_segment(i2cp, false);
    }
    else if ((i2cp->txbytes == 0U) && (i2cp->rxbytes == 0U)) {
      if (((raw & I2C_IRQ_BB) != 0U) && ((status & I2C_IRQ_BF) == 0U)) {
        /* All data done but the bus is still held: issue the final STOP
           (legal here, this is the ARDY window) and wait for the
           bus-free event. Skip STP if a STOP is already in flight.*/
        if ((i2c_getreg(i2cp, I2C_CON_OFFSET) & I2C_CON_STP) == 0U) {
          i2c_putreg(i2cp, I2C_CON_OFFSET,
                     i2c_getreg(i2cp, I2C_CON_OFFSET) | I2C_CON_STP);
        }
        i2c_putreg(i2cp, I2C_IRQENABLE_SET_OFFSET, I2C_IRQ_BF);
      }
      else {
        /* Transfer complete and bus free.*/
        i2c_putreg(i2cp, I2C_IRQENABLE_CLR_OFFSET, I2C_IRQ_ALLMASK);

        osalSysLockFromISR();
        osalThreadResumeI(&i2cp->thread, MSG_OK);
        osalSysUnlockFromISR();
      }
    }
  }
}

/**
 * @brief   Starts a transfer and suspends the calling thread.
 * @details Common tail of the transmit/receive API functions, called from
 *          within a lock zone.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] timeout   transfer timeout
 * @return              The transfer status (MSG_OK, MSG_RESET, MSG_TIMEOUT).
 */
static msg_t i2c_run_transfer(I2CDriver *i2cp, sysinterval_t timeout) {
  msg_t msg;

  if (!i2cp->ready) {
    i2cp->errors |= I2C_BUS_ERROR;
    return MSG_RESET;
  }

  i2c_start_segment(i2cp, i2cp->txbytes > 0U);

  msg = osalThreadSuspendTimeoutS(&i2cp->thread, timeout);
  if (msg == MSG_TIMEOUT) {
    /* Sample before the abort tears the evidence down. irqen and vim
       together locate the break: an asserted, enabled peripheral line
       that the VIM never dispatched is a VIM problem, an unset irqen is
       a driver problem.*/
    i2cp->dbg.con   = i2c_getreg(i2cp, I2C_CON_OFFSET);
    i2cp->dbg.raw   = i2c_getreg(i2cp, I2C_IRQSTATUS_RAW_OFFSET);
    i2cp->dbg.scll  = i2c_getreg(i2cp, I2C_SCLL_OFFSET);
    i2cp->dbg.irqen = i2c_getreg(i2cp, I2C_IRQENABLE_SET_OFFSET);
    i2cp->dbg.vim   = vim_line_state(AM67_MCU_I2C0_IRQ);
    i2c_abort(i2cp);
  }
  return msg;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (AM67_I2C_USE_MCU_I2C0 == TRUE) || defined(__DOXYGEN__)
static bool mcu_i2c0_irq_handler(void *arg) {
  bool preemption_required;

  (void)arg;

  i2c_serve_interrupt(&I2CD1);

  chSysLockFromISR();
  preemption_required = chSchIsPreemptionRequired();
  chSysUnlockFromISR();

  return preemption_required;
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

#if AM67_I2C_USE_MCU_I2C0 == TRUE
  i2cObjectInit(&I2CD1);
  I2CD1.base  = AM67_MCU_I2C0_BASE;
  I2CD1.clock = AM67_MCU_I2C0_CLOCK;
  vim_set_handler(AM67_MCU_I2C0_IRQ, mcu_i2c0_irq_handler, NULL);
  vim_set_priority(AM67_MCU_I2C0_IRQ, AM67_I2C_MCU_I2C0_IRQ_PRIORITY);
#endif
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {

#if AM67_I2C_USE_MCU_I2C0 == TRUE
  if (i2cp == &I2CD1) {
    i2c0_pinmux();
    i2c_hw_init(i2cp);
    vim_enable_irq(AM67_MCU_I2C0_IRQ);
  }
#endif
}

/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  if (i2cp->state != I2C_STOP) {
#if AM67_I2C_USE_MCU_I2C0 == TRUE
    if (i2cp == &I2CD1) {
      vim_disable_irq(AM67_MCU_I2C0_IRQ);
    }
#endif
    i2c_putreg(i2cp, I2C_IRQENABLE_CLR_OFFSET, I2C_IRQ_ALLMASK);
    i2c_putreg(i2cp, I2C_CON_OFFSET, 0U);
  }
}

/**
 * @brief   Transmits data via the I2C bus as master.
 * @details When @p rxbytes is greater than zero the receive phase follows
 *          the transmit phase with a repeated START.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[in] txbuf     pointer to the transmit buffer
 * @param[in] txbytes   number of bytes to be transmitted
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts
 * @return              The operation status.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      sysinterval_t timeout) {

  i2cp->addr       = addr;
  i2cp->txptr      = txbuf;
  i2cp->txbytes    = txbytes;
  i2cp->rxptr      = rxbuf;
  i2cp->rxbytes    = rxbytes;
  i2cp->rx_started = false;

  return i2c_run_transfer(i2cp, timeout);
}

/**
 * @brief   Receives data via the I2C bus as master.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] addr      slave device address
 * @param[out] rxbuf    pointer to the receive buffer
 * @param[in] rxbytes   number of bytes to be received
 * @param[in] timeout   the number of ticks before the operation timeouts
 * @return              The operation status.
 *
 * @notapi
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     sysinterval_t timeout) {

  i2cp->addr       = addr;
  i2cp->txptr      = NULL;
  i2cp->txbytes    = 0U;
  i2cp->rxptr      = rxbuf;
  i2cp->rxbytes    = rxbytes;
  i2cp->rx_started = true;

  return i2c_run_transfer(i2cp, timeout);
}

#endif /* HAL_USE_I2C == TRUE */

/** @} */
