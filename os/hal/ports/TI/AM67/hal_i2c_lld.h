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
 * @file    TI/AM67/hal_i2c_lld.h
 * @brief   AM67 (J722S) I2C subsystem low level driver header.
 * @details TI OMAP-style I2C controller. Register map from the TI J722S
 *          TRM, definitions derived from NuttX
 *          arch/arm/src/am67/am67_i2c_hw.h (Apache-2.0). All accesses are
 *          full 32-bit (K3 interconnect requirement).
 *
 * @addtogroup I2C
 * @{
 */

#ifndef HAL_I2C_LLD_H
#define HAL_I2C_LLD_H

#if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    MCU_I2C0 instance parameters
 * @details This is the instance wired to the 40-pin header I2C positions
 *          (pins 3/5, Linux exposes it as 4900000.i2c).
 * @{
 */
#define AM67_MCU_I2C0_BASE     0x04900000U
#define AM67_MCU_I2C0_CLOCK    96000000U   /* Functional clock, Hz.          */
#define AM67_MCU_I2C0_IRQ      197U        /* R5FSS0 core 0 VIM line.        */
/** @} */

/* Register offsets **********************************************************/

#define I2C_SYSC_OFFSET             0x010U
#define I2C_IRQSTATUS_RAW_OFFSET    0x024U
#define I2C_IRQSTATUS_OFFSET        0x028U
#define I2C_IRQENABLE_SET_OFFSET    0x02CU
#define I2C_IRQENABLE_CLR_OFFSET    0x030U
#define I2C_WE_OFFSET               0x034U
#define I2C_SYSS_OFFSET             0x090U
#define I2C_BUF_OFFSET              0x094U
#define I2C_CNT_OFFSET              0x098U
#define I2C_DATA_OFFSET             0x09CU
#define I2C_CON_OFFSET              0x0A4U
#define I2C_OA_OFFSET               0x0A8U
#define I2C_SA_OFFSET               0x0ACU
#define I2C_PSC_OFFSET              0x0B0U
#define I2C_SCLL_OFFSET             0x0B4U
#define I2C_SCLH_OFFSET             0x0B8U
#define I2C_SYSTEST_OFFSET          0x0BCU
#define I2C_BUFSTAT_OFFSET          0x0C0U

/* SYSC/SYSS bits ************************************************************/

#define I2C_SYSC_SRST               (1U << 1)
#define I2C_SYSC_IDLE_NO            (1U << 3)
#define I2C_SYSC_CLK_BOTH           (3U << 8)
#define I2C_SYSS_RST_DONE           (1U << 0)

/* IRQSTATUS/IRQENABLE bits **************************************************/

#define I2C_IRQ_AL                  (1U << 0)   /* Arbitration lost          */
#define I2C_IRQ_NACK                (1U << 1)   /* No acknowledgment         */
#define I2C_IRQ_ARDY                (1U << 2)   /* Register access ready     */
#define I2C_IRQ_RRDY                (1U << 3)   /* Receive data ready        */
#define I2C_IRQ_XRDY                (1U << 4)   /* Transmit data ready       */
#define I2C_IRQ_AERR                (1U << 7)   /* Access error              */
#define I2C_IRQ_BF                  (1U << 8)   /* Bus free                  */
#define I2C_IRQ_XUDF                (1U << 10)  /* Transmit underflow        */
#define I2C_IRQ_ROVR                (1U << 11)  /* Receive overrun           */
#define I2C_IRQ_BB                  (1U << 12)  /* Bus busy (status only)    */

#define I2C_IRQ_FATALMASK           (I2C_IRQ_AL | I2C_IRQ_NACK | I2C_IRQ_AERR)
#define I2C_IRQ_ALLMASK             0x7FFFU

/* Controller initialization outcomes ****************************************/

/**
 * @brief   Why the last i2c_lld_start() failed to bring the module up.
 * @details A failed initialization used to be silent, which surfaced as
 *          every later transfer timing out with no explanation.
 */
#define I2C_INIT_OK                 0U  /* Module ready                      */
#define I2C_INIT_RESET_TIMEOUT      1U  /* RST_DONE never rose, clock gated? */
#define I2C_INIT_TIMING_LOST        2U  /* SCL timing wiped by a late reset  */
#define I2C_INIT_BAD_FREQUENCY      3U  /* Requested SCL rate unreachable    */
#define I2C_INIT_BUS_STUCK          4U  /* BB still set after a STOP         */

/* WE (wakeup enable) bits ***************************************************/

#define I2C_WE_ALLMASK              0x6F6FU

/* BUF bits ******************************************************************/

#define I2C_BUF_TXFIFO_CLR          (1U << 6)
#define I2C_BUF_RXFIFO_CLR          (1U << 14)

/* CON bits ******************************************************************/

#define I2C_CON_STT                 (1U << 0)   /* Start condition           */
#define I2C_CON_STP                 (1U << 1)   /* Stop condition            */
#define I2C_CON_XSA                 (1U << 8)   /* 10-bit slave address      */
#define I2C_CON_TRX                 (1U << 9)   /* 1 = master transmitter    */
#define I2C_CON_MST                 (1U << 10)  /* Master mode               */
#define I2C_CON_EN                  (1U << 15)  /* Module enable             */

/* SYSTEST bits **************************************************************/

#define I2C_SYSTEST_FREE            (1U << 14)  /* Keep running on JTAG halt */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   I2CD1 driver enable switch.
 * @details If set to @p TRUE the support for MCU_I2C0 is included.
 */
#if !defined(AM67_I2C_USE_MCU_I2C0) || defined(__DOXYGEN__)
#define AM67_I2C_USE_MCU_I2C0    FALSE
#endif

/**
 * @brief   MCU_I2C0 interrupt priority level setting.
 */
#if !defined(AM67_I2C_MCU_I2C0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define AM67_I2C_MCU_I2C0_IRQ_PRIORITY    0x8U
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if AM67_I2C_USE_MCU_I2C0 == FALSE
#error "I2C driver activated but no I2C peripheral assigned"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type representing an I2C address.
 */
typedef uint16_t i2caddr_t;

/**
 * @brief   Type of I2C driver condition flags.
 */
typedef uint32_t i2cflags_t;

/**
 * @brief   I2C driver configuration structure.
 */
struct hal_i2c_config {
  /**
   * @brief   SCL frequency in Hz (standard 100000 or fast 400000).
   */
  uint32_t                  frequency;
};

/**
 * @brief   Type of a structure representing an I2C configuration.
 */
typedef struct hal_i2c_config I2CConfig;

/**
 * @brief   Type of a structure representing an I2C driver.
 */
typedef struct hal_i2c_driver I2CDriver;

/**
 * @brief   Structure representing an I2C driver.
 */
struct hal_i2c_driver {
  /**
   * @brief   Driver state.
   */
  i2cstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2CConfig           *config;
  /**
   * @brief   Error flags.
   */
  i2cflags_t                errors;
#if (I2C_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  mutex_t                   mutex;
#endif
#if defined(I2C_DRIVER_EXT_FIELDS)
  I2C_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief   I2C registers base address.
   */
  uint32_t                  base;
  /**
   * @brief   Functional clock frequency.
   */
  uint32_t                  clock;
  /**
   * @brief   False when the module failed to come out of reset.
   */
  bool                      ready;
  /**
   * @brief   Reason the last initialization failed, one of I2C_INIT_*.
   */
  uint32_t                  init_error;
  /**
   * @brief   Controller registers sampled when a transfer times out.
   * @details Bring-up diagnostic: a timeout says nothing about whether
   *          the clock was configured, the bus was held or the interrupt
   *          simply never arrived. Strip once the bus is trusted.
   */
  struct {
    uint32_t                con;
    uint32_t                raw;
    uint32_t                scll;
    uint32_t                irqen;
    uint32_t                vim;
  }                         dbg;
  /**
   * @brief   Thread waiting for the transfer to end.
   */
  thread_reference_t        thread;
  /**
   * @brief   Slave address of the current transfer.
   */
  i2caddr_t                 addr;
  /**
   * @brief   Transmit pointer of the current transfer.
   */
  const uint8_t             *txptr;
  /**
   * @brief   Transmit bytes still to be sent.
   */
  size_t                    txbytes;
  /**
   * @brief   Receive pointer of the current transfer.
   */
  uint8_t                   *rxptr;
  /**
   * @brief   Receive bytes still to be read.
   */
  size_t                    rxbytes;
  /**
   * @brief   True once the receive segment has been programmed.
   */
  bool                      rx_started;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Get errors from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_errors(i2cp) ((i2cp)->errors)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if (AM67_I2C_USE_MCU_I2C0 == TRUE) && !defined(__DOXYGEN__)
extern I2CDriver I2CD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void i2c_lld_init(void);
  void i2c_lld_start(I2CDriver *i2cp);
  void i2c_lld_stop(I2CDriver *i2cp);
  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                        const uint8_t *txbuf, size_t txbytes,
                                        uint8_t *rxbuf, size_t rxbytes,
                                        sysinterval_t timeout);
  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                       uint8_t *rxbuf, size_t rxbytes,
                                       sysinterval_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C == TRUE */

#endif /* HAL_I2C_LLD_H */

/** @} */
