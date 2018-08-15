/****************************************************************************
 * include/nuttx/ioexpander/tca9534.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * References:
 *   "16-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_IOEXPANDER_TCA9534_H
#define __INCLUDE_NUTTX_IOEXPANDER_TCA9534_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the TCA9534
 * driver when the driver is instantiated. This structure provides
 * information about the configuration of the TCA9534 and provides some
 * board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by
 * the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify the frequency.
 */

struct tca9534_config_s
{
  /* Device characterization */

  uint8_t address;     /* 7-bit I2C address (only bits 0-6 used) */
  uint32_t frequency;  /* I2C or SPI frequency */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* If multiple TCA9534 devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the TCA9534 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the TCA9534 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   */

  CODE int  (*attach)(FAR struct tca9534_config_s *state, xcpt_t isr,
                      FAR void *arg);
  CODE void (*enable)(FAR struct tca9534_config_s *state, bool enable);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: tca9534_initialize
 *
 * Description:
 *   Instantiate and configure the TCA9534 device driver to use the provided
 *   I2C device
 *   instance.
 *
 * Input Parameters:
 *   dev     - An I2C driver instance
 *   minor   - The device i2c address
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   an ioexpander_dev_s instance on success, NULL on failure.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *tca9534_initialize(FAR struct i2c_master_s *dev,
                                                FAR struct tca9534_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_TCA9534_H */
