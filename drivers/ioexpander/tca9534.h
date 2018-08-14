/********************************************************************************************
 * drivers/ioexpander/tca9534.h
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
 ********************************************************************************************/

#ifndef __DRIVERS_IOEXPANDER_TCA9534_H
#define __DRIVERS_IOEXPANDER_TCA9534_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>

#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/tca9534.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_TCA9534)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Configuration ****************************************************************************/
/* Prerequisites:
 *   CONFIG_I2C
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables support for the TCA9534 I/O expander
 *
 * Other settings that effect the driver: CONFIG_DISABLE_POLL
 *
 * CONFIG_IOEXPANDER_TCA9534
 *   Enables support for the TCA9534 driver (Needs CONFIG_INPUT)
 * CONFIG_TCA9534_MULTIPLE
 *   Can be defined to support multiple TCA9534 devices on board.
 * CONFIG_TCA9534_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_TCA9534_INT_NCALLBACKS
#    define CONFIG_TCA9534_INT_NCALLBACKS 4
#  endif
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#undef CONFIG_TCA9534_REFCNT

/* Driver support ***************************************************************************/
/* This format is used to construct the /dev/input[n] device driver path.  It defined here
 * so that it will be used consistently in all places.
 */

/* TCA9534 Resources ************************************************************************/

#define TCA9534_GPIO_NPINS 8 /* All pins can be used as GPIOs */

#ifndef CONFIG_I2C
#error "CONFIG_I2C is required by TCA9534"
#endif

#define TCA9534_MAXDEVS             8

/* I2C frequency */

#define TCA9534_I2C_MAXFREQUENCY    400000       /* 400KHz */

/* TCA9534 Registers ************************************************************************/
/* Register Addresses */

#define TCA9534_REG_INPUT  0x00
#define TCA9534_REG_OUTPUT 0x01
#define TCA9534_REG_POLINV 0x02
#define TCA9534_REG_CONFIG 0x03

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct tca9534_callback_s
{
   ioe_pinset_t pinset;                 /* Set of pin interrupts that will generate
                                         * the callback. */
   ioe_callback_t cbfunc;               /* The saved callback function pointer */
   FAR void *cbarg;                       /* Callback argument */
};
#endif

/* This structure represents the state of the TCA9534 driver */

struct tca9534_dev_s
{
  struct ioexpander_dev_s      dev;     /* Nested structure to allow casting as public gpio
                                         * expander. */
#ifdef CONFIG_TCA9534_SHADOW_MODE
  uint8_t sreg[4];                      /* Shadowed registers of the TCA9534 */
#endif
#ifdef CONFIG_TCA9534_MULTIPLE
  FAR struct tca9534_dev_s    *flink;   /* Supports a singly linked list of drivers */
#endif
  FAR struct tca9534_config_s *config;  /* Board configuration data */
  FAR struct i2c_master_s     *i2c;     /* Saved I2C driver instance */
  sem_t                        exclsem; /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct work_s work;                   /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct tca9534_callback_s cb[CONFIG_TCA9534_INT_NCALLBACKS];
#endif
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_TCA9534 */
#endif /* __DRIVERS_IOEXPANDER_TCA9534_H */
