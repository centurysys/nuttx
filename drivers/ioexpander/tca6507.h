/****************************************************************************
 * drivers/ioexpander/tca6507.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_IOEXPANDER_TCA6507_H
#define __DRIVERS_IOEXPANDER_TCA6507_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/tca6507.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_TCA6507)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Prerequisites:
 *   CONFIG_I2C
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables support for the TCA6507 I/O expander
 *
 * CONFIG_IOEXPANDER_TCA6507
 *   Enables support for the TCA6507 driver (Needs CONFIG_INPUT)
 */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_TCA6507_INT_NCALLBACKS
#    define CONFIG_TCA6507_INT_NCALLBACKS 4
#  endif
#endif

#undef CONFIG_TCA6507_REFCNT

/* TCA6507 Resources ********************************************************/

#define TCA6507_GPIO_NPINS 7

#ifndef CONFIG_I2C
#error "CONFIG_I2C is required by tca6507"
#endif

#define TCA6507_MAXDEVS             4

/* I2C frequency */

#define TCA6507_I2C_MAXFREQUENCY    400000       /* 400KHz */

/* TCA6507 Registers ********************************************************/

#define TCA6507_REG_SELECT0 0x00
#define TCA6507_REG_SELECT1 0x01
#define TCA6507_REG_SELECT2 0x02

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the state of the TCA6507 driver */

struct tca6507_dev_s
{
  struct ioexpander_dev_s      dev;      /* Nested structure to allow casting
                                          * as public gpio expander. */
  uint8_t sreg;                          /* Shadowed registers of the TCA6507 */
#ifdef CONFIG_TCA6507_MULTIPLE
  FAR struct tca6507_dev_s    *flink;    /* Supports a singly linked list of drivers */
#endif
  FAR struct tca6507_config_s *config;   /* Board configuration data */
  FAR struct i2c_master_s     *i2c;      /* Saved I2C driver instance */
  mutex_t                      lock;     /* Mutual exclusion */
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_TCA6507 */
#endif /* __DRIVERS_IOEXPANDER_TCA6507_H */
