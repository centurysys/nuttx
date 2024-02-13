/****************************************************************************
 * boards/arm/sama5/mas1xx/src/mas1xx_led.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <fcntl.h>
#include <debug.h>

#if defined(CONFIG_IOEXPANDER_TCA6507)
#  include <nuttx/ioexpander/tca6507.h>
#  include <nuttx/ioexpander/ioexpander.h>
#  include <nuttx/ioexpander/gpio.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sam_twi.h"
#include "mas1xx_param.h"
#include "mas1xx_xio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XIO_I2C_BUS  (1)
#define TCA6507_I2C_ADDR (0x45)
#define TCA6507_I2C_FREQ (100000)

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

struct leds
{
  int idx;
  char *name;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool initialized = false;

#if defined(CONFIG_IOEXPANDER_TCA6507)
struct tca6507_config_s g_tca6507_cfg =
{
  .address        = TCA6507_I2C_ADDR,
  .frequency      = TCA6507_I2C_FREQ,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_IOEXPANDER_TCA6507)
/****************************************************************************
 * Name: tca6507_pincfg
 ****************************************************************************/

static void tca6507_pincfg(struct ioexpander_dev_s *ioe)
{
  int i;
  const struct leds leds[] = {
    { .idx = 0, .name = "LED_Power_R" },
    { .idx = 1, .name = "LED_Stat1_R" },
    { .idx = 2, .name = "LED_Stat2_R" },
    { .idx = 3, .name = "LED_Stat3_R" },
    { .idx = 4, .name = "LED_Stat1_G" },
    { .idx = 5, .name = "LED_Stat2_G" },
    { .idx = 6, .name = "LED_Stat3_G" }
  };

  for (i = 0; i < ARRAY_SIZE(leds); i++)
    {
      IOEXP_WRITEPIN(ioe, i, false);
      gpio_lower_half_byname(ioe, i, GPIO_OUTPUT_PIN, leds[i].name);

      _info("Register LED %s.\n", leds[i].name);
    }
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mas1xx_led_initialize
 ****************************************************************************/

int mas1xx_led_initialize(void)
{
  struct i2c_master_s     *i2c = NULL;
  struct ioexpander_dev_s *ioe = NULL;
  int ret = OK;

  if (initialized)
    {
      return OK;
    }

#if defined(CONFIG_IOEXPANDER_TCA6507)
  i2c = sam_i2cbus_initialize(1);

  if (!i2c)
    {
      _err("%s: failed to get I2c1 interface\n", __FUNCTION__);
      return -ENODEV;
    }

  ioe = tca6507_initialize(i2c, &g_tca6507_cfg);
  if (!ioe)
    {
      _err("%s: Failed to initialize TCA6507\n", __FUNCTION__);
      return ERROR;
    }

  tca6507_pincfg(ioe);
#endif

  initialized = true;
  return ret;
}
