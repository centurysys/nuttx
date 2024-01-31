/****************************************************************************
 * boards/arm/sama5/mas1xx/src/sam_gpio.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ioexpander/gpio.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "mas1xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct samgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint32_t pin;
  const char *name;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value);
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

static struct samgpio_dev_s g_gpin[] =
{
  {
    .pin = PIO_DIN0,
    .name = "DI0",
  },
  {
    .pin = PIO_DIN1,
    .name = "DI1",
  },
  {
    .pin = PIO_DIN2,
    .name = "DI2",
  },
  {
    .pin = PIO_DIN3,
    .name = "DI3",
  },
};

static struct samgpio_dev_s g_gpout[] =
{
  {
    .pin = PIO_DO0,
    .name = "DO0",
  },
  {
    .pin = PIO_DO1,
    .name = "DO1",
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  gpioinfo("Reading %s\n", samgpio->name);

  *value = !sam_pioread(samgpio->pin);
  return OK;
}

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  gpioinfo("Reading %s\n", samgpio->name);

  *value = !sam_pioread(samgpio->pin);
  return OK;
}

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct samgpio_dev_s *samgpio = (struct samgpio_dev_s *)dev;

  gpioinfo("Writing %s <- %d\n", samgpio->name, (int)(!value));

  sam_piowrite(samgpio->pin, !value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gpio_initialize
 *
 ****************************************************************************/

void sam_gpio_initialize(void)
{
  int i;
  struct samgpio_dev_s *gpio;

  for (i = 0; i < ARRAY_SIZE(g_gpin); i++)
    {
      gpio = &g_gpin[i];

      gpio->gpio.gp_pintype = GPIO_INPUT_PIN;
      gpio->gpio.gp_ops = &gpin_ops;

      gpio_pin_register_byname(&gpio->gpio, gpio->name);

      sam_configpio(gpio->pin);
    }

  for (i = 0; i < ARRAY_SIZE(g_gpout); i++)
    {
      gpio = &g_gpout[i];

      gpio->gpio.gp_pintype  = GPIO_OUTPUT_PIN;
      gpio->gpio.gp_ops = &gpout_ops;

      gpio_pin_register_byname(&gpio->gpio, gpio->name);

      sam_configpio(gpio->pin);
    }
}
