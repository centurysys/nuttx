/****************************************************************************
 * boards/arm/sama5/mas1xx/src/sam_userleds.c
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

/* There is Red/Green LEDs on board the MAS1xx.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PC1                            GPIO_PC1            MobileLED0(G)
 *   PC2                            GPIO_PC2            MobileLED0(R)
 *   PC3                            GPIO_PC3            MobileLED1(G)
 *   PC4                            GPIO_PC4            MobileLED1(R)
 *   PC5                            GPIO_PC5            PowerLED(G)
 *   ------------------------------ ------------------- ---------------------
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "sam_pio.h"
#include "mas1xx.h"

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct led_bits
{
  uint32_t bit;
  uint32_t cfg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct led_bits leds[] =
{
  {
    .bit = BOARD_MOBILE0_G_BIT,
    .cfg = PIO_LED_MOBILE0_G
  },
  {
    .bit = BOARD_MOBILE0_R_BIT,
    .cfg = PIO_LED_MOBILE0_R
  },
  {
    .bit = BOARD_MOBILE1_G_BIT,
    .cfg = PIO_LED_MOBILE1_G
  },
  {
    .bit = BOARD_MOBILE1_R_BIT,
    .cfg = PIO_LED_MOBILE1_R
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED PIOs for output */

  sam_configpio(PIO_LED_MOBILE0_G);
  sam_configpio(PIO_LED_MOBILE0_R);
  sam_configpio(PIO_LED_MOBILE1_G);
  sam_configpio(PIO_LED_MOBILE1_R);
  sam_configpio(PIO_LED_ENABLE);
  sam_piowrite(PIO_LED_ENABLE, true);
#ifndef CONFIG_ARCH_LEDS
  sam_configpio(PIO_LED_POWER_G);
  sam_piowrite(PIO_LED_POWER_G, true);
  _info("PowerLED initialized.\n");
#endif

  return ARRAY_SIZE(leds);
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  switch (led)
    {
      case BOARD_MOBILE0_G:
        ledcfg = PIO_LED_MOBILE0_G;
        break;

      case BOARD_MOBILE0_R:
        ledcfg = PIO_LED_MOBILE0_R;
        break;

      case BOARD_MOBILE1_G:
        ledcfg = PIO_LED_MOBILE1_G;
        break;

      case BOARD_MOBILE1_R:
        ledcfg = PIO_LED_MOBILE1_R;
        break;

      default:
        return;
    }

  /* Low illuminates */

  sam_piowrite(ledcfg, !ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  int i;
  bool ledon;
  const struct led_bits *led;

  for (i = 0; i < ARRAY_SIZE(leds); i++)
    {
      led = &leds[i];
      ledon = (ledset & led->bit) != 0;
      sam_piowrite(led->cfg, !ledon);
    }
}
