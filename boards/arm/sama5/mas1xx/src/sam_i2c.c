/****************************************************************************
 * boards/arm/sama5/mas1xx/src/sam_i2c.c
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

/* EEPROM, LED driver, RTC, TempSensor, IO Expander are available on the MAS1XX
 *
 *  ------------------------------ ------------------- ----------------------
 *  SAMA5D2 I2C-1 address          Device              USAGE
 *  ------------------------------ ------------------- ----------------------
 *  0x32                           DSK324SR            RTC
 *  0x45                           TCA6507             LED Driver
 *  0x49                           TMP75               Temperatur Sensor
 *  0x50                           AT24MAC402          EEPROM
 *  0x70                           TCA9538             IO Expander
 *  ------------------------------ ------------------- ----------------------
 *
 *  Closing PB_USER will bring PB6 to ground so 1) PB6 should have a weak
 *  pull-up,  and 2) when PB_USER is pressed, a low value will be senses.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/eeprom/i2c_xx24xx.h>

#include <arch/board/board.h>

#include "sam_twi.h"
#include "mas1xx.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_i2c_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

void board_i2c_initialize(void)
{
  struct i2c_master_s *i2c;

  i2c = sam_i2cbus_initialize(1);
  if (!i2c)
    {
      _err("%s: failed to get I2c1 interface\n", __FUNCTION__);
      return;
    }

#ifdef CONFIG_I2C_EE_24XX
  ee24xx_initialize(i2c, 0x50, "/dev/eeprom", EEPROM_AT24C02, 1);
#endif
}
