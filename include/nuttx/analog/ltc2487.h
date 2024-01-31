/****************************************************************************
 * include/nuttx/analog/ltc2487.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_LTC2487_H
#define __INCLUDE_NUTTX_ANALOG_LTC2487_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands
 * Cmd: ANIOC_LTC2487_SET_REF       Arg: enum ltc2487_ref_e
 * Cmd: ANIOC_LTC2487_MODE          Arg: ltc2487_mode_e
 * Cmd: ANIOC_LTC2487_ADD_CHAN      Arg: uint8_t value
 * Cmd: ANIOC_LTC2487_REMOVE_CHAN   ARG: uint8_t value
 * Cmd: ANIOC_LTC2487_READ_CHANNEL  Arg: struct adc_msg_s *channel
 */

#define ANIOC_LTC2487_SET_REF       _ANIOC(AN_LTC2487_FIRST + 0)
#define ANIOC_LTC2487_MODE          _ANIOC(AN_LTC2487_FIRST + 1)
#define ANIOC_LTC2487_ADD_CHAN      _ANIOC(AN_LTC2487_FIRST + 2)
#define ANIOC_LTC2487_REMOVE_CHAN   _ANIOC(AN_LTC2487_FIRST + 3)
#define ANIOC_LTC2487_READ_CHANNEL  _ANIOC(AN_LTC2487_FIRST + 4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ltc2487_ref_e
{
  LTC2487_REF_EXTERNAL = 0u,
  LTC2487_REF_INTERNAL
};

enum ltc2487_mode_e
{
  LTC2487_DIFFERENTIAL = 0u,
  LTC2487_SINGLE_ENDED
};

enum ltc2487_diffch_e
{
  LTC2487_DIFF_PCH0_NCH1 = 0u,
  LTC2487_DIFF_PCH2_NCH3,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct adc_dev_s *ltc2487_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr);
int ltc2487_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr);

#endif /* __INCLUDE_NUTTX_ANALOG_LTC2487_H */
