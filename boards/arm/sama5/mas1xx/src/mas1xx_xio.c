/****************************************************************************
 * boards/arm/sama5/mas1xx/src/mas1xx_xio.c
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

#if defined(CONFIG_IOEXPANDER_PCA9538)
#include <nuttx/ioexpander/pca9538.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio.h>
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
#define PCA9538_I2C_ADDR (0x70)
#define PCA9538_I2C_FREQ (100000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool initialized = false;

#if defined(CONFIG_IOEXPANDER_PCA9538)
struct pca9538_config_s g_pca9538_cfg =
{
  .address        = PCA9538_I2C_ADDR,
  .frequency      = PCA9538_I2C_FREQ,
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  .attach         = mas1xx_pca9538_attach,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_IOEXPANDER_PCA9538)
/****************************************************************************
 * Name: xio_02_pincfg
 ****************************************************************************/

static void xio_02_pincfg(struct ioexpander_dev_s *ioe)
{
  /* Pin 0: DC Power OUT */

  IOEXP_SETDIRECTION(ioe, 0, IOEXPANDER_DIRECTION_OUT);
  IOEXP_WRITEPIN(ioe, 0, false);
  gpio_lower_half(ioe, 0, GPIO_OUTPUT_PIN, 0);

  _info("Register \"DC Power OUT\" as GPIO minor 0\n");

  /* Pin 4: ADC Power ON */

  IOEXP_SETDIRECTION(ioe, 4, IOEXPANDER_DIRECTION_OUT);
  IOEXP_WRITEPIN(ioe, 4, true);
  gpio_lower_half(ioe, 4, GPIO_OUTPUT_PIN, 1);

  _info("Register \"ADC Power ON\" as GPIO minor 1\n");
}

/****************************************************************************
 * Name: init_xio_02
 ****************************************************************************/

static int init_xio_02(void)
{
  struct i2c_master_s     *i2c = NULL;
  struct ioexpander_dev_s *ioe = NULL;

  i2c = sam_i2cbus_initialize(1);

  if (!i2c)
    {
      _err("%s: failed to get I2c1 interface\n", __FUNCTION__);
      return -ENODEV;
    }

  ioe = pca9538_initialize(i2c, &g_pca9538_cfg);
  if (!ioe)
    {
      _err("%s: Failed to initialize PCA9538\n", __FUNCTION__);
      return ERROR;
    }

  xio_02_pincfg(ioe);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mas1xx_init_parameter
 ****************************************************************************/

int mas1xx_xio_initialize(void)
{
  uint8_t xio_id;
  int ret;

  ret = mas1xx_param_get_xio_id(&xio_id);
  if (ret != OK)
    {
      _err("%s: could not get XIO_ID\n", __FUNCTION__);
      return ERROR;
    }

  _info("%s: XIO_ID: 0x%02x...\n", __FUNCTION__, xio_id);

  switch (xio_id)
    {
#if defined(CONFIG_IOEXPANDER_PCA9538)
      case 0x02:
        ret = init_xio_02();
        break;
#endif
      default:
        ret = OK;
        break;
    }

  return ret;
}
