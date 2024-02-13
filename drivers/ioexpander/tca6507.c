/****************************************************************************
 * drivers/ioexpander/tca6507.c
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

#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "tca6507.h"

#if defined(CONFIG_IOEXPANDER_TCA6507)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_I2C
#  warning I2C support is required (CONFIG_I2C)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int tca6507_write(struct tca6507_dev_s *tca,
                                const uint8_t *wbuffer, int wbuflen);
static inline int tca6507_writeread(struct tca6507_dev_s *tca,
                                    const uint8_t *wbuffer, int wbuflen,
                                    uint8_t *rbuffer, int rbuflen);
static int tca6507_direction(struct ioexpander_dev_s *dev, uint8_t pin,
                             int dir);
static int tca6507_option(struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value);
static int tca6507_writepin(struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value);
static int tca6507_readpin(struct ioexpander_dev_s *dev, uint8_t pin,
                           bool *value);
static int tca6507_readbuf(struct ioexpander_dev_s *dev, uint8_t pin,
                           bool *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_TCA6507_MULTIPLE
/* If only a single TCA6507 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

static struct tca6507_dev_s g_tca6507;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct tca6507_dev_s *g_tca6507list;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_tca6507_ops =
{
  tca6507_direction,
  tca6507_option,
  tca6507_writepin,
  tca6507_readpin,
  tca6507_readbuf,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tca6507_direction(struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  return OK;
}

static int tca6507_option(struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *value)
{
  return OK;
}

/****************************************************************************
 * Name: tca6507_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int tca6507_write(struct tca6507_dev_s *tca, const uint8_t *wbuffer,
                                int wbuflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = tca->config->frequency;
  msg.addr      = tca->config->address;
  msg.flags     = 0;
  msg.buffer    = (uint8_t *)wbuffer;  /* Override const */
  msg.length    = wbuflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(tca->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: tca6507_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static inline int tca6507_writeread(struct tca6507_dev_s *tca,
                                    const uint8_t *wbuffer, int wbuflen,
                                    uint8_t *rbuffer, int rbuflen)
{
  struct i2c_config_s config;

  /* Set up the configuration and perform the write-read operation */

  config.frequency = tca->config->frequency;
  config.address   = tca->config->address;
  config.addrlen   = 7;

  return i2c_writeread(tca->i2c, &config, wbuffer,
                       wbuflen, rbuffer, rbuflen);
}

/****************************************************************************
 * Name: tca6507_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int tca6507_setbit(struct tca6507_dev_s *tca, uint8_t pin, bool bitval)
{
  uint8_t buf[5];
  int ret;

  if (pin >= TCA6507_GPIO_NPINS)
    {
      return -ENXIO;
    }

  buf[0] = 0x10;
  buf[1] = 0;
  buf[2] = 0;

  /* Get the shadowed register value */

  buf[3] = tca->sreg;

  if (bitval)
    {
      buf[3] |= (1 << pin);
    }
  else
    {
      buf[3] &= ~(1 << pin);
    }

  /* Save the new register value in the shadow register */

  tca->sreg = buf[3];

  ret = tca6507_write(tca, buf, 4);

#ifdef CONFIG_TCA6507_RETRY
  if (ret != OK)
    {
      /* Try again (only once) */

      ret = tca6507_write(tca, buf, 4);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: tca6507_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int tca6507_getbit(struct tca6507_dev_s *tca, uint8_t pin, bool *val)
{
  uint8_t buf;
  int ret;

  if (pin >= TCA6507_GPIO_NPINS)
    {
      return -ENXIO;
    }

  *val = (buf >> pin) & 1;
  return OK;
}

/****************************************************************************
 * Name: tca6507_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca6507_writepin(struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  struct tca6507_dev_s *tca = (struct tca6507_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA6507 */

  ret = nxmutex_lock(&tca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = tca6507_setbit(tca, pin, value);
  nxmutex_unlock(&tca->lock);
  return ret;
}

/****************************************************************************
 * Name: tca6507_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored.
 *            Usually TRUE if the pin is high, except if OPTION_INVERT
 *            has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca6507_readpin(struct ioexpander_dev_s *dev, uint8_t pin,
                           bool *value)
{
   struct tca6507_dev_s *tca = ( struct tca6507_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA6507 */

  ret = nxmutex_lock(&tca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = tca6507_getbit(tca, pin, value);
  nxmutex_unlock(&tca->lock);
  return ret;
}

/****************************************************************************
 * Name: tca6507_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the level is stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca6507_readbuf(struct ioexpander_dev_s *dev, uint8_t pin,
                           bool *value)
{
  struct tca6507_dev_s *tca = (struct tca6507_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA6507 */

  ret = nxmutex_lock(&tca->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = tca6507_getbit(tca, pin, value);
  nxmutex_unlock(&tca->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca6507_initialize
 *
 * Description:
 *   Initialize a TCA6507 I2C device.
 *
 ****************************************************************************/

 struct ioexpander_dev_s *tca6507_initialize(struct i2c_master_s *i2cdev,
                                             struct tca6507_config_s *config)
{
  struct tca6507_dev_s *tcadev;

#ifdef CONFIG_TCA6507_MULTIPLE
  /* Allocate the device state structure */

  tcadev = kmm_zalloc(sizeof(struct tca6507_dev_s));
  if (!tcadev)
    {
      return NULL;
    }

  /* And save the device structure in the list of TCA6507 so that we can
   * find it later.
   */

  tcadev->flink = g_tca6507list;
  g_tca6507list = tcadev;

#else
  /* Use the one-and-only TCA6507 driver instance */

  tcadev = &g_tca6507;
#endif

  /* Initialize the device state structure */

  tcadev->i2c     = i2cdev;
  tcadev->dev.ops = &g_tca6507_ops;
  tcadev->config  = config;

  nxmutex_init(&tcadev->lock);
  return &tcadev->dev;
}

#endif /* CONFIG_IOEXPANDER_TCA6507 */
