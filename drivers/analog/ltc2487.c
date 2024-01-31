/****************************************************************************
 * drivers/analog/ltc2487.c
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <endian.h>
#include <sys/time.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ltc2487.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ADC_LTC2487)

#define LTC2487_NUM_CHANNELS            2

/* All conversions are started by writing a command byte to the LTC2487.
 * Below are the bit/ mask definitions for this byte.
 */

#define LTC2487_ENABLE              0xA0
#define LTC2487_SGL                 BIT(4)
#define LTC2487_DIFF                0
#define LTC2487_SIGN                BIT(3)
#define LTC2487_CONFIG_DEFAULT      LTC2487_ENABLE
#define LTC2487_CONVERSION_TIME_MS  (170ULL)
#define LTC2487_CONVERSION_TIME_US  (LTC2487_CONVERSION_TIME_MS * 1000ULL)
#define LTC2487_CONVERSION_TIME_NS  (LTC2487_CONVERSION_TIME_US * 1000ULL)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ltc2487_dev_s
{
  FAR struct i2c_master_s           *i2c;
  FAR const struct adc_callback_s   *cb;
  uint8_t                           addr;

  /* List of channels to read on every convert trigger.
   * Bit position corresponds to channel. i.e. bit0 = 1 to read channel 0.
   */

  uint8_t                           chanstrobed;

  uint8_t                           chan_prev;

  hrtime_t                          time_prev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ltc2487 helpers */

static int ltc2487_manage_strobe(FAR struct ltc2487_dev_s *priv,
                                 uint8_t channel, bool add_nremove);
static int ltc2487_wait_conv(FAR struct ltc2487_dev_s *priv);
static int ltc2487_readchannel(FAR struct ltc2487_dev_s *priv,
                               FAR struct adc_msg_s *msg);

/* ADC methods */

static int  ltc2487_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback);
static void ltc2487_reset(FAR struct adc_dev_s *dev);
static int  ltc2487_setup(FAR struct adc_dev_s *dev);
static void ltc2487_shutdown(FAR struct adc_dev_s *dev);
static void ltc2487_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  ltc2487_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  ltc2487_bind,      /* ao_bind */
  ltc2487_reset,     /* ao_reset */
  ltc2487_setup,     /* ao_setup */
  ltc2487_shutdown,  /* ao_shutdown */
  ltc2487_rxint,     /* ao_rxint */
  ltc2487_ioctl      /* ao_read */
};

static struct ltc2487_dev_s g_adcpriv;

static struct adc_dev_s g_adcdev =
{
  &g_adcops,    /* ad_ops */
  &g_adcpriv    /* ad_priv */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltc2487_manage_strobe
 *
 * Description:
 *   Controls which channels are read on ANIOC_TRIGGER. By default all
 *   channels are read.
 *
 * Returned Value:
 *   0 on success. Negated errno on failure.
 *
 ****************************************************************************/

static int ltc2487_manage_strobe(FAR struct ltc2487_dev_s *priv,
                                 uint8_t channel, bool add_nremove)
{
  int ret = OK;

  if (priv == NULL || channel >= LTC2487_NUM_CHANNELS)
    {
      ret = -EINVAL;
    }
  else
    {
      uint8_t flag = BIT(channel);

      if (add_nremove)
        {
          priv->chanstrobed |= flag;
        }
       else
        {
          priv->chanstrobed &= ~flag;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ltc2487_wait_conv
 ****************************************************************************/

static int ltc2487_wait_conv(FAR struct ltc2487_dev_s *priv)
{
  int64_t time_elapsed_usec, time_now;

  time_now = gethrtime();
  time_elapsed_usec = (time_now - priv->time_prev) / 1000;

  if (time_elapsed_usec < LTC2487_CONVERSION_TIME_US)
    {
      /* delay if conversion time not passed
       * since last read or write
       */
      if (nxsig_usleep(LTC2487_CONVERSION_TIME_US - time_elapsed_usec) < 0)
        {
          return -EAGAIN;
        }

      return 0;
    }

  if ((time_elapsed_usec - LTC2487_CONVERSION_TIME_US) <= 0)
    {
      /* We're in automatic mode -
       * so the last reading is stil not outdated
       */
      return 0;
    }

  return 1;
}

/****************************************************************************
 * Name: ltc2487_readchannel
 *
 * Description:
 *   Reads a conversion from the ADC.
 *
 * Input Parameters:
 *   msg - msg->am_channel should be set to the channel to be read.
 *         msg->am_data will store the result of the read.
 *
 * Returned Value:
 *   0 on success. Negated errno on failure.
 *
 * Assumptions/Limitations:
 *   NOTE: When used in single-ended mode, msg->am_channel will be converted
 *         to the corresponding channel selection bits in the command byte.
 *         In differential mode, msg->am_channel is used as the channel
 *         selection bits. The corresponding "channels" are as follows:
 *
 *         msg->am_channel  Analog Source
 *         0                +CH0, -CH1
 *         1                +CH2, -CH3
 *
 ****************************************************************************/

static int ltc2487_readchannel(FAR struct ltc2487_dev_s *priv,
                               FAR struct adc_msg_s *msg)
{
  int ret = OK;

  if (priv == NULL || msg == NULL)
    {
      ret = -EINVAL;
    }
  else
    {
      struct i2c_msg_s i2cmsg;
      uint8_t buf[2];
      uint32_t output;
      uint8_t channel = msg->am_channel;

      ret = ltc2487_wait_conv(priv);

      if (ret < 0)
        {
          return ret;
        }

      if (ret || priv->chan_prev != channel)
        {
          buf[0] = LTC2487_CONFIG_DEFAULT | channel;
          buf[1] = 0x80;

          i2cmsg.frequency = CONFIG_LTC2487_FREQUENCY;
          i2cmsg.addr = priv->addr;
          i2cmsg.flags = 0;
          i2cmsg.buffer = buf;
          i2cmsg.length = 2;

          ret = I2C_TRANSFER(priv->i2c, &i2cmsg, 1);

          if (ret < 0)
            {
              aerr("(setup) LTC2487 I2C transfer failed: channel: %d, ret: %d",
                   channel, ret);
              return ret;
            }
          priv->chan_prev = channel;
          nxsig_usleep(LTC2487_CONVERSION_TIME_US);
        }

      i2cmsg.frequency = CONFIG_LTC2487_FREQUENCY;
      i2cmsg.addr = priv->addr;
      i2cmsg.flags = I2C_M_READ;
      i2cmsg.buffer = (uint8_t *)&output;
      i2cmsg.length = 3;
      ret = I2C_TRANSFER(priv->i2c, &i2cmsg, 1);

      if (ret < 0)
        {
            aerr("LTC2487 I2C transfer failed: %d", ret);
        }

      msg->am_data = ((be32toh(output) >> 14) - (1 << 17));
      priv->time_prev = gethrtime();
    }

  return ret;
}

/****************************************************************************
 * Name: ltc2487_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int ltc2487_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct ltc2487_dev_s *priv = (FAR struct ltc2487_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: ltc2487_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before ao_setup() and on error conditions.
 *
 ****************************************************************************/

static void ltc2487_reset(FAR struct adc_dev_s *dev)
{
  FAR struct ltc2487_dev_s *priv = (FAR struct ltc2487_dev_s *)dev->ad_priv;

  priv->chan_prev = 0xff;
  priv->chanstrobed = 0xffu;
}

/****************************************************************************
 * Name: ltc2487_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  The LTC2487 is quite simple and nothing special is
 *   needed to be done.
 *
 ****************************************************************************/

static int ltc2487_setup(FAR struct adc_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: ltc2487_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method should reverse the operation of the setup method, but as
 *   the LTC2487 is quite simple does not need to do anything.
 *
 ****************************************************************************/

static void ltc2487_shutdown(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: ltc2487_rxint
 *
 * Description:
 *   Needed for ADC upper-half compatibility but conversion interrupts
 *   are not supported by the ADC7828.
 *
 ****************************************************************************/

static void ltc2487_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: ltc2487_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ltc2487_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct ltc2487_dev_s *priv = (FAR struct ltc2487_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          struct adc_msg_s msg;
          int i;

          for (i = 0; (i < LTC2487_NUM_CHANNELS) && (ret == OK); i++)
            {
              if ((priv->chanstrobed >> i) & 1u)
                {
                  msg.am_channel = i;
                  ret = ltc2487_readchannel(priv, &msg);
                  if (ret == OK)
                    {
                      priv->cb->au_receive(&g_adcdev, i, msg.am_data);
                    }
                }
            }
        }
        break;

      /* Add a channel to list of channels read on ANIOC_TRIGGER */

      case ANIOC_LTC2487_ADD_CHAN:
        {
          ret = ltc2487_manage_strobe(priv, (uint8_t)arg, true);
        }
        break;

      /* Remove a channel from list of channels read on ANIOC_TRIGGER */

      case ANIOC_LTC2487_REMOVE_CHAN:
        {
          ret = ltc2487_manage_strobe(priv, (uint8_t)arg, false);
        }
        break;

      /* Read a single channel from the ADC */

      case ANIOC_LTC2487_READ_CHANNEL:
        {
          FAR struct adc_msg_s *msg = (FAR struct adc_msg_s *)arg;
          ret = ltc2487_readchannel(priv, msg);
        }
        break;

      /* Command was not recognized */

      default:
      ret = -ENOTTY;
      aerr("LTC2487 ERROR: Unrecognized cmd: %d\n", cmd);
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ltc2487_initialize
 *
 * Description:
 *   Initialize the selected adc
 *
 * Input Parameters:
 *   i2c - Pointer to a I2C master struct for the bus the ADC resides on.
 *   addr - I2C address of the ADC
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *ltc2487_initialize(FAR struct i2c_master_s *i2c,
                                         uint8_t addr)
{
  DEBUGASSERT(i2c != NULL);

  /* Driver state data */

  FAR struct ltc2487_dev_s *priv;
  priv = (FAR struct ltc2487_dev_s *)g_adcdev.ad_priv;

  priv->cb   = NULL;
  priv->i2c  = i2c;
  priv->addr = addr;
  priv->chan_prev = 0xff;
  priv->chanstrobed = 0xff;

  return &g_adcdev;
}

/****************************************************************************
 * Name: ltc1867l_register
 *
 * Description:
 *   Register the LTC2487 character device as 'devpath'
 *
 ****************************************************************************/

int ltc2487_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr)
{
  FAR struct adc_dev_s *adcdev;
  int ret;

  adcdev = ltc2487_initialize(i2c, addr);
  if (!adcdev)
    {
      return ERROR;
    }

  ret = adc_register(devpath, adcdev);

  return ret;
}

#endif
