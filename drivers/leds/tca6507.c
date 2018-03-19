/****************************************************************************
 * drivers/leds/tca6507.c
 * based on drivers/leds/ncp5623c.c
 *
 *   Author: Takeyoshi Kikuchi <kikuchi@centurysys.co.jp>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/tca6507.h>

#if defined(CONFIG_I2C) && defined(CONFIG_TCA6507)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct tca6507_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t i2c_addr;

  uint8_t reg_sel2;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tca6507_i2c_write_byte(FAR struct tca6507_dev_s *priv,
                                  uint8_t const reg_addr, uint8_t const reg_val);

static int tca6507_open(FAR struct file *filep);
static int tca6507_close(FAR struct file *filep);
static int tca6507_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static ssize_t tca6507_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_tca6507_fileops =
{
  tca6507_open,               /* open  */
  tca6507_close,              /* close */
  tca6507_read,               /* read  */
  0,                          /* write */
  0,                          /* seek  */
  tca6507_ioctl,              /* ioctl */
  0                           /* poll  */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca6507_i2c_write_byte
 *
 * Description:
 *   Write a single byte to one of the TCA6507 configuration registers.
 *
 ****************************************************************************/

static int tca6507_i2c_write_byte(FAR struct tca6507_dev_s *priv,
                                  uint8_t const reg_addr,
                                  uint8_t const reg_val)
{
  struct i2c_config_s config;
  int ret = OK;

  uint8_t const BUFFER_SIZE = 2;
  uint8_t buffer[BUFFER_SIZE];

  buffer[0] = reg_addr;
  buffer[1] = reg_val;

  /* Setup up the I2C configuration */

  config.frequency = I2C_BUS_FREQ_HZ;
  config.address   = priv->i2c_addr;
  config.addrlen   = 7;

  /* Write the data (no RESTART) */

  lcdinfo("i2c addr: 0x%02X value: 0x%02X\n", priv->i2c_addr,
          buffer[0]);

  ret = i2c_write(priv->i2c, &config, buffer, BUFFER_SIZE);
  if (ret != OK)
    {
      lcderr("ERROR: i2c_write returned error code %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: tca6507_open
 *
 * Description:
 *   This function is called whenever a TCA6507 device is opened.
 *
 ****************************************************************************/

static int tca6507_open(FAR struct file *filep)
{
  /* Let the chip settle a bit */

  nxsig_usleep(1);
  return OK;
}

/****************************************************************************
 * Name: tca6507_close
 *
 * Description:
 *   This function is called whenever a TCA6507 device is closed.
 *
 ****************************************************************************/

static int tca6507_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: tca6507_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t tca6507_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}


/****************************************************************************
 * Name: tca6507_ioctl
 *
 * Description:
 *   This function is called whenever an ioctl call to a TCA6507 is performed.
 *
 ****************************************************************************/

static int tca6507_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct tca6507_dev_s *priv = inode->i_private;
  int ret = OK;

  lcdinfo("cmd: %d arg: %ld\n", cmd, arg);

  switch (cmd)
    {
    case LEDIOC_ONOFF:
      {
        /* Retrieve the information handed over as argument for this ioctl */

        FAR const struct tca6507_onoff_s *ptr =
          (FAR const struct tca6507_onoff_s *)((uintptr_t) arg);

        DEBUGASSERT(ptr != NULL);
        if (ptr->led > TCA6507_MAX_LED)
          {
            lcderr("ERROR: Unrecognized LED: %d\n", ptr->led);
            ret = -EFAULT;
            break;
          }

        if (ptr->on)
          {
            priv->reg_sel2 |= 1 << ptr->led;
          }
        else
          {
            priv->reg_sel2 &= ~(1 << ptr->led);
          }

        ret = tca6507_i2c_write_byte(priv, TCA6507_SELECT2, priv->reg_sel2);
      }
      break;

    case LEDIOC_ALLOFF:
      {
        ret = tca6507_i2c_write_byte(priv, TCA6507_SELECT2, 0);
        priv->reg_sel2 = 0;
      }
      break;

      /* The used ioctl command was invalid */

    default:
      {
        lcderr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
      }
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca6507_register
 *
 * Description:
 *   Register the TCA6507 device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/rgbdrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LED driver.
 *   tca6507_i2c_addr
 *           - The I2C address of the TCA6507.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tca6507_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      uint8_t const tca6507_i2c_addr)
{
  int i, ret;

  /* Sanity check */

  DEBUGASSERT(i2c != NULL);

  /* Initialize the TCA6507 device structure */

  FAR struct tca6507_dev_s *priv =
    (FAR struct tca6507_dev_s *)kmm_malloc(sizeof(struct tca6507_dev_s));

  if (priv == NULL)
    {
      lcderr("ERROR: Failed to allocate instance of tca6507_dev_s\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->i2c_addr = tca6507_i2c_addr;
  priv->reg_sel2 = 0;

  /* Register the character driver */

  ret = register_driver(devpath, &g_tca6507_fileops, 0666, priv);
  if (ret != OK)
    {
      lcderr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* All LED OFF */

  for (i = 0; i < 3; i++)
    {
      ret = tca6507_i2c_write_byte(priv, TCA6507_SELECT(i), 0x00);

      if (ret != OK)
        {
          lcderr("ERROR: Could not write TCA6507 register.\n");
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_I2C_TCA6507 */
