/****************************************************************************
 * drivers/ioexpander/tca9534.c
 *
 *   Copyright (C) 2015, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * References:
 *   "8-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
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

#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "tca9534.h"

#if defined(CONFIG_IOEXPANDER_TCA9534)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_I2C
#  warning I2C support is required (CONFIG_I2C)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int tca9534_write(FAR struct tca9534_dev_s *tca,
             FAR const uint8_t *wbuffer, int wbuflen);
static inline int tca9534_writeread(FAR struct tca9534_dev_s *tca,
             FAR const uint8_t *wbuffer, int wbuflen, FAR uint8_t *rbuffer,
             int rbuflen);
static int tca9534_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int dir);
static int tca9534_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             int opt, void *val);
static int tca9534_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             bool value);
static int tca9534_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
static int tca9534_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
             FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int tca9534_multiwritepin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int tca9534_multireadpin(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
static int tca9534_multireadbuf(FAR struct ioexpander_dev_s *dev,
             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *tca9534_attach(FAR struct ioexpander_dev_s *dev,
             ioe_pinset_t pinset, ioe_callback_t callback, FAR void *arg);
static int tca9534_detach(FAR struct ioexpander_dev_s *dev,
             FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_TCA9534_MULTIPLE
/* If only a single TCA9534 device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

static struct tca9534_dev_s g_tca9534;

/* Otherwise, we will need to maintain allocated driver instances in a list */

#else
static struct tca9534_dev_s *g_tca9534list;
#endif

/* I/O expander vtable */

static const struct ioexpander_ops_s g_tca9534_ops =
{
  tca9534_direction,
  tca9534_option,
  tca9534_writepin,
  tca9534_readpin,
  tca9534_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , tca9534_multiwritepin
  , tca9534_multireadpin
  , tca9534_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , tca9534_attach
  , tca9534_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca9534_lock
 *
 * Description:
 *   Get exclusive access to the TCA9534
 *
 ****************************************************************************/

static void tca9534_lock(FAR struct tca9534_dev_s *tca)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(&tca->exclsem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

#define tca9534_unlock(p) nxsem_post(&(p)->exclsem)

/****************************************************************************
 * Name: tca9534_write
 *
 * Description:
 *   Write to the I2C device.
 *
 ****************************************************************************/

static inline int tca9534_write(FAR struct tca9534_dev_s *tca,
                                FAR const uint8_t *wbuffer, int wbuflen)
{
  struct i2c_msg_s msg;
  int ret;

  /* Setup for the transfer */

  msg.frequency = tca->config->frequency;
  msg.addr      = tca->config->address;
  msg.flags     = 0;
  msg.buffer    = (FAR uint8_t *)wbuffer;  /* Override const */
  msg.length    = wbuflen;

  /* Then perform the transfer. */

  ret = I2C_TRANSFER(tca->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/****************************************************************************
 * Name: tca9534_writeread
 *
 * Description:
 *   Write to then read from the I2C device.
 *
 ****************************************************************************/

static inline int tca9534_writeread(FAR struct tca9534_dev_s *tca,
                                    FAR const uint8_t *wbuffer, int wbuflen,
                                    FAR uint8_t *rbuffer, int rbuflen)
{
  struct i2c_config_s config;

  /* Set up the configuration and perform the write-read operation */

  config.frequency = tca->config->frequency;
  config.address   = tca->config->address;
  config.addrlen   = 7;

  return i2c_writeread(tca->i2c, &config, wbuffer, wbuflen, rbuffer, rbuflen);
}

/****************************************************************************
 * Name: tca9534_setbit
 *
 * Description:
 *  Write a bit in a register pair
 *
 ****************************************************************************/

static int tca9534_setbit(FAR struct tca9534_dev_s *tca, uint8_t addr,
                          uint8_t pin, int bitval)
{
  uint8_t buf[2];
  int ret;

  if (pin > 7)
    {
      return -ENXIO;
    }

  buf[0] = addr;

#ifdef CONFIG_TCA9534_SHADOW_MODE
  /* Get the shadowed register value */

  buf[1] = tca->sreg[addr];

#else
  /* Get the register value from the IO-Expander */

  ret = tca9534_writeread(tca, &buf[0], 1, &buf[1], 1);
  if (ret < 0)
    {
      return ret;
    }
#endif

  if (bitval)
    {
      buf[1] |= (1 << pin);
    }
  else
    {
      buf[1] &= ~(1 << pin);
    }

#ifdef CONFIG_TCA9534_SHADOW_MODE
  /* Save the new register value in the shadow register */

  tca->sreg[addr] = buf[1];
#endif

  ret = tca9534_write(tca, buf, 1);
#ifdef CONFIG_TCA9534_RETRY
  if (ret != OK)
    {
      /* Try again (only once) */

      ret = tca9534_write(tca, buf, 2);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: tca9534_getbit
 *
 * Description:
 *  Get a bit from a register pair
 *
 ****************************************************************************/

static int tca9534_getbit(FAR struct tca9534_dev_s *tca, uint8_t addr,
                          uint8_t pin, FAR bool *val)
{
  uint8_t buf;
  int ret;

  if (pin > 7)
    {
      return -ENXIO;
    }

  ret = tca9534_writeread(tca, &addr, 1, &buf, 1);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_TCA9534_SHADOW_MODE
  /* Save the new register value in the shadow register */

  tca->sreg[addr] = buf;
#endif

  *val = (buf >> pin) & 1;
  return OK;
}

/****************************************************************************
 * Name: tca9534_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca9534_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             int direction)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA555 */

  tca9534_lock(tca);
  ret = tca9534_setbit(tca, TCA9534_REG_CONFIG, pin,
                       (direction == IOEXPANDER_DIRECTION_IN));
  tca9534_unlock(tca);
  return ret;
}

/****************************************************************************
 * Name: tca9534_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca9534_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int opt, FAR void *val)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  int ret = -EINVAL;

  if (opt == IOEXPANDER_OPTION_INVERT)
    {
      int ival = (int)((intptr_t)val);

      /* Get exclusive access to the TCA555 */

      tca9534_lock(tca);
      ret = tca9534_setbit(tca, TCA9534_REG_POLINV, pin, ival);
      tca9534_unlock(tca);
    }

  return ret;
}

/****************************************************************************
 * Name: tca9534_writepin
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

static int tca9534_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            bool value)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA555 */

  tca9534_lock(tca);
  ret = tca9534_setbit(tca, TCA9534_REG_OUTPUT, pin, value);
  tca9534_unlock(tca);
  return ret;
}

/****************************************************************************
 * Name: tca9534_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value written
 *      to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca9534_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA555 */

  tca9534_lock(tca);
  ret = tca9534_getbit(tca, TCA9534_REG_INPUT, pin, value);
  tca9534_unlock(tca);
  return ret;
}

/****************************************************************************
 * Name: tca9534_readbuf
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

static int tca9534_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                           FAR bool *value)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA555 */

  tca9534_lock(tca);
  ret = tca9534_getbit(tca, TCA9534_REG_OUTPUT, pin, value);
  tca9534_unlock(tca);
  return ret;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN

/****************************************************************************
 * Name: tca9534_getmultibits
 *
 * Description:
 *  Read multiple bits from TCA9534 registers.
 *
 ****************************************************************************/

static int tca9534_getmultibits(FAR struct tca9534_dev_s *tca, uint8_t addr,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  uint8_t buf;
  int ret = OK;
  int i;
  int pin;

  ret = tca9534_writeread(tca, &addr, 1, &buf, 1);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_TCA9534_SHADOW_MODE
  /* Save the new register value in the shadow register */

  tca->sreg[addr]   = buf;
#endif

  /* Read the requested bits */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin > 7)
        {
          return -ENXIO;
        }

      values[0] = (buf >> pin) & 1;
    }

  return OK;
}

/****************************************************************************
 * Name: tca9534_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca9534_multiwritepin(FAR struct ioexpander_dev_s *dev,
                                 FAR uint8_t *pins, FAR bool *values,
                                 int count)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  uint8_t addr = TCA9534_REG_OUTPUT;
  uint8_t buf[2];
  int ret;
  int i;
  int index;
  int pin;

  /* Get exclusive access to the TCA555 */

  tca9534_lock(tca);

  /* Start by reading both registers, whatever the pins to change. We could
   * attempt to read one port only if all pins were on the same port, but
   * this would not save much.
   */

#ifndef CONFIG_TCA9534_SHADOW_MODE
  ret = tca9534_writeread(tca, &addr, 1, &buf[1], 1);
  if (ret < 0)
    {
      tca9534_unlock(tca);
      return ret;
    }
#else
  /* In Shadow-Mode we "read" the pin status from the shadow registers */

  buf[1] = tca->sreg[addr];
#endif

  /* Apply the user defined changes */

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin > 7)
        {
          tca9534_unlock(tca);
          return -ENXIO;
        }

      if (values[i])
        {
          buf[1] |= (1 << pin);
        }
      else
        {
          buf[1] &= ~(1 << pin);
        }
    }

  /* Now write back the new pins states */

  buf[0] = addr;
#ifdef CONFIG_TCA9534_SHADOW_MODE
  /* Save the new register values in the shadow register */
  tca->sreg[addr] = buf[1];
#endif
  ret = tca9534_write(tca, buf, 2);

  tca9534_unlock(tca);
  return ret;
}

/****************************************************************************
 * Name: tca9534_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   valptr - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca9534_multireadpin(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA9534 */

  tca9534_lock(tca);
  ret = tca9534_getmultibits(tca, TCA9534_REG_INPUT,
                             pins, values, count);
  tca9534_unlock(tca);
  return ret;
}

/****************************************************************************
 * Name: tca9534_multireadbuf
 *
 * Description:
 *   Read the buffered level of multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the buffered levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca9534_multireadbuf(FAR struct ioexpander_dev_s *dev,
                                FAR uint8_t *pins, FAR bool *values,
                                int count)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  int ret;

  /* Get exclusive access to the TCA555 */

  tca9534_lock(tca);
  ret = tca9534_getmultibits(tca, TCA9534_REG_OUTPUT,
                             pins, values, count);
  tca9534_unlock(tca);
  return ret;
}

#endif

#ifdef CONFIG_TCA9534_INT_ENABLE

/****************************************************************************
 * Name: tca9534_attach
 *
 * Description:
 *   Attach and enable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback.
 *   arg      - User-provided callback argument
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  This handle may be
 *   used later to detach and disable the pin interrupt.
 *
 ****************************************************************************/

static FAR void *tca9534_attach(FAR struct ioexpander_dev_s *dev,
                                ioe_pinset_t pinset, ioe_callback_t callback,
                                FAR void *arg)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  FAR void *handle = NULL;
  int i;

  /* Get exclusive access to the TCA555 */

  tca9534_lock(tca);

  /* Find and available in entry in the callback table */

  for (i = 0; i < CONFIG_TCA9534_INT_NCALLBACKS; i++)
    {
       /* Is this entry available (i.e., no callback attached) */

       if (tca->cb[i].cbfunc == NULL)
         {
           /* Yes.. use this entry */

           tca->cb[i].pinset = pinset;
           tca->cb[i].cbfunc = callback;
           tca->cb[i].cbarg  = arg;
           handle            = &tca->cb[i];
           break;
         }
    }

  /* Add this callback to the table */

  tca9534_unlock(tca);
  return handle;
}

/****************************************************************************
 * Name: tca9534_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by tca9534_attch()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int tca9534_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)dev;
  FAR struct tca9534_callback_s *cb = (FAR struct tca9534_callback_s *)handle;

  DEBUGASSERT(tca != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&tca->cb[0] &&
              (uintptr_t)cb <= (uintptr_t)&tca->cb[CONFIG_TCA64XX_INT_NCALLBACKS-1]);
  UNUSED(tca);

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}

/****************************************************************************
 * Name: tca9534_irqworker
 *
 * Description:
 *   Handle GPIO interrupt events (this function actually executes in the
 *   context of the worker thread).
 *
 ****************************************************************************/

static void tca9534_irqworker(void *arg)
{
  FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s *)arg;
  uint8_t addr = TCA9534_REG_INPUT;
  uint8_t buf;
  ioe_pinset_t pinset;
  int ret;
  int i;

  /* Read inputs */

  ret = tca9534_writeread(tca, &addr, 1, &buf, 1);
  if (ret == OK)
    {
#ifdef CONFIG_TCA9534_SHADOW_MODE
      /* Don't forget to update the shadow registers at this point */

      tca->sreg[addr]   = buf;
#endif
      /* Create a 8-bit pinset */

      pinset = ((unsigned char) buf;

      /* Perform pin interrupt callbacks */

      for (i = 0; i < CONFIG_TCA9534_INT_NCALLBACKS; i++)
        {
          /* Is this entry valid (i.e., callback attached)?  If so, did
           * any of the requested pin interrupts occur?
           */

          if (tca->cb[i].cbfunc != NULL)
            {
              /* Did any of the requested pin interrupts occur? */

              ioe_pinset_t match = pinset & tca->cb[i].pinset;
              if (match != 0)
                {
                  /* Yes.. perform the callback */

                  (void)tca->cb[i].cbfunc(&tca->dev, match,
                                          tca->cb[i].cbarg);
                }
            }
        }
    }

  /* Re-enable interrupts */

  tca->config->enable(tca->config, TRUE);
}

/****************************************************************************
 * Name: tca9534_interrupt
 *
 * Description:
 *   Handle GPIO interrupt events (this function executes in the
 *   context of the interrupt).
 *
 ****************************************************************************/

static int tca9534_interrupt(int irq, FAR void *context, FAR void *arg)
{
  register FAR struct tca9534_dev_s *tca = (FAR struct tca9534_dev_s*)arg;

  /* In complex environments, we cannot do I2C transfers from the interrupt
   * handler because semaphores are probably used to lock the I2C bus.  In
   * this case, we will defer processing to the worker thread.  This is also
   * much kinder in the use of system resources and is, therefore, probably
   * a good thing to do in any event.
   */

  /* Notice that further GPIO interrupts are disabled until the work is
   * actually performed.  This is to prevent overrun of the worker thread.
   * Interrupts are re-enabled in tca9534_irqworker() when the work is
   * completed.
   */

  if (work_available(&tca->work))
    {
      tca->config->enable(tca->config, FALSE);
      work_queue(HPWORK, &tca->work, tca9534_irqworker,
                 (FAR void *)tca, 0);
    }

  return OK;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca9534_initialize
 *
 * Description:
 *   Initialize a TCA9534 I2C device.
 *
 * TODO: Actually support more than one device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *tca9534_initialize(FAR struct i2c_master_s *i2cdev,
                                                FAR struct tca9534_config_s *config)
{
  FAR struct tca9534_dev_s *tcadev;

  DEBUGASSERT(i2cdev != NULL && config != NULL);

#ifdef CONFIG_TCA9534_MULTIPLE
  /* Allocate the device state structure */

  tcadev = (FAR struct tca9534_dev_s *)kmm_zalloc(sizeof(struct tca9534_dev_s));
  if (!tcadev)
    {
      return NULL;
    }

  /* And save the device structure in the list of TCA9534 so that we can
   * find it later.
   */

  tcadev->flink = g_tca9534list;
  g_tca9534list = tcadev;

#else
  /* Use the one-and-only TCA9534 driver instance */

  tcadev = &g_tca9534;
#endif

  /* Initialize the device state structure */

  tcadev->i2c     = i2cdev;
  tcadev->dev.ops = &g_tca9534_ops;
  tcadev->config  = config;

#ifdef CONFIG_TCA9534_INT_ENABLE
  tcadev->config->attach(tcadev->config, tca9534_interrupt, tcadev);
  tcadev->config->enable(tcadev->config, TRUE);
#endif

  nxsem_init(&tcadev->exclsem, 0, 1);
  return &tcadev->dev;
}

#endif /* CONFIG_IOEXPANDER_TCA9534 */
