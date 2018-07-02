/****************************************************************************
 * configs/centurysys-xg50/src/stm32l4_appinit.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>
#include <stm32l4_uid.h>

#include <arch/board/board.h>
#include <arch/board/boardctl.h>

#include "centurysys-xg50.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif

#undef HAVE_I2C_DRIVER
#if defined(CONFIG_STM32L4_I2C1) && defined(CONFIG_I2C_DRIVER)
#  include "stm32l4_i2c.h"
#  include <nuttx/mtd/configdata.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/leds/tca6507.h>
#  define HAVE_I2C_DRIVER 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *rtclower;
#endif
#ifdef HAVE_I2C_DRIVER
  FAR struct i2c_master_s *i2c;
  struct mtd_dev_s *at24;
#endif
  int ret;

  (void)ret;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_I2C_DRIVER
  /* Get the I2C lower half instance */

  i2c = stm32l4_i2cbus_initialize(1);

  if (i2c == NULL)
    {
      i2cerr("ERROR: Initialize I2C1: %d\n", ret);
    }
  else
    {
#  ifdef CONFIG_I2C_DRIVER
      /* Register the I2C character driver */

      ret = i2c_register(i2c, 1);

      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register I2C1 device: %d\n", ret);
        }
    }
#  endif

  /* Initialize the AT24 driver */

  at24 = at24c_initialize(i2c);

  if (!at24)
    {
      i2cerr("ERROR: Failed to initialize the AT24 driver\n");
    }
  else
    {
      ret = ftl_initialize(0, at24);

      if (ret < 0)
        {
          i2cerr("ERROR: Failed to initialize the FTL layer: %d\n", ret);
        }
    }

#  ifdef CONFIG_TCA6507

  ret = tca6507_register("/dev/leddrv0", i2c, 0x45);

  if (ret < 0)
    {
      i2cerr("ERROR: Failed to initialize TCA6507 driver\n");
    }
#  endif
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32L4 lower-half RTC driver */

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32l4_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

  ret = board_timer_driver_initialize("/dev/timer0", 2);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_BOARDCTL_IOCTL
  board_ioctl(BIOC_CONFIG_GPIO, 0);
  board_ioctl(BIOC_ENABLE_B2B, 0);
#endif

  UNUSED(ret);
  return OK;
}

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  int res = OK;

  switch (cmd)
    {
    case BIOC_CONFIG_GPIO:
      /* configure GPIO pins */
      stm32l4_configgpio(GPIO_B2B_RESET);
      stm32l4_configgpio(GPIO_B2B_DCDC);
      stm32l4_configgpio(GPIO_B2B_POWER);
      stm32l4_configgpio(GPIO_B2B_WAKEUP);
      stm32l4_configgpio(GPIO_B2B_GPIO);
      stm32l4_configgpio(GPIO_B2B_RI);
      stm32l4_configgpio(GPIO_MODESW);
      stm32l4_configgpio(GPIO_LEDSW);
      stm32l4_configgpio(GPIO_INITSW);
      stm32l4_configgpio(GPIO_RX485_RXE);
      stm32l4_configgpio(GPIO_RX485_TXE);
      stm32l4_configgpio(GPIO_DCDC_SEL);
      break;

    case BIOC_ENABLE_B2B:
      syslog(LOG_INFO, "%s: BIOC_ENABLE_B2B\n", __FUNCTION__);

      stm32l4_gpiowrite(GPIO_B2B_RESET, 1);
      stm32l4_gpiowrite(GPIO_B2B_DCDC, 0);
      stm32l4_gpiowrite(GPIO_B2B_POWER, 1);

      break;

    case BIOC_DISABLE_B2B:
      stm32l4_gpiowrite(GPIO_B2B_RESET, 0);
      stm32l4_gpiowrite(GPIO_B2B_POWER, 0);
      stm32l4_gpiowrite(GPIO_B2B_DCDC, 1);
      break;

    case BIOC_RESET_B2B:
      {
        uint32_t wait_msec = *((uint32_t *) arg);

        if (wait_msec == 0)
          {
            wait_msec = 100;
          }
        else if (wait_msec > 1000)
          {
            wait_msec = 1000;
          }

        stm32l4_gpiowrite(GPIO_B2B_RESET, 0);
        usleep(wait_msec * 1000);
        stm32l4_gpiowrite(GPIO_B2B_RESET, 1);

        break;
      }

    case BIOC_GET_LEDSW:
      {
        int *stat;

        if (arg)
          {
            stat = (int *) arg;

            stm32l4_configgpio(GPIO_LEDSW);
            *stat = stm32l4_gpioread(GPIO_LEDSW);
          }
        else
          {
            res = -EFAULT;
          }
        break;
      }

    case BIOC_SET_LED:
      {
        bool on = (bool) arg;

        board_userled(BOARD_LED_G, on);

        break;
      }

    case BIOC_GET_DI:
      break;

    default:
      res = -ENOTTY;
      break;
    }

  return res;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32l4_get_uniqueid(uniqueid);
  return OK;
}
#endif
