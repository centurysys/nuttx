/****************************************************************************
 * boards/arm/sama5/mas1xx/src/sam_bringup.c
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

#include <sys/mount.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <debug.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "sam_twi.h"
#include "sam_pio.h"
#include "mas1xx.h"
#include "mas1xx_lte.h"
#include "mas1xx_param.h"
#include "mas1xx_xio.h"

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_NET_CDCECM
#  include <nuttx/usb/cdcecm.h>
#  include <net/if.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_MMCSD
#  include <nuttx/mmcsd.h>
#  include "sam_sdmmc.h"
#endif

#ifdef CONFIG_RTC_DSK324SR
#  include <nuttx/timers/rtc.h>
#  include <nuttx/timers/dsk324sr.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NSECTORS(n) \
  (((n)+CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE-1) / \
   CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void sam_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = sam_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      _err("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          _err("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          sam_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: sam_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#ifdef HAVE_I2CTOOL
static void sam_i2ctool(void)
{
#ifdef CONFIG_SAMA5_TWI0
  sam_i2c_register(0);
#endif
#ifdef CONFIG_SAMA5_TWI1
  sam_i2c_register(1);
#endif
}
#else
#  define sam_i2ctool()
#endif

#ifdef CONFIG_RTC_DSK324SR
/****************************************************************************
 * Name: rtc_rtc2sys
 ****************************************************************************/

static void rtc_rtc2sys(void)
{
  int fd;
  struct rtc_time rtctime;
  struct tm tm;
  struct timespec ts;

  fd = open("/dev/rtc0", O_RDONLY);
  ioctl(fd, RTC_RD_TIME, &rtctime);
  close(fd);

  tm.tm_year = rtctime.tm_year;
  tm.tm_mon = rtctime.tm_mon;
  tm.tm_mday = rtctime.tm_mday;
  tm.tm_hour = rtctime.tm_hour;
  tm.tm_min = rtctime.tm_min;
  tm.tm_sec = rtctime.tm_sec;

  ts.tv_sec = timegm(&tm);
  ts.tv_sec += 9 * 60 * 60;
  ts.tv_nsec = 0;

  clock_settime(CLOCK_REALTIME, &ts);
}

/****************************************************************************
 * Name: nsh_rtc_initialize
 *
 * Description:
 *   Initialize RTC driver
 *
 ****************************************************************************/

static int nsh_rtc_initialize(void)
{
  struct i2c_master_s *i2c;
  struct rtc_lowerhalf_s *dsk324sr;
  int ret;

  i2c = sam_i2cbus_initialize(1);
  if (!i2c)
    {
      _err("%s: failed to get I2c1 interface\n", __FUNCTION__);
      return -ENODEV;
    }

  dsk324sr = dsk324sr_rtc_lowerhalf(i2c, 0x32);
  if (!dsk324sr)
    {
      _err("%s: failed to get dsk324sr lowerhalf\n", __FUNCTION__);
      return -ENODEV;
    }

  ret = rtc_initialize(0, dsk324sr);

  if (ret == OK)
    {
      rtc_rtc2sys();
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: nsh_sdmmc_initialize
 *
 * Description:
 *   Initialize SDMMC drivers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SDMMC

static int nsh_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdmmc0;
#ifdef CONFIG_SAMA5_SDMMC1
  struct sdio_dev_s *sdmmc1;
#endif
  int ret = 0;

  /* Get an instance of the SDIO interface */

#ifdef CONFIG_SAMA5_SDMMC0
  sdmmc0 = sam_sdmmc_sdio_initialize(SDMMC0_SLOTNO);
  if (!sdmmc0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(SDMMC0_MINOR, sdmmc0);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver (slot 0): "
                 "%d\n",
                 ret);
        }
    }

#ifdef CONFIG_SAMA5D27_SDMMC0_MOUNT
  /* Mount the volume on SDMMC0 */

  ret = nx_mount(CONFIG_SAMA5D27_SDMMC0_MOUNT_BLKDEV,
                 CONFIG_SAMA5D27_SDMMC0_MOUNT_MOUNTPOINT,
                 CONFIG_SAMA5D27_SDMMC0_MOUNT_FSTYPE,
                 0, NULL);

  if (ret < 0)
    {
      _err("ERROR: Failed to mount %s: %d\n",
           CONFIG_SAMA5D27_SDMMC0_MOUNT_MOUNTPOINT, ret);
    }
#endif
#endif

#ifdef CONFIG_SAMA5_SDMMC1
  sdmmc1 = sam_sdmmc_sdio_initialize(SDMMC1_SLOTNO);
  if (!sdmmc1)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(SDMMC1_MINOR, sdmmc1);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver (slot 0): "
                 "%d\n",
                 ret);
        }
    }

#ifdef CONFIG_SAMA5D27_SDMMC1_MOUNT
  /* Mount the volume on SDMMC1 */

  ret = nx_mount(CONFIG_SAMA5D27_SDMMC1_MOUNT_BLKDEV,
                 CONFIG_SAMA5D27_SDMMC1_MOUNT_MOUNTPOINT,
                 CONFIG_SAMA5D27_SDMMC1_MOUNT_FSTYPE,
                 0, NULL);

  if (ret < 0)
    {
      _err("ERROR: Failed to mount %s: %d\n",
           CONFIG_SAMA5D27_SDMMC1_MOUNT_MOUNTPOINT, ret);
    }
#endif
#endif

  return OK;
}
#else
#  define nsh_sdmmc_initialize() (OK)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void)
{
  int ret;

  /* Register I2C drivers on behalf of the I2C tool */

  sam_i2ctool();

#ifdef CONFIG_SAMA5_TWI1
  board_i2c_initialize();
  mas1xx_init_parameter();
#endif

#ifdef HAVE_USBHOST
  lte_pio_initialize();
#endif

#ifdef HAVE_MACADDR
  /* Read the Ethernet MAC address from the AT24 EEPROM and configure the
   * Ethernet driver with that address.
   */

  ret = sam_emac0_setmac();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_emac0_setmac() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LIBC_ZONEINFO_ROMFS
  sam_zoneinfo(0);
#endif

#ifdef CONFIG_RTC_DSK324SR
  nsh_rtc_initialize();
#endif

#ifdef HAVE_SDMMC
#ifdef CONFIG_SAMA5_SDMMC
  /* Initialize SDMCC-based MMC/SD card support */

  nsh_sdmmc_initialize();
#endif
#endif

#ifdef HAVE_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif

#ifdef HAVE_ROMFS
  /* Create a ROM disk for the /etc filesystem */

  ret = romdisk_register(CONFIG_SAMA5D4EK_ROMFS_ROMDISK_MINOR, romfs_img,
                         NSECTORS(romfs_img_len),
                         CONFIG_SAMA5D4EK_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      _err("ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = nx_mount(CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
                     CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT,
                     "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          _err("ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
               CONFIG_SAMA5D4EK_ROMFS_ROMDISK_DEVNAME,
               CONFIG_SAMA5D4EK_ROMFS_MOUNT_MOUNTPOINT, ret);
        }
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  sam_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = sam_usbhost_initialize();
  if (ret != OK)
    {
        _err("ERROR: Failed to initialize USB host: %d\n", ret);
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
        _err("ERROR: Failed to start the USB monitor: %d\n", ret);
    }
#endif

#ifdef HAVE_USBHOST
  /* Power-ON LTE module */

  lte_power_ctrl(true);
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = sam_adc_setup();
  if (ret < 0)
    {
      _err("ERROR: sam_adc_setup failed: %d\n", ret);
    }
#endif

  ret = mas1xx_xio_initialize();
  if (ret < 0)
    {
      _err("ERROR: failed to initialize XIO: %d\n", ret);
    }

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, SAMA5_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("ERROR: Failed to mount procfs at %s: %d\n",
           SAMA5_PROCFS_MOUNTPOINT, ret);
    }
#endif

  if (sam_netinitialize)
    {
      sam_netinitialize();
    }

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
