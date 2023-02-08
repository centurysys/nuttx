/****************************************************************************
 * boards/arm/sama5/mas1xx/src/sam_zoneinfo.c
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
#include <errno.h>

#include <nuttx/drivers/ramdisk.h>
#include <nuttx/zoneinfo.h>

#ifdef CONFIG_LIBC_ZONEINFO_ROMFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LIBC_TZDIR
#  error CONFIG_LIBC_TZDIR is not defined
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "Mountpoint support is disabled"
#endif

#ifndef CONFIG_FS_ROMFS
#  error "ROMFS support not enabled"
#endif

#define SECTORSIZE  64
#define NSECTORS(b) (((b)+SECTORSIZE-1)/SECTORSIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sam_zoneinfo(int minor)
{
   char devname[32];
   int  ret;

  /* Create a RAM disk for the test */

  ret = romdisk_register(minor, romfs_zoneinfo_img,
                         NSECTORS(romfs_zoneinfo_img_len), SECTORSIZE);
  if (ret < 0)
    {
      printf("ERROR: Failed to create RAM disk\n");
      return ret;
    }

  /* Use the minor number to create a name for the ROM disk block device */

  snprintf(devname, 32, "/dev/ram%d", minor);

  /* Mount the ROMFS file system */

  printf("Mounting ROMFS filesystem at target=%s with source=%s\n",
         CONFIG_LIBC_TZDIR, devname);

  ret = mount(devname, CONFIG_LIBC_TZDIR, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      printf("ERROR: Mount failed: %d\n", errno);
      return ret;
    }

  printf("TZ database mounted at %s\n", CONFIG_LIBC_TZDIR);
  return OK;
}
#endif /* CONFIG_LIB_ZONEINFO_ROMFS */
