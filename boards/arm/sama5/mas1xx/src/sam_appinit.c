/****************************************************************************
 * boards/arm/sama5/mas1xx/src/sam_appinit.c
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
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/boardctl.h>

#include "arm_internal.h"
#include "hardware/sam_shdwc.h"
#include "mas1xx.h"

#ifndef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
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
#ifndef CONFIG_BOARD_LATE_INITIALIZE
  /* Perform board initialization */

  mcinfo("Entry\n");
  return sam_bringup();
#else
  return OK;
#endif
}

#endif /* CONFIG_BUILD_KERNEL */

#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  int ret = -ENOTTY;

  switch (cmd)
    {
      case BIOC_ENABLE_WAKEUP:
        {
          uint32_t val;

          val= getreg32(SAM_SHDWC_WUIR);
          val |= ((uint32_t)arg) & SHDWC_WUIR_WKUPEN_MASK;
          putreg32(val, SAM_SHDWC_WUIR);

          ret = OK;
        }
        break;

      case BIOC_DISABLE_WAKEUP:
        {
          uint32_t val;

          val = getreg32(SAM_SHDWC_WUIR);
          val ^= ((uint32_t)arg) & SHDWC_WUIR_WKUPEN_MASK;
          putreg32(val, SAM_SHDWC_WUIR);

          ret = OK;
        }
        break;

      case BIOC_GET_WAKEUP:
        {
          uint32_t val;

          if (!arg)
            {
              ret = -EFAULT;
              break;
            }

          val = getreg32(SAM_SHDWC_WUIR) & SHDWC_WUIR_WKUPEN_MASK;
          *(uint32_t *)arg = val;

          ret = OK;
        }
        break;

      case BIOC_SHUTDOWN:
        {
          putreg32(SHDWC_CR_SHDW + SHDWC_CR_KEY, SAM_SHDWC_CR);

          /* not reached. */
          ret = OK;
        }
        break;

      default:
        break;
    }

  return ret;
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == NULL)
    {
      return -EINVAL;
    }

  uniqueid = 1;
  return OK;
}
#endif
