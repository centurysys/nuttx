/****************************************************************************
 * boards/arm/sama5/mas1xx/src/mas1xx_param.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "mas1xx_param.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool initialized = false;
static struct hw_parameter params;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_powerkey_worker
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mas1xx_init_parameter
 ****************************************************************************/

bool mas1xx_init_parameter(void)
{
  int fd;
  ssize_t nread;
  const int len = sizeof(struct hw_parameter);

  fd = open("/dev/eeprom", O_RDONLY);
  if (fd < 0)
    {
      _err("unable to open /dev/eeprom device.\n");
      return false;
    }

  nread = read(fd, &params, len);
  close(fd);

  if (nread < len)
    {
      _err("read() MACADDR failed.\n");
    }
  else
    {
      initialized = true;
    }

  return initialized;
}

/****************************************************************************
 * Name: mas1xx_param_get_macaddr
 ****************************************************************************/

int mas1xx_param_get_macaddr(uint8_t *mac, int size)
{
  int ret = ERROR;

  if (!initialized)
    {
      bool ok = mas1xx_init_parameter();

      if (!ok)
        {
          return ret;
        }
    }

  if (size >= 6)
    {
      memcpy(mac, params.macaddr0, 6);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: mas1xx_param_get_xio_id
 ****************************************************************************/

int mas1xx_param_get_xio_id(uint8_t *xio_id)
{
  if (!initialized)
    {
      bool ok = mas1xx_init_parameter();

      if (!ok)
        {
          return ERROR;
        }
    }

  *xio_id = params.xio_id;
  return OK;
}
