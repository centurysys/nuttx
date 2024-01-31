/****************************************************************************
 * boards/arm/sama5/mas1xx/src/mas1xx_lte.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sam_pio.h"
#include "mas1xx_lte.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool initialized = false;
static struct work_s lte_power_work, lte_reset_work;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lte_powerkey_worker
 ****************************************************************************/

static void lte_powerkey_worker(void *arg)
{
  bool current, onoff;
  useconds_t wait;

  current = get_lte_status();
  onoff = (bool)arg;

  _info("Control: %s\n", onoff ? "ON" : "OFF");

  if (current == onoff)
    {
      return;
    }

  wait = onoff == true ? 550 : 700;

  sam_piowrite(PIO_LTE_PWRKEY, true);
  nxsig_usleep(wait * 1000);
  sam_piowrite(PIO_LTE_PWRKEY, false);
}

/****************************************************************************
 * Name: lte_reset_worker
 ****************************************************************************/

static void lte_reset_worker(void *arg)
{
  useconds_t wait;

  wait = 500;

  _info("Reset LTE module\n");

  sam_piowrite(PIO_LTE_RESET, true);
  nxsig_usleep(wait * 1000);
  sam_piowrite(PIO_LTE_RESET, false);
}

/****************************************************************************
 * Name: pio_initialize
 ****************************************************************************/

void lte_pio_initialize(void)
{
  if (initialized)
    {
      return;
    }

  sam_configpio(PIO_LTE_PWRKEY);
  sam_configpio(PIO_LTE_RESET);
  sam_configpio(PIO_LTE_READY);

  _info("LTE GPIOs initialized.\n");
  initialized = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_lte_status
 ****************************************************************************/

bool get_lte_status(void)
{
  bool on;

  lte_pio_initialize();
  on = sam_pioread(PIO_LTE_READY) == 1 ? false : true;

  return on;
}

/****************************************************************************
 * Name: get_lte_status
 ****************************************************************************/

bool lte_power_ctrl(bool on)
{
  bool current = get_lte_status();
  int ret;
  bool arg = on;

  if (current == on)
    {
      return true;
    }

  ret = work_queue(LPWORK, &lte_power_work, lte_powerkey_worker,
                   (void *)arg, 0);
  return ret == 0;
}

/****************************************************************************
 * Name: get_lte_status
 ****************************************************************************/

bool lte_reset(void)
{
  int ret;

  ret = work_queue(LPWORK, &lte_reset_work, lte_reset_worker, NULL, 0);
  return ret == 0;
}
