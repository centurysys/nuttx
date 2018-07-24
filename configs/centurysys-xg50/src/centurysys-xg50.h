/************************************************************************************
 * configs/centurysys-xg50/src/centurysys-xg50.h
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Frank Bennett
 *            Gregory Nutt <gnutt@nuttx.org>
 *            Sebastien Lorquet <sebastien@lorquet.fr>
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
 ************************************************************************************/

#ifndef __CONFIGS_CENTURYSYS_XG50_SRC_CENTURYSYS_XG50_H
#define __CONFIGS_CENTURYSYS_XG50_SRC_CENTURYSYS_XG50_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_PROC             1
#define HAVE_RTC_DRIVER       1

#if !defined(CONFIG_FS_PROCFS)
#  undef HAVE_PROC
#endif

#if defined(HAVE_PROC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No procfs support
#  undef HAVE_PROC
#endif

/* Check if we can support the RTC driver */

#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* LED */

#define GPIO_LED_R                                                      \
  (GPIO_PORTD | GPIO_PIN2  | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_OPENDRAIN | \
   GPIO_SPEED_50MHz)

#define GPIO_LED_G                                                      \
  (GPIO_PORTC | GPIO_PIN12 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_OPENDRAIN | \
   GPIO_SPEED_50MHz)

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */

#define XG50_I2C_OBDEV_LED       0x55

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32l4_usbinitialize(void);

/************************************************************************************
 * Name: stm32l4_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int stm32l4_adc_setup(void);
#endif

/****************************************************************************
 * Name: board_timer_driver_initialize
 *
 * Description:
 *   Initialize and register a timer
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int board_timer_driver_initialize(FAR const char *devpath, int timer);
#endif

#endif /* __CONFIGS_CENTURYSYS_XG50_SRC_CENTURYSYS_XG50_H */
