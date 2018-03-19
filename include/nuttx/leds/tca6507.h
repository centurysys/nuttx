/****************************************************************************
 * include/nuttx/leds/tca6507.h
 * based on include/nuttx/leds/ncp5623c.h
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

#ifndef __INCLUDE_NUTTX_LEDS_TCA6507_H
#define __INCLUDE_NUTTX_LEDS_TCA6507_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_TCA6507 - Enables support for the TCA6507 driver
 */

#if defined(CONFIG_I2C) && defined(CONFIG_TCA6507)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C definitions */

#define I2C_BUS_FREQ_HZ        (400000)

/* TCA6507 register addresses */

#define TCA6507_SELECT0        (0x00)     /* Select0                   */
#define TCA6507_SELECT1        (0x01)     /* Select1                   */
#define TCA6507_SELECT2        (0x02)     /* Select2                   */
#define TCA6507_SELECT(x)      (x)
#define TCA6507_FADE_ON_TIME   (0x02)     /* Fade-ON Time              */
#define TCA6507_FULLY_ON_TIME  (0x02)     /* Fully-ON Time             */
#define TCA6507_FADE_OFF_TIME  (0x02)     /* Fade-OFF Time             */
#define TCA6507_1ST_FULLY_ON   (0x02)     /* First Fully-OFF Time      */
#define TCA6507_2ND_FULLY_OFF  (0x02)     /* Second Fully-OFF Time     */
#define TCA6507_MAX_INTENSITY  (0x02)     /* Maximum Intensity         */
#define TCA6507_ONESHOT        (0x02)     /* One Shot/MAster Intensity */
#define TCA6507_INITIALIZATION (0x02)     /* Initialization            */

#define TCA6507_MAX_LED        6

#define LED_STATUS1_RED   0
#define LED_STATUS2_RED   1
#define LED_STATUS3_RED   2
#define LED_STATUS1_GREEN 3
#define LED_STATUS2_GREEN 4
#define LED_STATUS3_GREEN 5

/* IOCTL commands */

#define LEDIOC_ONOFF  _ULEDIOC(1)        /* Arg: tca6507_set_reg_s * pointer */
#define LEDIOC_ALLOFF _ULEDIOC(2)        /* Arg: NULL */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct tca6507_onoff_s
{
  uint8_t led;
  bool on;
};

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: tca6507_register
 *
 * Description:
 *   Register the TCA6507 device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   i2c     - An instance of the I2C interface to use to communicate
 *             with the LM92.
 *   tca6507_i2c_addr
 *           - The I2C address of the TCA6507.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tca6507_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t const tca6507_i2c_addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_TCA6507 */
#endif /* __INCLUDE_NUTTX_LEDS_TCA6507_H */
