/****************************************************************************
 * drivers/timers/dsk324sr.c
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

#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/timers/dsk324sr.h>

#include "dsk324sr.h"

#ifdef CONFIG_RTC_DSK324SR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSK324SR_OSCRUN_READ_RETRY  5  /* How many time to read OSCRUN status */

/* Configuration ************************************************************/

/* This RTC implementation supports only date/time RTC hardware */

#ifndef CONFIG_RTC_DATETIME
#  error CONFIG_RTC_DATETIME must be set to use this driver
#endif

#ifdef CONFIG_RTC_HIRES
#  error CONFIG_RTC_HIRES must NOT be set with this driver
#endif

#ifndef CONFIG_DSK324SR_I2C_FREQUENCY
#  define CONFIG_DSK324SR_I2C_FREQUENCY 100000
#endif

#if CONFIG_DSK324SR_I2C_FREQUENCY > 400000
#  error CONFIG_DSK324SR_I2C_FREQUENCY is out of range
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum {
  UNKNOWN = 0,
  DSK324SR,
  DD3225TS,
} rtc_type_t;

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct dsk324sr_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;      /* Threads can only exclusively access the RTC */

  struct i2c_master_s *i2c;

  uint8_t addr;
  rtc_type_t type;

  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int dsk324sr_rtc_rdtime(struct rtc_lowerhalf_s *lower,
                               struct rtc_time *rtctime);
static int dsk324sr_rtc_settime(struct rtc_lowerhalf_s *lower,
                                const struct rtc_time *rtctime);
static int dsk324sr_rtc_alarm_read(struct rtc_lowerhalf_s *lower,
                                   struct rtc_wkalrm *alarm);
static int dsk324sr_rtc_alarm_set(struct rtc_lowerhalf_s *lower,
                                  const struct rtc_wkalrm *alarm);
static int dsk324sr_rtc_ioctl(struct rtc_lowerhalf_s *lower, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DSK324SR RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = dsk324sr_rtc_rdtime,
  .settime     = dsk324sr_rtc_settime,
  .ioctl       = dsk324sr_rtc_ioctl,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = dsk324sr_rtc_setalarm,
  .setrelative = dsk324sr_rtc_setrelative,
  .cancelalarm = dsk324sr_rtc_cancelalarm,
  .rdalarm     = dsk324sr_rtc_rdalarm,
#endif
};

/* DSK324SR RTC device state */

static struct dsk324sr_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *   Show the broken out time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("   tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("   tm_min: %08x\n", tp->tm_min);
  rtcinfo("  tm_hour: %08x\n", tp->tm_hour);
  rtcinfo("  tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("   tm_mon: %08x\n", tp->tm_mon);
  rtcinfo("  tm_year: %08x\n", tp->tm_year);
  rtcinfo("  tm_wday: %08x\n", tp->tm_wday);
  rtcinfo("  tm_yday: %08x\n", tp->tm_yday);
  rtcinfo(" tm_isdst: %08x\n", tp->tm_isdst);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/****************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Converts a 2 digit binary to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ****************************************************************************/

static uint8_t rtc_bin2bcd(int value)
{
  uint8_t msbcd = 0;

  while (value >= 10)
    {
      msbcd++;
      value -= 10;
    }

  return (msbcd << 4) | value;
}

/****************************************************************************
 * Name: rtc_bcd2bin
 *
 * Description:
 *   Convert from 2 digit BCD to binary.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ****************************************************************************/

static int rtc_bcd2bin(uint8_t value)
{
  int tens = ((int)value >> 4) * 10;
  return tens + (value & 0x0f);
}

/****************************************************************************
 * Name: read_block_data
 ****************************************************************************/

static int read_block_data(const struct dsk324sr_lowerhalf_s *lower, uint8_t regaddr,
                           uint8_t length, uint8_t *values)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[0].addr      = lower->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[1].addr      = lower->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = values;
  msg[1].length    = length;

  return I2C_TRANSFER(lower->i2c, msg, 2);
}

/****************************************************************************
 * Name: write_block_data
 ****************************************************************************/

static int write_block_data(const struct dsk324sr_lowerhalf_s *lower, uint8_t regaddr,
                            uint8_t length, uint8_t *values)
{
  struct i2c_msg_s msg[1];
  uint8_t buffer[256];

  if (length > 254)
    {
      return -1;
    }

  buffer[0] = regaddr;
  memcpy(&buffer[1], values, length);

  msg[0].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[0].addr      = lower->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = buffer;
  msg[0].length    = length + 1;

  return I2C_TRANSFER(lower->i2c, msg, 1);
}

/****************************************************************************
 * Name: read_byte_data
 ****************************************************************************/

static int read_byte_data(const struct dsk324sr_lowerhalf_s *lower, uint8_t addr,
                          uint8_t *value)
{
  return read_block_data(lower, addr, 1, value);
}

/****************************************************************************
 * Name: write_byte_data
 ****************************************************************************/

static int write_byte_data(const struct dsk324sr_lowerhalf_s *lower, uint8_t addr,
                           uint8_t value)
{
  return write_block_data(lower, addr, 1, &value);
}

/****************************************************************************
 * Name: get_rtc_type
 ****************************************************************************/

static rtc_type_t get_rtc_type(const struct dsk324sr_lowerhalf_s *lower)
{
  uint8_t buf[32];
  char *rtcname;
  int err, retry;
  rtc_type_t type = UNKNOWN;

  for (retry = 10; retry >= 0; retry--)
    {
      err = read_block_data(lower, DSK324SR_REG_RTCSEC, 30, buf);
      if (err < 0)
        {
          continue;
        }
      else
        {
          if (memcmp(&buf[0], &buf[15], 7) == 0)
            {
              rtcname = "DD3225TS";
              type = DD3225TS;
            }
          else if (memcmp(&buf[0], &buf[14], 7) == 0)
            {
              rtcname = "DS324SR";
              type = DSK324SR;
            }
          else
            {
              continue;
            }

          _info("RTC %s detected.\n", rtcname);
          break;
        }
    }

  if (type != UNKNOWN)
    {
      err = read_byte_data(lower, DSK324SR_REG_SELECT, buf);

      if (err >= 0)
        {
          buf[0] |= DSK324SR_SELECT_TCS_30S;

          err = write_byte_data(lower, DSK324SR_REG_SELECT, buf[0]);

          if (err >= 0)
            {
              _info("SEL Register updated to 30s.\n");
            }
          else
            {
              _err("SEL Register updated failed.\n");
            }
        }

      err = read_byte_data(lower, DSK324SR_REG_FLAG, buf);

      if (err >= 0)
        {
          if ((buf[0] & DSK324SR_FLAG_VDHF) != 0)
            {
              _info("high voltage detected, date/time is not reliable.\n");
            }

          if ((buf[0] & DSK324SR_FLAG_VDLF) != 0)
            {
              _info("low voltage detected, date/time is not reliable.\n");
            }

          buf[0] &= ~(DSK324SR_FLAG_VDF | DSK324SR_FLAG_TEST);

          err = write_byte_data(lower, DSK324SR_REG_FLAG, buf[0]);
        }

      if (type == DD3225TS)
        {
          buf[0] = 0;

          err = write_byte_data(lower, DD3225TS_REG_TEST, buf[0]);

          if (err < 0)
            {
              _err("Clear TEST register failed.\n");
            }
        }
    }

  return type;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dsk324sr_rtc_rdtime
 ****************************************************************************/

static int dsk324sr_rtc_rdtime(struct rtc_lowerhalf_s *lower,
                               struct rtc_time *rtctime)
{
  struct dsk324sr_lowerhalf_s *priv;
  struct i2c_msg_s msg[4];
  uint8_t secaddr;
  uint8_t buffer[7];
  uint8_t seconds;
  int ret;

  priv = (struct dsk324sr_lowerhalf_s *)lower;

  /* The start address of the read is the seconds address (0x00)
   * The chip increments the address to read from after each read.
   */

  secaddr = DSK324SR_REG_RTCSEC;

  msg[0].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &secaddr;
  msg[0].length    = 1;

  /* Setup the read. Seven (7) registers will be read.
   * (Seconds, minutes, hours, wday, date, month and year)
   */

  msg[1].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = 7;

  /* Read the seconds register again */

  msg[2].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[2].addr      = priv->addr;
  msg[2].flags     = I2C_M_NOSTOP;
  msg[2].buffer    = &secaddr;
  msg[2].length    = 1;

  msg[3].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[3].addr      = priv->addr;
  msg[3].flags     = I2C_M_READ;
  msg[3].buffer    = &seconds;
  msg[3].length    = 1;

  /* Perform the transfer. The transfer may be performed repeatedly of the
   * seconds values decreases, meaning that was a rollover in the seconds.
   */

  do
    {
      ret = I2C_TRANSFER(priv->i2c, msg, 4);
      if (ret < 0)
        {
          rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
          return ret;
        }
    }
  while ((buffer[0] & DSK324SR_RTCSEC_BCDMASK) >
         (seconds & DSK324SR_RTCSEC_BCDMASK));

  /* Format the return time */

  /* Return seconds (0-59) */

  rtctime->tm_sec = rtc_bcd2bin(buffer[0] & DSK324SR_RTCSEC_BCDMASK);

  /* Return minutes (0-59) */

  rtctime->tm_min = rtc_bcd2bin(buffer[1] & DSK324SR_RTCMIN_BCDMASK);

  /* Return hour (0-23).  This assumes 24-hour time was set. */

  rtctime->tm_hour = rtc_bcd2bin(buffer[2] & DSK324SR_RTCHOUR_BCDMASK);

  /* Return the day of the week (0-6) */

  rtctime->tm_wday = (rtc_bcd2bin(buffer[3]) & DSK324SR_RTCWKDAY_BCDMASK) - 1;

  /* Return the day of the month (1-31) */

  rtctime->tm_mday = rtc_bcd2bin(buffer[4] & DSK324SR_RTCDATE_BCDMASK);

  /* Return the month (0-11) */

  rtctime->tm_mon = rtc_bcd2bin(buffer[5] & DSK324SR_RTCMTH_BCDMASK) - 1;

  /* Return the years since 1900 */

  rtctime->tm_year = rtc_bcd2bin(buffer[6] & DSK324SR_RTCYEAR_BCDMASK);

  /* The Year is stored in the RTC starting from 2001. We need to convert it
   * to POSIX format that expects the year starting from 1900.
   */

  rtctime->tm_year += 100;

  return OK;
}

/****************************************************************************
 * Name: dsk324sr_rtc_settime
 ****************************************************************************/

static int dsk324sr_rtc_settime(struct rtc_lowerhalf_s *lower,
                                const struct rtc_time *rtctime)
{
  struct dsk324sr_lowerhalf_s *priv;
  uint8_t buffer[8];
  uint8_t ctrl;
  int ret;

  priv = (struct dsk324sr_lowerhalf_s *)lower;

  /* Set RESET bit. */

  ret = read_byte_data(priv, DSK324SR_REG_CONTROL, &ctrl);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  ctrl |= DSK324SR_CONTROL_RESET;

  ret = write_byte_data(priv, DSK324SR_REG_CONTROL, ctrl);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  /* Construct the message */

  /* Write starting with the seconds register */

  buffer[0] = DSK324SR_REG_RTCSEC;

  /* Save seconds (0-59) converted to BCD. And keep ST cleared. */

  buffer[1] = rtc_bin2bcd(rtctime->tm_sec);

  /* Save minutes (0-59) converted to BCD */

  buffer[2] = rtc_bin2bcd(rtctime->tm_min);

  /* Save hour (0-23) with 24-hour time indication */

  buffer[3] = rtc_bin2bcd(rtctime->tm_hour);

  /* Save the day of the week (1-7) */

  buffer[4] = rtc_bin2bcd(rtctime->tm_wday + 1);

  /* Save the day of the month (1-31) */

  buffer[5] = rtc_bin2bcd(rtctime->tm_mday);

  /* Save the month (1-12) */

  buffer[6] = rtc_bin2bcd(rtctime->tm_mon + 1);

  /* Save the year (00-99) */

  /* First we need to convert "tm_year" to value starting from 2001.
   * The "tm_year" in POSIX is relative to 1900, so 2019 is 119,
   * so you just need to subtract 101: year = (1900 + value) - 2001
   */

  buffer[7] = rtc_bin2bcd(rtctime->tm_year - 101);

  ret = write_block_data(priv, DSK324SR_REG_RTCSEC, 7, buffer);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  /* Clear VDHF and VDLF. */

  ret = read_byte_data(priv, DSK324SR_REG_FLAG, buffer);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  buffer[0] &= ~(DSK324SR_FLAG_VDF | DSK324SR_FLAG_TEST);
  ret = write_byte_data(priv, DSK324SR_REG_FLAG, buffer[0]);

  return ret;
}

/****************************************************************************
 * Name: dsk324sr_rtc_alarm_read
 ****************************************************************************/

static int dsk324sr_rtc_alarm_read(struct rtc_lowerhalf_s *lower,
                                   struct rtc_wkalrm *alarm)
{
  struct dsk324sr_lowerhalf_s *priv;
  struct i2c_msg_s msg[2];
  uint8_t buffer[14];
  uint8_t addr;
  int ret;

  memset(alarm, 0, sizeof(struct rtc_wkalrm));

  priv = (struct dsk324sr_lowerhalf_s *)lower;

  /* The start address of the read is the seconds address (0x00)
   * The chip increments the address to read from after each read.
   */

  addr = DSK324SR_REG_RTCSEC;

  msg[0].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &addr;
  msg[0].length    = 1;

  /* Setup the read. Seven (7) registers will be read.
   * (Seconds, minutes, hours, wday, date, month and year)
   */

  msg[1].frequency = CONFIG_DSK324SR_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = 14;

  /* Perform the transfer. The transfer may be performed repeatedly of the
   * seconds values decreases, meaning that was a rollover in the seconds.
   */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      rtcerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return ret;
    }

  if (buffer[DSK324SR_REG_CONTROL] & DSK324SR_CONTROL_AIE)
    {
      alarm->enabled = 1;
    }

  if (buffer[DSK324SR_REG_FLAG] & DSK324SR_FLAG_AF)
    {
      alarm->pending = 1;
    }

  alarm->time.tm_year = rtc_bcd2bin(buffer[DSK324SR_REG_RTCYEAR] &
                                    DSK324SR_RTCYEAR_BCDMASK) + 100;
  alarm->time.tm_mon  = rtc_bcd2bin(buffer[DSK324SR_REG_RTCMTH]  &
                                    DSK324SR_RTCMTH_BCDMASK) - 1;
  alarm->time.tm_mday = rtc_bcd2bin(buffer[DSK324SR_REG_RTCWKDAYALM] &
                                    DSK324SR_RTCWKDAYALM_BCDMASK);
  alarm->time.tm_hour = rtc_bcd2bin(buffer[DSK324SR_REG_RTCHOURALM] &
                                    DSK324SR_RTCHOURALM_BCDMASK);
  alarm->time.tm_min  = rtc_bcd2bin(buffer[DSK324SR_REG_RTCMINALM] &
                                    DSK324SR_RTCMINALM_BCDMASK);

  if (alarm->time.tm_mday == 0)
    {
      alarm->time.tm_mday = rtc_bcd2bin(buffer[DSK324SR_REG_RTCDATE]  &
                                        DSK324SR_RTCDATE_BCDMASK);
    }

  if (alarm->time.tm_hour == 0)
    {
      alarm->time.tm_hour = rtc_bcd2bin(buffer[DSK324SR_REG_RTCHOUR]  &
                                        DSK324SR_RTCHOUR_BCDMASK);
    }

  if (alarm->time.tm_min == 0)
    {
      alarm->time.tm_min = rtc_bcd2bin(buffer[DSK324SR_REG_RTCMIN]  &
                                       DSK324SR_RTCMIN_BCDMASK);
    }

  alarm->time.tm_wday = clock_dayoftheweek(alarm->time.tm_mday, alarm->time.tm_mon + 1,
                                           alarm->time.tm_year + TM_YEAR_BASE);

  return OK;
}

/****************************************************************************
 * Name: dsk324sr_rtc_alarm_set
 ****************************************************************************/

static int _rtc_alarm_set(struct rtc_lowerhalf_s *lower,
                          const struct rtc_wkalrm *alarm)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: dsk324sr_rtc_alarm_set
 ****************************************************************************/

static int dsk324sr_rtc_alarm_set(struct rtc_lowerhalf_s *lower,
                                  const struct rtc_wkalrm *alarm)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: dsk324sr_rtc_ioctl
 ****************************************************************************/

static int dsk324sr_rtc_ioctl(struct rtc_lowerhalf_s *lower, int cmd,
                              unsigned long arg)
{
  int ret = -ENOSYS;

  switch (cmd)
    {
    case RTC_ALM_READ:
      {
        struct rtc_wkalrm *alarm;

        if (arg == 0)
          {
            ret = -EFAULT;
            break;
          }

        alarm = (struct rtc_wkalrm *)((uintptr_t)arg);
        ret = dsk324sr_rtc_alarm_read(lower, alarm);
      }
      break;

    case RTC_ALM_SET:
      {
        const struct rtc_wkalrm *alarm;

        if (arg == 0)
          {
            ret = -EFAULT;
            break;
          }

        alarm = (const struct rtc_wkalrm *)((uintptr_t)arg);
        ret = dsk324sr_rtc_alarm_set(lower, alarm);
      }
      break;

    default:
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dsk324sr_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the DSK324SR.
 *
 * Input Parameters:
 *   i2c master
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *dsk324sr_rtc_lowerhalf(struct i2c_master_s *i2c,
                                               uint8_t addr)
{
  rtc_type_t type;

  if (g_rtc_lowerhalf.initialized || !i2c)
    {
      return NULL;
    }

  g_rtc_lowerhalf.i2c = i2c;
  g_rtc_lowerhalf.addr = addr;

  type = get_rtc_type(&g_rtc_lowerhalf);

  if (type == UNKNOWN)
    {
      return NULL;
    }

  g_rtc_lowerhalf.type = type;
  g_rtc_lowerhalf.initialized = true;

  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif
