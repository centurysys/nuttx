/****************************************************************************
 * include/nuttx/timers/dsk324sr.h
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

#ifndef __INCLUDE_NUTTX_TIMERS_DSK324SR_H
#define __INCLUDE_NUTTX_TIMERS_DSK324SR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_RTC_DSK324SR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Linux-like ioctl */
#define RTC_ALM_READ _RTCIOC(RTC_USER_IOCBASE)
#define RTC_ALM_SET  _RTCIOC(RTC_USER_IOCBASE + 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rtc_wkalrm
{
  uint8_t enabled; /* 0 = alarm disabled, 1 = alarm enabled */
  uint8_t pending; /* 0 = alarm not pending, 1 = alarm pending */
  struct rtc_time time;  /* time the alarm is set to */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct rtc_lowerhalf_s *dsk324sr_rtc_lowerhalf(struct i2c_master_s *i2c,
                                               uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_RTC_DSK324SR */
#endif /* __INCLUDE_NUTTX_TIMERS_DSK324SR_H */
