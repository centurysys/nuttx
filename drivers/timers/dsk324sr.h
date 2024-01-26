/****************************************************************************
 * drivers/timers/dsk324sr.h
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

#ifndef __DRIVERS_TIMERS_DSK324SR_H
#define __DRIVERS_TIMERS_DSK324SR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSK324SR_REG_RTCSEC          0x00  /* Seconds register. */
#  define DSK324SR_RTCSEC_10SEC      (7 << 4)
#  define DSK324SR_RTCSEC_ST         (1 << 7)
#  define DSK324SR_RTCSEC_BCDMASK    0x7F

#define DSK324SR_REG_RTCMIN          0x01  /* Minutes register. */
#  define DSK324SR_RTCMIN_10MIN      (7 << 4)
#  define DSK324SR_RTCMIN_BCDMASK    0x7F

#define DSK324SR_REG_RTCHOUR         0x02  /* Hours register. */
#  define DSK324SR_RTCHOUR_10HOUR    (3 << 4)
#  define DSK324SR_RTCHOUR_BCDMASK   0x3F

#define DSK324SR_REG_RTCWKDAY        0x03  /* Day register. */
#  define DSK324SR_RTCWKDAY_BCDMASK  0x07

#define DSK324SR_REG_RTCDATE         0x04  /* Date register. */
#  define DSK324SR_RTCDATE_10DATE    (3 << 4)
#  define DSK324SR_RTCDATE_BCDMASK   0x3F

#define DSK324SR_REG_RTCMTH          0x05  /* Month register. */
#  define DSK324SR_RTCMTH_10MONTH    (1 << 4)
#  define DSK324SR_RTCMTH_BCDMASK    0x1F

#define DSK324SR_REG_RTCYEAR         0x06  /* Year register. */
#  define DSK324SR_RTCYEAR_10YEAR    (15 << 4)
#  define DSK324SR_RTCYEAR_BCDMASK   0xFF

#define DSK324SR_REG_SELECT          0x0b  /* Select Register */
#  define DSK324SR_SELECT_UTS        (1 << 0)
#  define DSK324SR_SELECT_AS         (1 << 1)
#  define DSK324SR_SELECT_TSS0       (2 << 0)
#  define DSK324SR_SELECT_TSS1       (3 << 0)
#  define DSK324SR_SELECT_CFS0       (4 << 0)
#  define DSK324SR_SELECT_CFS1       (5 << 0)
#  define DSK324SR_SELECT_TCS0       (6 << 0)
#  define DSK324SR_SELECT_TCS1       (7 << 0)
#  define DSK324SR_SELECT_TCS_0_5S   (0)
#  define DSK324SR_SELECT_TCS_2S     (DSK324SR_SELECT_TCS0)
#  define DSK324SR_SELECT_TCS_10S    (DSK324SR_SELECT_TCS1)
#  define DSK324SR_SELECT_TCS_30S    (DSK324SR_SELECT_TCS1 | DSK324SR_SELECT_TCS0)

#define DSK324SR_REG_FLAG            0x0c  /* Flag Register */
#  define DSK324SR_FLAG_UTF          (1 << 0) /* Update */
#  define DSK324SR_FLAG_AF           (1 << 1) /* Alarm */
#  define DSK324SR_FLAG_TF           (1 << 2) /* Timer */
#  define DSK324SR_FLAG_VDLF         (1 << 4) /* Voltage Detect Low */
#  define DSK324SR_FLAG_VDHF         (1 << 5) /* Voltage Detect High */
#  define DSK324SR_FLAG_VDF          (DSK324SR_FLAG_VDHF | DSK324SR_FLAG_VDLF)

#define DSK324SR_REG_CONTROL         0x0d  /* Control register. */
#  define DSK324SR_CONTROL_UTIE      (1 << 0)
#  define DSK324SR_CONTROL_AIE       (1 << 1)
#  define DSK324SR_CONTROL_TIE       (1 << 2)
#  define DSK324SR_CONTROL_TE        (1 << 3)
#  define DSK324SR_CONTROL_FIE       (1 << 4)
#  define DSK324SR_CONTROL_RAM       (1 << 5)
#  define DSK324SR_CONTROL_TEST      (1 << 6)
#  define DSK324SR_CONTROL_RESET     (1 << 7)

#define DD3225TS_REG_TEST            0x0e /* TEST register (DS3225TS only) */

#endif /* __DRIVERS_TIMERS_DSK324SR_H */
