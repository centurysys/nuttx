/****************************************************************************
 * boards/arm/sama5/mas1xx/include/boardctl.h
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

#ifndef __BOARDS_ARM_SAMA5_MAS1XX_INCLUDE_BOARDCTL_H
#define __BOARDS_ARM_SAMA5_MAS1XX_INCLUDE_BOARDCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/boardctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BIOC_ENABLE_WAKEUP   (BOARDIOC_USER + 1)
#define BIOC_DISABLE_WAKEUP  (BOARDIOC_USER + 2)
#define BIOC_GET_WAKEUP      (BOARDIOC_USER + 3)
#define BIOC_SHUTDOWN        (BOARDIOC_USER + 4)

#define BIT(n)        (1 << n)
#define WKUPEN_OPTSW  BIT(4)
#define WKUPEN_MSP430 BIT(5)
#define WKUPEN_ALARM  BIT(6)

#endif
