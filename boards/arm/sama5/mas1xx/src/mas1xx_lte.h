/****************************************************************************
 * boards/arm/sama5/mas1xx/src/mas1xx_lte.h
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

#ifndef __BOARDS_ARM_SAMA5_MAS1XX_SRC_MAS1XX_LTE_H
#define __BOARDS_ARM_SAMA5_MAS1XX_SRC_MAS1XX_LTE_H

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void lte_pio_initialize(void);
bool get_lte_status(void);
bool lte_power_ctrl(bool on);
bool lte_reset(void);

#endif
