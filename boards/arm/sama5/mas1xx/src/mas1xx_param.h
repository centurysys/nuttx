/****************************************************************************
 * boards/arm/sama5/mas1xx/src/mas1xx_param.h
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

#ifndef __BOARDS_ARM_SAMA5_MAS1XX_SRC_MAS1XX_PARAM_H
#define __BOARDS_ARM_SAMA5_MAS1XX_SRC_MAS1XX_PARAM_H

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hw_parameter
{
  uint8_t macaddr0[6];
  uint8_t macaddr1[6];
  uint8_t macaddr2[6];

  struct
  {
    uint8_t year[4];
    uint8_t month[2];
    uint8_t day[2];
  } manufacture_date;

  uint32_t serial;

  struct
  {
    uint8_t number[5];
    uint8_t model[2];
    uint8_t revision;
  } board;

  uint8_t customer_code[4];
  uint8_t xio_id;
  uint8_t reserved[20];

  uint8_t bcc;
} __attribute__((packed));

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool mas1xx_init_parameter(void);

int mas1xx_param_get_macaddr(uint8_t *mac, int size);
int mas1xx_param_get_xio_id(uint8_t *xio_id);

#endif
