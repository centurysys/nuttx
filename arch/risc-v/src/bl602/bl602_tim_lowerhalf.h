/**
 * incubator-nuttx/arch/risc-v/src/bl602/bl602_lowerhalf.h
 *
 * Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 * Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __ARCH_RISCV_SRC_BL602_TIM_LOWERHALF_H
#define __ARCH_RISCV_SRC_BL602_TIM_LOWERHALF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <bl602_tim_lowerhalf.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_timer_initialize
 ****************************************************************************/

int bl602_timer_initialize(FAR const char *devpath, uint8_t timer);

#endif /* __ARCH_RISCV_SRC_BL602_TIM_LOWERHALF_H */
