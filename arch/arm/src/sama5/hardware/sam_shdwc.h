/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_shdwc.h
 * Reset Controller (SHDWC) definitions for the SAMA5
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SHDWC_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SHDWC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/sama5/chip.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SHDWC register offsets ****************************************************/

#define SAM_SHDWC_CR_OFFSET      0x00       /* Control Register */
#define SAM_SHDWC_MR_OFFSET      0x04       /* Mode Register  */
#define SAM_SHDWC_SR_OFFSET      0x08       /* Status Register */
#define SAM_SHDWC_WUIR_OFFSET    0x0c       /* Wakeup Inputs Register */

/* SHDWC register addresses **************************************************/

#define SAM_SHDWC_CR             (SAM_SHDWC_VBASE+SAM_SHDWC_CR_OFFSET)
#define SAM_SHDWC_MR             (SAM_SHDWC_VBASE+SAM_SHDWC_MR_OFFSET)
#define SAM_SHDWC_SR             (SAM_SHDWC_VBASE+SAM_SHDWC_SR_OFFSET)
#define SAM_SHDWC_WUIR           (SAM_SHDWC_VBASE+SAM_SHDWC_WUIR_OFFSET)

/* SHDWC register bit definitions ********************************************/

/* Shutdown Controller Control Register */

#define SHDWC_CR_SHDW            (1 << 0)   /* Bit 0:  Shutdown Command */
#define SHDWC_CR_KEY_SHIFT       (24)       /* Bits 24-31:  Password */
#define SHDWC_CR_KEY_MASK        (0xff << SHDWC_CR_KEY_SHIFT)
#  define SHDWC_CR_KEY           (0xa5 << SHDWC_CR_KEY_SHIFT)

/* Shutdown Controller Mode Register */

#define SHDWC_MR_RTCWKEN         (1 << 17)  /* Bit 17: Real-time Clock Wakeup Enable */
#define SHDWC_MR_ACCWKEN         (1 << 18)  /* Bit 18: Analog Comparator Controller
                                                         Wakeup Enable */
#define SHDWC_MR_RXLPWKEN        (1 << 19)  /* Bit 19: Debug Unit Wakeup Enable */
#define SHDWC_MR_WKUPDBC_SHIFT   (24)       /* Bits 24-26: Wakeup Inputs Debouncer Period */
#  define SHDWC_MR_WKUPDBC_IMMEDIATE  (0 << SHDWC_MR_WKUPDBC_SHIFT)
#  define SHDWC_MR_WKUPDBC_3_SCLK     (1 << SHDWC_MR_WKUPDBC_SHIFT)
#  define SHDWC_MR_WKUPDBC_32_SCLK    (2 << SHDWC_MR_WKUPDBC_SHIFT)
#  define SHDWC_MR_WKUPDBC_512_SCLK   (3 << SHDWC_MR_WKUPDBC_SHIFT)
#  define SHDWC_MR_WKUPDBC_4096_SCLK  (4 << SHDWC_MR_WKUPDBC_SHIFT)
#  define SHDWC_MR_WKUPDBC_32768_SCLK (5 << SHDWC_MR_WKUPDBC_SHIFT)

/* Shutdown Controller Status Register */

#define SHDWC_SR_WKUPS           (1 << 0)   /* Bit 0: Wakeup Status */
#define SHDWC_SR_RTCWK           (1 << 5)   /* Bit 5: Real-time Clock Wakeup */
#define SHDWC_SR_ACCWK           (1 << 6)   /* Bit 6: Analog Comparator Controller
                                                        Wakeup */
#define SHDWC_SR_WKUPIS_SHIFT(n) (16 + n)
#define   SHDWC_SR_WKUPIS(n)     (1 << SHDWC_SR_WKUPIS_SHIFT(n))

/* Shutdown Controller Wakeup Inputs Register */
#define SHDWC_WUIR_WKUPEN_MAX     9
#define SHDWC_WUIR_WKUPEN_MASK    0x000003ff
#define   SHDWC_WUIR_WKUPEN(n)    (1 << n)
#define SHDWC_WUIR_WKUPT_MAX      9
#define SHDWC_WUIR_WKUPT_MASK     0x03ff0000
#define SHDWC_WUIR_WKUPT_SHIFT(n) (16 + n)
#define   SHDWC_WUIR_WKUT(n)      (1 << SHDWC_WUIR_WKUPT_SHIFT(n))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_SHDWC_H */
