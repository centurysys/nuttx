/****************************************************************************
 * boards/arm/sama5/mas1xx/src/mas1xx.h
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

#ifndef __BOARDS_ARM_SAMA5_MAS1XX_SRC_MAS1XX_H
#define __BOARDS_ARM_SAMA5_MAS1XX_SRC_MAS1XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "sam_pio.h"
#include "hardware/sam_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_SDMMC      1
#define HAVE_AT25       1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_NETWORK    1
#define HAVE_I2CTOOL    1
#define HAVE_MACADDR    1

#define PIO_INT_ETH1      PIO_FIQ_1

/* SDMMC */

/* Can't support MMC/SD if the card interface(s) are not enable */

#if !defined(CONFIG_SAMA5_SDMMC) && !defined(CONFIG_SAMA5_SDMMC0)
#  undef HAVE_SDMMC
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_SDMMC) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_SDMCC
#endif

/* We need PIO interrupts on PIOD to support card detect interrupts */

#if defined(HAVE_SDMMC) && !defined(CONFIG_SAMA5_PIOA_IRQ)
#  warning PIOA interrupts not enabled.  No MMC/SD support.
#  undef HAVE_SDMMC
#endif

/* AT25 Serial FLASH */

/* Can't support the AT25 device if it SPI0 or AT25 support are not enabled */

#if !defined(CONFIG_SAMA5_SPI0) || !defined(CONFIG_MTD_AT25)
#  undef HAVE_AT25
#endif

/* Can't support AT25 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAMA5D3XPLAINED_AT25_AUTOMOUNT)
#  undef HAVE_AT25
#endif

/* If we are going to mount the AT25, then they user must also have told
 * us what to do with it by setting one of these.
 */

#ifndef CONFIG_FS_NXFFS
#  undef CONFIG_SAMA5D3XPLAINED_AT25_NXFFS
#endif

#if !defined(CONFIG_MAS1XX_AT25_FTL) && !defined(CONFIG_MAS1XX_AT25_NXFFS)
#  undef HAVE_AT25
#endif

#if defined(CONFIG_MAS1XX_AT25_FTL) && defined(CONFIG_MAS1XX_AT25_NXFFS)
#  warning Both CONFIG_MAS1XX_AT25_FTL and CONFIG_MAS1XX_AT25_NXFFS are set
#  warning Ignoring CONFIG_MAS1XX_AT25_NXFFS
#  undef CONFIG_MAS1XX_AT25_NXFFS
#endif

/* Assign minor device numbers.  For example, if we also use MINOR number 0
 * for the AT25, it should appear as /dev/mtdblock0
 */

#define _NAND_MINOR 0

#ifdef HAVE_NAND
#  define NAND_MINOR  _NAND_MINOR
#  define _AT25_MINOR (_NAND_MINOR+1)
#else
#  define _AT25_MINOR _NAND_MINOR
#endif

#ifdef HAVE_AT25
#  define AT25_MINOR  _AT25_MINOR
#endif

/* MMC/SD minor numbers:  The NSH device minor extended is extended to
 * support two devices.  If CONFIG_NSH_MMCSDMINOR is zero, these will be:
 * /dev/mmcsd0 and /dev/mmcsd1.
 */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

#ifdef HAVE_SDMMC

#  if ( defined(CONFIG_SAMA5_SDMMC0) && defined(CONFIG_SAMA5_SDMMC1) )
#    define SDMMC0_SLOTNO 0
#    define SDMMC1_SLOTNO 1
#  else
#    if ( defined(CONFIG_SAMA5_SDMMC0) )
#      define SDMMC0_SLOTNO 0
#    endif
#    if ( defined(CONFIG_SAMA5_SDMMC1) )
#      define SDMMC1_SLOTNO 0
#    endif
#  endif

#  ifdef CONFIG_SAMA5_SDMMC0
#     define SDMMC0_MINOR  CONFIG_NSH_MMCSDMINOR
#     define SDMMC1_MINOR  (CONFIG_NSH_MMCSDMINOR+1)
#  else
#     define SDMMC1_MINOR  CONFIG_NSH_MMCSDMINOR
#  endif
#else
#endif

/* USB Host / USB Device */

/* Either CONFIG_SAMA5_UHPHS or CONFIG_SAMA5_UDPHS must be defined,
 * or there is no USB of any kind.
 */

#if !defined(CONFIG_SAMA5_UHPHS)
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if !defined(CONFIG_SAMA5_UDPHS)
#  undef HAVE_USBDEV
#endif

/* CONFIG_USBDEV and CONFIG_USBHOST must also be defined */

#if !defined(CONFIG_USBDEV)
#  undef HAVE_USBDEV
#endif

#if defined(CONFIG_USBHOST)
#  if !defined(CONFIG_SAMA5_OHCI) && !defined(CONFIG_SAMA5_EHCI)
#    warning CONFIG_USBHOST is defined, but neither CONFIG_SAMA5_OHCI nor CONFIG_SAMA5_EHCI are defined
#  endif
#else
#  undef CONFIG_SAMA5_OHCI
#  undef CONFIG_SAMA5_EHCI
#endif

#if !defined(CONFIG_SAMA5_OHCI) && !defined(CONFIG_SAMA5_EHCI)
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Networking */

#if !defined(CONFIG_NET) || (!defined(CONFIG_SAMA5_EMACB) && !defined(CONFIG_SAMA5_GMAC))
#  undef HAVE_NETWORK
#endif

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define SAMA5_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define SAMA5_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* LEDs *********************************************************************/

/* There are R/B LEDs on board the MAS1xx.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PC1                            GPIO_PC1            MobileLED0(G)
 *   PC2                            GPIO_PC2            MobileLED0(R)
 *   PC3                            GPIO_PC3            MobileLED1(G)
 *   PC4                            GPIO_PC4            MobileLED1(R)
 *   PC5                            GPIO_PC5            PowerLED(G)
 *   ------------------------------ ------------------- ---------------------
 */

#define PIO_LED_MOBILE0_G (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                           PIO_PORT_PIOC | PIO_PIN1)
#define PIO_LED_MOBILE0_R (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                           PIO_PORT_PIOC | PIO_PIN2)
#define PIO_LED_MOBILE1_G (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                           PIO_PORT_PIOC | PIO_PIN3)
#define PIO_LED_MOBILE1_R (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_CLEAR | \
                           PIO_PORT_PIOC | PIO_PIN4)
#define PIO_LED_POWER_G   (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                           PIO_PORT_PIOC | PIO_PIN5)

/* Buttons ******************************************************************/

/* A single button, PB_USER (PB6), is available on the MAS1XX
 *
 *  ------------------------------ ------------------- ----------------------
 *  SAMA5D2 PIO                    SIGNAL              USAGE
 *  ------------------------------ ------------------- ----------------------
 *  PB25                           GPIO_PG25           OUT push button
 *  ------------------------------ ------------------- ----------------------
 */

#define PIO_BTN_USER (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOB | PIO_PIN25)
#define IRQ_BTN_USER  SAM_IRQ_PB25

/* DIN/DOUT *****************************************************************/

#define PIO_DIN0 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_SCHMITT | \
                  PIO_PORT_PIOD | PIO_PIN6)
#define PIO_DIN1 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_SCHMITT | \
                  PIO_PORT_PIOD | PIO_PIN7)
#define PIO_DIN2 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_SCHMITT | \
                  PIO_PORT_PIOD | PIO_PIN8)
#define PIO_DIN3 (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_SCHMITT | \
                  PIO_PORT_PIOD | PIO_PIN9)
#define PIO_DO0  (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                  PIO_PORT_PIOD | PIO_PIN10)
#define PIO_DO1  (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                  PIO_PORT_PIOD | PIO_PIN11)

#define PIO_MSP430_RST (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                        PIO_PORT_PIOD | PIO_PIN12)

/* SDMMC clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCI / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 96MHz, CLKDIV = 119, MCI_SPEED = 96MHz / 2 * (119+1) = 400 KHz */

#define SDMMC_INIT_CLKDIV          (119 << SDMMC_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 3, MCI_SPEED = 96MHz / 2 * (3+1) = 12 MHz */

#define SDMMC_MMCXFR_CLKDIV        (3 << SDMMC_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 1, MCI_SPEED = 96MHz / 2 * (1+1) = 24 MHz */

#define SDMMC_SDXFR_CLKDIV         (1 << SDMMC_MR_CLKDIV_SHIFT)
#define SDMMC_SDWIDEXFR_CLKDIV     SDMMC_SDXFR_CLKDIV

/* SDMMC Card Slots *********************************************************/

/* The MAS1XX provides a SD memory card slot:
 *  a full size SD card slot (SD)
 *
 * The full size SD card slot connects via SDMMC1.  The card detect discrete
 * is available on PA30 (pulled high).  The write protect discrete is tied to
 * ground and not available to software.  The slot only supports 4-bit
 * wide transfer mode, and the NuttX driver currently uses only the 4-bit
 * wide transfer mode.
 *
 *   PA30 SDMMC1_CD
 *   PA18 SDMMC1_DAT0
 *   PA19 SDMMC1_DAT1
 *   PA20 SDMMC1_DAT2
 *   PA21 SDMMC1_DAT3
 *   PA22 SDMMC1_CK
 *   PA28 SDMMC1_CDA
 */

#define IRQ_SDMMC1_CD   SAM_IRQ_PA30

/* USB Ports ****************************************************************/

/* The MAS1XX features one USB communication port:
 *
 *   1. Port B Host High Speed (EHCI) and Full Speed (OHCI) standard type A
 *      connector.
 *
 * The USB host port (only) is equipped with 500-mA high-side power
 * switch for self-powered and bus-powered applications.
 *
 */

/* Port B
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE3  EN5V_USBB   VBus power enable via MN3 SP2526A-2E dual power
 *                    switch.  PE3 (EN5V_USBB)connects to ENB pin of MN3.
 *                    MN3 OUTB (5V_USBB) is provided to pin 1 of J13 USB
 *                    A connector
 *
 *                    Active high for SP2526A-1; active low for SP2526A-2
 */

#define PIO_USBB_VBUS_ENABLE \
                     (PIO_OUTPUT | PIO_CFG_DEFAULT | PIO_OUTPUT_SET | \
                      PIO_PORT_PIOA | PIO_PIN31)

/*  Ports B
 *
 *   PIO  Signal Name Function
 *   ---- ----------- -------------------------------------------------------
 *   PE5  OVCUR_USB   Over-current indication from B
 */

#define PIO_USBB_VBUS_OVERCURRENT \
                     (PIO_INPUT | PIO_CFG_PULLUP | PIO_CFG_DEGLITCH | \
                      PIO_INT_BOTHEDGES | PIO_PORT_PIOB | PIO_PIN0)
#define IRQ_USBB_VBUS_OVERCURRENT \
                     SAM_IRQ_PB0

#define PIO_MOBILE_POWER_KEY (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_CLEAR | \
                              PIO_PORT_PIOC | PIO_PIN23)
#define PIO_MOBILE_RESET_KEY (PIO_OUTPUT | PIO_CFG_PULLUP | PIO_OUTPUT_CLEAR | \
                              PIO_PORT_PIOC | PIO_PIN24)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int sam_bringup(void);

/****************************************************************************
 * Name: sam_sdmmc_initialize
 *
 * Description:
 *   Initialize and configure one SDMMC slot
 *
 ****************************************************************************/

#ifdef HAVE_SDMMC
int sam_sdmmc_initialize(int slotno, int minor);
#endif

/****************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected SDMMC slot
 *
 ****************************************************************************/

#ifdef HAVE_SDMMC
bool sam_cardinserted(int slotno);
#endif

/****************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ****************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#endif

/****************************************************************************
 * Name: sam_usbinitialize
 *
 * Description:
 *   Called from sam_usbinitialize very early in initialization to setup
 *   USB-related PIO pins for the MAS1XX board.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_UHPHS) || defined(CONFIG_SAMA5_UDPHS)
void weak_function sam_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST
int sam_usbhost_initialize(void);
#endif

#ifdef CONFIG_SAMA5_TWI1
void board_i2c_initialize(void);
#endif

#ifdef CONFIG_ADC
int sam_adc_setup(void);
#endif

void weak_function sam_netinitialize(void);
int sam_emac0_setmac(void);

#ifdef CONFIG_LIBC_ZONEINFO_ROMFS
int sam_zoneinfo(int minor);
#endif

void sam_gpio_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM__SAMA5_MAS1XX_SRC_MAS1XX_H */
