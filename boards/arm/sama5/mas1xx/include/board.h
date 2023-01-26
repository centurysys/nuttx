/****************************************************************************
 * boards/arm/sama5/mas1xx/include/board.h
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

#ifndef __BOARDS_ARM_SAMA5_MAS1XX_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAMA5_MAS1XX_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <nuttx/irq.h>
#endif

/* Clocking *****************************************************************/

/* On-board crystal frequencies */

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_SLOWCLK_FREQUENCY    (32768)     /* Slow Clock: 32.768KHz */
#define BOARD_UPLL_FREQUENCY       (480000000) /* USB PLL: 480MHz */

/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.
 * These definitions will configure operational clocking.
 */

#if defined(CONFIG_SAMA5_BOOT_SDRAM)
/* When booting from SDRAM, NuttX is loaded in SDRAM by an intermediate
 * bootloader.
 * That bootloader had to have already configured the PLL and SDRAM for
 * proper operation.
 *
 * In this case, we don not reconfigure the clocking.
 * Rather, we need to query the register settings to determine the clock
 * frequencies.
 * We can only assume that the Main clock source is the on-board 12MHz
 * crystal.
 */

#  include <arch/board/board_sdram.h>

#elif defined(CONFIG_MAS1XX_498MHZ)

/* This is the configuration results in a CPU clock of 498MHz.
 *
 * In this configuration, UPLL is the source of the UHPHS clock (if enabled).
 */

#  include <arch/board/board_498mhz.h>

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EMAC RMII connection to KSZ8081 Ethernet PHY *************************/

/* SAMA5D27 Interface        KSZ8081 Interface
 * ---- ------------ ------- ----------------------------------
 * PIO  Usage        Pin     Function
 * ---- ------------ ------- ----------------------------------
 * PB22 MDC          MDC     Management Interface Clock Input
 * PB23 MDIO         MDIO    Management Interface Data I/O
 * PB18 RX0          RX0     Receive Data Output 0
 * PB19 RX1          RX1     Receive Data Output 1
 * PB16 RXDV         RXDV    Receive Data Valid Output
 * PB17 RXER         RXER    Receive Error Output
 * PB20 TX0          TX0     Transmit Data Input 0
 * PB21 TX1          TX1     Transmit Data Input 1
 * PB14 TXCK         TXCK    Transmit Clock Input
 * PB15 TXEN         TXEN    Transmit Enable Input
 * ---- ------------ ------- ----------------------------------
 */

#define PIO_EMAC0_MDC     PIO_EMAC0_MDC_3
#define PIO_EMAC0_MDIO    PIO_EMAC0_MDIO_3
#define PIO_EMAC0_RX0     PIO_EMAC0_RX0_3
#define PIO_EMAC0_RX1     PIO_EMAC0_RX1_3
#define PIO_EMAC0_RXDV    PIO_EMAC0_RXDV_3
#define PIO_EMAC0_RXER    PIO_EMAC0_RXER_3
#define PIO_EMAC0_TX0     PIO_EMAC0_TX0_3
#define PIO_EMAC0_TX1     PIO_EMAC0_TX1_3
#define PIO_EMAC0_TXCK    PIO_EMAC0_TXCK_3
#define PIO_EMAC0_TXEN    PIO_EMAC0_TXEN_3

/* LED definitions **********************************************************/

/* There is an RGB LED on board the SAMA5D2-XULT.
 * The RED component is driven by the SDHC_CD pin (PA13) and so will not
 * be used.  The LEDs are provided VDD_LED and so bringing the LED low will
 * will illuminated the LED.
 *
 *   ------------------------------ ------------------- ---------------------
 *   SAMA5D2 PIO                    SIGNAL              USAGE
 *   ------------------------------ ------------------- ---------------------
 *   PA13                           SDHC_CD_PA13        Red LED
 *   PB5                            LED_GREEN_PB5       Green LED
 *   PB0                            LED_BLUE_PB0        Blue LED
 *   ------------------------------ ------------------- ---------------------
 */

#ifndef CONFIG_ARCH_LEDS

/* LED index values for use with board_userled() */

#define BOARD_GREEN       0
#define BOARD_BLUE        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_GREEN_BIT  (1 << BOARD_GREEN)
#define BOARD_BLUE_BIT   (1 << BOARD_BLUE)

#else

/* LED index values for use with board_userled() */

#define BOARD_BLUE        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_BLUE_BIT   (1 << BOARD_BLUE)
#endif

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows.  Note that only the GREEN LED is used in this case
 *
 *      SYMBOL            Val    Meaning                   Green LED
 *      ----------------- ---   -----------------------  -----------
 */

#define LED_STARTED       0  /* NuttX has been started     OFF       */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF       */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF       */
#define LED_STACKCREATED  1  /* Idle stack created         ON        */
#define LED_INIRQ         2  /* In an interrupt            N/C       */
#define LED_SIGNAL        2  /* In a signal handler        N/C       */
#define LED_ASSERTION     2  /* An assertion failed        N/C       */
#define LED_PANIC         3  /* The system has crashed     Flash     */
#undef  LED_IDLE             /* MCU is is sleep mode       Not used  */

/* Thus if the Green LED is statically on, NuttX has successfully  booted
 * and is, apparently, running normally.
 * If LED is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 */

/* Button definitions *******************************************************/

/* A single button, PB_USER (PB6), is available on the SAMA5D2-XULT
 *
 *  ------------------------------ ------------------- ----------------------
 *  SAMA5D2 PIO                    SIGNAL              USAGE
 *  ------------------------------ ------------------- ----------------------
 *  PB6                            USER_PB_PB6         PB_USER push button
 *  ------------------------------ ------------------- ----------------------
 *
 *  Closing PB_USER will bring PB6 to ground so 1) PB6 should have a weak
 * pull-up, and 2) when PB_USER is pressed, a low value will be senses.
 */

#define BUTTON_USER       0
#define NUM_BUTTONS       1

#define BUTTON_USER_BIT   (1 << BUTTON_USER)

/* Pin disambiguation *******************************************************/

/* Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc. Drivers, however, will use the pin selection without the numeric
 * suffix.
 * Additional definitions are required in this board.h file.
 * For example, if we wanted the PCK0on PB26, then the following definition
 * should appear in the board.h header file for that board:
 *
 *   #define PIO_PMC_PCK0 PIO_PMC_PCK0_1
 *
 * The PCK logic will then automatically configure PB26 as the PCK0 pin.
 */

/* USB Console.
 *
 */

#define PIO_UART1_RXD     PIO_UART1_RXD_1
#define PIO_UART1_TXD     PIO_UART1_TXD_1

/* DSUB-9 RS-232C is UART5 (FLEXCOM0).
 *
 */

#define PIO_UART5_RXD     PIO_FLEXCOM0_IO1
#define PIO_UART5_TXD     PIO_FLEXCOM0_IO0
#define PIO_UART5_RTS     PIO_FLEXCOM0_IO4
#define PIO_UART5_CTS     PIO_FLEXCOM0_IO3

/* RS-485 is UART7 (FLEXCOM2).
 *
 */

#define PIO_UART7_RXD     PIO_FLEXCOM0_IO1
#define PIO_UART7_TXD     PIO_FLEXCOM0_IO0
#define PIO_UART7_TXEN    PIO_FLEXCOM0_IO4
#define PIO_UART7_RXEN_N  PIO_FLEXCOM0_IO3

/* SPIs available on EXT1
 *
 *   ---- ------- -------------
 *   EXT1 BOARD      SAMA5D2
 *   PIN  NAME     PIO  FUNCTION
 *   ---- ------- -------------
 *    15  SPI_SS   PD29 SPI1
 *    16  SPI_MOSI PD26 SPI1
 *    17  SPI_MISO PD27 SPI1
 *    18  SPI_SCK  PD25 SPI1
 *   ---- ------- ---- --------
 */

#define PIO_SPI1_MISO     PIO_SPI1_MISO_1
#define PIO_SPI1_MOSI     PIO_SPI1_MOSI_1
#define PIO_SPI1_NPCS1    PIO_SPI1_NPCS1_1
#define PIO_SPI1_SPCK     PIO_SPI1_SPCK_1

/* SPI0 Definition on EXP */

#define PIO_SPI0_MISO     PIO_SPI0_MISO_1
#define PIO_SPI0_MOSI     PIO_SPI0_MOSI_1
#define PIO_SPI0_NPCS0    PIO_SPI0_NPCS0_1
#define PIO_SPI0_SPCK     PIO_SPI0_SPCK_1

/* CANs are available on J9:
 *
 *   ---- ------- -------------
 *   J9   BOARD      SAMA5D2
 *   PIN  NAME    PIO  FUNCTION
 *   ---- ------- -------------
 *    5   CANRX1  PC27 MCAN1-RX
 *    6   CANTX1  PC26 MCAN1-TX
 *    7   CANRX0  PC11 MCAN0-RX
 *    8   CANTX0  PC10 MCAN0-TX
 *   ---- ------- -------------
 */

#define PIO_MCAN0_RX      PIO_MCAN0_RX_2
#define PIO_MCAN0_TX      PIO_MCAN0_TX_2

/* SDIO - Used for both Port 0 & 1 ******************************************/

/* 386 KHz for initial inquiry stuff */

#define BOARD_SDMMC_IDMODE_PRESCALER    SDMMC_SYSCTL_SDCLKFS_DIV256
#define BOARD_SDMMC_IDMODE_DIVISOR      SDMMC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_SDMMC_MMCMODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_MMCMODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

#define BOARD_SDMMC_SD1MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_SD1MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

#define BOARD_SDMMC_SD4MODE_PRESCALER   SDMMC_SYSCTL_SDCLKFS_DIV8
#define BOARD_SDMMC_SD4MODE_DIVISOR     SDMMC_SYSCTL_DVS_DIV(1)

/****************************************************************************
 * Assembly Language Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__
  .macro config_sdram
  .endm
#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_SAMA5_MAS1XX_INCLUDE_BOARD_H */
