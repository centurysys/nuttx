/************************************************************************************
 * configs/centurysys-xg50/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_CENTURYSYS_XG50_INCLUDE_BOARD_H
#define __CONFIGS_CENTURYSYS_XG50_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include <stm32l4.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32L476RG)
#  include <arch/board/centurysys-xg50.h>
#endif

/* DMA Channel/Stream Selections ****************************************************/
/* Stream selections are arbitrary for now but might become important in the future
 * is we set aside more DMA channels/streams.
 */

/* Values defined in arch/arm/src/stm32l4/chip/stm32l4x6xx_dma.h */

/* UART RX DMA configurations */

#define DMACHAN_USART1_RX DMACHAN_USART1_RX_2

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1_1
#define ADC2_DMA_CHAN DMACHAN_ADC2_2
#define ADC3_DMA_CHAN DMACHAN_ADC3_2

/* Alternate function pin selections ************************************************/

/* USART1 */

#define GPIO_USART1_RX   (GPIO_USART1_RX_2)     /* PB7 */
#define GPIO_USART1_TX   (GPIO_USART1_TX_2)     /* PB6 */
#ifndef CONFIG_CENTURYSYS_XG50_ADDON_OB_KM
#  define GPIO_USART1_RTS  (GPIO_USART1_RTS_DE_2) /* PB3 */
#  define GPIO_USART1_CTS  (GPIO_USART1_CTS_2)    /* PB4 */
#else
#  define GPIO_USART1_RTS  (STM32L4_NPORTS)       /* not assigned */
#  define GPIO_USART1_CTS  (STM32L4_NPORTS)       /* not assigned */
#endif
#define GPIO_USART1_DSR  (STM32L4_NPORTS)       /* not assigned */
#define GPIO_USART1_DCD  (STM32L4_NPORTS)       /* not assigned */
#define GPIO_USART1_RI   (STM32L4_NPORTS)       /* not assigned */

/* USART2 */

#define GPIO_USART2_RX   (GPIO_USART2_RX_1)     /* PA3 */
#define GPIO_USART2_TX   (GPIO_USART2_TX_1)     /* PA2 */
#define GPIO_USART2_RTS  (GPIO_USART2_RTS_DE_1) /* PA1 */
#define GPIO_USART2_CTS  (GPIO_USART2_CTS_1)    /* PA0 */
#define GPIO_USART2_DTR  (GPIO_PORTC | GPIO_PIN2 | GPIO_OUTPUT | GPIO_OPENDRAIN | \
                          GPIO_SPEED_50MHz)
#define GPIO_USART2_DSR  (GPIO_PORTC | GPIO_PIN3 | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_USART2_DCD  (GPIO_PORTA | GPIO_PIN4 | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_USART2_RI   (GPIO_PORTA | GPIO_PIN5 | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_USART2_RS485_DIR (GPIO_PORTC | GPIO_PIN9 | GPIO_OUTPUT)

/* USART3 */

#define GPIO_USART3_RX   (GPIO_USART3_RX_2)     /* PC5 */
#define GPIO_USART3_TX   (GPIO_USART3_TX_2)     /* PC4 */
#define GPIO_USART3_RTS  (GPIO_USART3_RTS_DE_1) /* PB1 */
#define GPIO_USART3_CTS  (GPIO_USART3_CTS_1)    /* PA6 */
#define GPIO_USART3_DTR  (GPIO_PORTB | GPIO_PIN2 | GPIO_OUTPUT | GPIO_OPENDRAIN | \
                          GPIO_SPEED_50MHz)
#define GPIO_USART3_DSR  (GPIO_PORTA | GPIO_PIN7 | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_USART3_DCD  (GPIO_PORTB | GPIO_PIN0 | GPIO_INPUT | GPIO_PULLUP)
#define GPIO_USART3_RI   (GPIO_PORTC | GPIO_PIN6 | GPIO_INPUT | GPIO_PULLUP)

/* UART4 */

#define GPIO_UART4_RX    (GPIO_UART4_RX_2)      /* PC11 */
#define GPIO_UART4_TX    (GPIO_UART4_TX_2)      /* PC10 */

/* I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */

#define GPIO_I2C1_SCL \
   (GPIO_I2C1_SCL_2 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SDA \
   (GPIO_I2C1_SDA_2 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C1_SCL_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN9)

#define GPIO_I2C2_SCL \
   (GPIO_I2C2_SCL_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C2_SDA \
   (GPIO_I2C2_SDA_1 | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)
#define GPIO_I2C2_SCL_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN10)
#define GPIO_I2C2_SDA_GPIO \
   (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
    GPIO_PORTB | GPIO_PIN11)

/* USB */

/* GPIO */

#define GPIO_B2B_RESET  (GPIO_OUTPUT | GPIO_PORTB | GPIO_PIN12 | GPIO_OUTPUT_SET)
#define GPIO_B2B_DCDC   (GPIO_OUTPUT | GPIO_PORTB | GPIO_PIN5  | GPIO_OUTPUT_SET | \
                         GPIO_OPENDRAIN)
#define GPIO_B2B_POWER  (GPIO_OUTPUT | GPIO_PORTB | GPIO_PIN13)
#define GPIO_B2B_WAKEUP (GPIO_OUTPUT | GPIO_PORTB | GPIO_PIN14)
#define GPIO_B2B_GPIO   (GPIO_OUTPUT | GPIO_PORTB | GPIO_PIN15)
#define GPIO_B2B_RI     (GPIO_INPUT  | GPIO_PORTC | GPIO_PIN6  | GPIO_PULLUP)
#define GPIO_MODESW     (GPIO_INPUT  | GPIO_PORTC | GPIO_PIN7  | GPIO_PULLUP)
#define GPIO_LEDSW      (GPIO_INPUT  | GPIO_PORTA | GPIO_PIN15)
#define GPIO_INITSW     (GPIO_INPUT  | GPIO_PORTC | GPIO_PIN13 | GPIO_PULLUP)
#define GPIO_RX485_RXE  (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN8)
#define GPIO_RX485_TXE  (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN9)
#define GPIO_DCDC_SEL   (GPIO_INPUT  | GPIO_PORTA | GPIO_PIN8  | GPIO_PULLUP)
#define GPIO_INT_EXT    (GPIO_INPUT  | GPIO_PORTA | GPIO_PIN9  | GPIO_PULLUP)
#define GPIO_UART2_ROUT (GPIO_INPUT  | GPIO_PORTA | GPIO_PIN10 | GPIO_PULLUP)

/* LEDs
 *
 * The Nucleo l476RG board provides a single user LED, LD2.  LD2
 * is the green LED connected to Arduino signal D13 corresponding to MCU I/O
 * PA5 (pin 21) or PB13 (pin 34) depending on the STM32 target.
 *
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT     (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT     (1 << BOARD_LED_G)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning                   LD2
 *   -------------------  -----------------------  -----------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     Blinking
 *   LED_IDLE             MCU is is sleep mode       Not used
 *
 * Thus if LD2, NuttX has successfully booted and is, apparently, running
 * normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        1
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32l4_board_initialize
 *
 * Description:
 *   All STM32L4 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32l4_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_CENTURYSYS_XG50_INCLUDE_BOARD_H */
