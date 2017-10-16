/************************************************************************************
 * configs/sa-1xx/src/stm32_boot.c
 *
 *   Copyright (C) 2011-2013, 2015 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/board.h>

#include "stm32.h"
#include "sa-1xx.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Should we initialize the NX server using nx_start?  This is done for NxWidgets
 * (CONFIG_NXWIDGETS=y) and if the NxWidget::CNxServer class expects the RTOS do the
 * the NX initialization (CONFIG_NXWIDGET_SERVERINIT=n).  This combination of
 * settings is normally only used in the kernel build mode* (CONFIG_BUILD_PROTECTED)
 * when NxWidgets is unable to initialize NX from user-space.
 */

#undef HAVE_NXSTART

#if !defined(CONFIG_NX_MULTIUSER)
#  undef CONFIG_NX_START
#endif

#if defined(CONFIG_NXWIDGETS) && !defined(CONFIG_NXWIDGET_SERVERINIT)
#   define HAVE_NXSTART
#   include <nuttx/nx/nx.h>
#endif

/* Check if we will need to support the initialization kernel thread */

#undef HAVE_INITTHREAD

#ifdef CONFIG_BOARD_INITIALIZE
#  if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_LIB_BOARDCTL)
#    define HAVE_INITTHREAD 1
#  elif defined(HAVE_NXSTART)
#    define HAVE_INITTHREAD 1
#  elif defined(HAVE_TCINIT)
#    define HAVE_INITTHREAD 1
#  endif
#endif

#ifdef HAVE_INITTHREAD
#  include <stdlib.h>
#  include <assert.h>
#  include <nuttx/kthread.h>
#  ifndef CONFIG_SA1XX_BOARDINIT_PRIO
#    define CONFIG_SA1XX_BOARDINIT_PRIO 196
#  endif
#  ifndef CONFIG_SA1XX_BOARDINIT_STACK
#    define CONFIG_SA1XX_BOARDINIT_STACK 2048
#  endif
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_initthread
 *
 * Description:
 *   Board initialization kernel thread.  This thread exists to support
 *   initialization when CONFIG_BOARD_INITIALIZE is defined.  It is started by
 *   board_initialize() which runs on the IDLE thread.
 *
 *   This function thread exists because some initialization steps may require
 *   waiting for events.  Such waiting is not possible on the IDLE thread.
 *
 * Input Parameters:
 *   Standard task start-up parameters (none of which are used)
 *
 * Returned Value:
 *   Always returns EXIT_SUCCESS.
 *
 ************************************************************************************/

#ifdef HAVE_INITTHREAD
static int board_initthread(int argc, char *argv[])
{
  int ret;

  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in user-space
   * but the initialization function must run in kernel space.
   */

  ret = stm32_bringup();
  if (ret < 0)
    {
      gerr("ERROR: stm32_bringup failed: %d\n", ret);
    }

#ifdef HAVE_NXSTART
  /* Initialize the NX server */

  ret = nx_start();
  if (ret < 0)
    {
      gerr("ERROR: nx_start failed: %d\n", ret);
    }
#endif

  return EXIT_SUCCESS;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void)
{
#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || defined(CONFIG_STM32_SPI3)
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak function
   * stm32_spidev_initialize() has been brought into the link.
   */

  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

#ifdef CONFIG_STM32_OTGFS
  /* Initialize USB if the 1) OTG FS controller is in the configuration and 2)
   * disabled, and 3) the weak function stm32_usbinitialize() has been brought
   * the weak function stm32_usbinitialize() has been brought into the build.
   * Presumeably either CONFIG_USBDEV or CONFIG_USBHOST is also selected.
   */

  if (stm32_usbinitialize)
    {
      stm32_usbinitialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  stm32_led_initialize();
#endif

#ifdef CONFIG_STM32_USART3
  /* Power-up RS-232C transceiver */

  stm32_configgpio(GPIO_USART3_FORCEOFF_N);
  stm32_gpiowrite(GPIO_USART3_FORCEOFF_N, 1);
#endif

#ifdef CONFIG_STM32_ETHMAC
  stm32_configgpio(GPIO_ETH_PHY_RESET_N);
  stm32_configgpio(GPIO_ETH_PHY_INTRP_N);
  stm32_configgpio(GPIO_ETH_PHY_OSCENB);

  stm32_gpiowrite(GPIO_ETH_PHY_OSCENB, 1);
  stm32_gpiowrite(GPIO_ETH_PHY_RESET_N, 1);
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
#ifdef HAVE_INITTHREAD
  pid_t server;

  /* Start the board initialization kernel thread */

  server = kernel_thread("Board Init", CONFIG_SA1XX_BOARDINIT_PRIO,
                         CONFIG_SA1XX_BOARDINIT_STACK, board_initthread,
                         NULL);
  ASSERT(server > 0);
#else
  (void)stm32_bringup();
#endif
}
#endif
