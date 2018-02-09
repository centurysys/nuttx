/************************************************************************************
 * configs/centurysys-xg50/src/stm32_clockconfig.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: dev@ziggurat29.com
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
#include <arch/board/centurysys-xg50.h>

#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI and MSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT
#define MSIRDY_TIMEOUT HSERDY_TIMEOUT

/* HSE divisor to yield ~1MHz RTC clock */

#define HSE_DIVISOR (STM32L4_HSE_FREQUENCY + 500000) / 1000000

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   I provided this module when I was doing some debugging of a problem I had with
 *  clocking (it was helpful to do A/B tests).  I'm leaving it here in the config
 *  partially because I expect to have similar problems again as I develop more of
 *  the various peripheral support, but also because it may become necessary in the
 *  end for certain project configurations which have specialized clock configurations
 *  that aren't appropriate to expose in the 'arch' default code.
 *
 ************************************************************************************/

#if defined(CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG)
void stm32l4_board_clockconfig(void)
{
  volatile uint32_t regval;
  volatile int32_t timeout_hsi = 1, timeout_hse = 1, timeout_msi = 1;

#ifdef STM32L4_BOARD_ENABLE_HSI
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout_hsi = HSIRDY_TIMEOUT; timeout_hsi > 0; timeout_hsi--)
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32L4_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

#if defined(STM32L4_BOARD_ENABLE_MSI)
  /* Enable Internal Multi-Speed Clock (MSI) */

  /* Wait until the MSI is either off or ready (or until a timeout elapsed) */

  for (timeout_msi = MSIRDY_TIMEOUT; timeout_msi > 0; timeout_msi--)
    {
      if ((regval = getreg32(STM32L4_RCC_CR)), (regval & RCC_CR_MSIRDY) || ~(regval & RCC_CR_MSION))
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

  /* setting MSIRANGE */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= (STM32L4_BOARD_MSIRANGE | RCC_CR_MSION);    /* Enable MSI and frequency */
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the MSI is ready (or until a timeout elapsed) */

  for (timeout_msi = MSIRDY_TIMEOUT; timeout_msi > 0; timeout_msi--)
    {
      /* Check if the MSIRDY flag is the set in the CR */

      if ((getreg32(STM32L4_RCC_CR) & RCC_CR_MSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

#if defined(STM32L4_BOARD_ENABLE_HSE)
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout_hse = HSERDY_TIMEOUT; timeout_hse > 0; timeout_hse--)
    {
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32L4_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout_hsi > 0 && timeout_hse > 0 && timeout_msi > 0)
    {
#warning todo: regulator voltage according to clock freq
#if 0
      /* Ensure Power control is enabled before modifying it. */

      regval  = getreg32(STM32L4_RCC_APB1ENR1);
      regval |= RCC_APB1ENR1_PWREN;
      putreg32(regval, STM32L4_RCC_APB1ENR1);

      /* Select regulator voltage output Scale 1 mode to support system
       * frequencies up to 80 MHz.
       */

      regval  = getreg32(STM32L4_PWR_CR1);
      regval &= ~PWR_CR1_VOS_MASK;
      regval |= PWR_CR1_VOS_SCALE_1;
      putreg32(regval, STM32L4_PWR_CR1);
#endif

      /* Set the HCLK source/divider */

      regval  = getreg32(STM32L4_RCC_CFGR);
      regval &= ~RCC_CFGR_HPRE_MASK;
      regval |= STM32L4_RCC_CFGR_HPRE;
      putreg32(regval, STM32L4_RCC_CFGR);

      /* Set the PCLK2 divider */

      regval  = getreg32(STM32L4_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE2_MASK;
      regval |= STM32L4_RCC_CFGR_PPRE2;
      putreg32(regval, STM32L4_RCC_CFGR);

      /* Set the PCLK1 divider */

      regval  = getreg32(STM32L4_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE1_MASK;
      regval |= STM32L4_RCC_CFGR_PPRE1;
      putreg32(regval, STM32L4_RCC_CFGR);

#ifdef CONFIG_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval  = getreg32(STM32L4_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32L4_RCC_CFGR);
#endif

      /* Set the PLL source and main divider */

      regval  = getreg32(STM32L4_RCC_PLLCFG);

      /* XXX The choice of clock source to PLL (all three) is independent
       * of the sys clock source choice, review the STM32L4_BOARD_USEHSI
       * name; probably split it into two, one for PLL source and one
       * for sys clock source.
       */

#define USE_MAIN_PLL

      /* Set the PLL dividers and multipliers to configure the main PLL */

      regval = (STM32L4_PLLCFG_PLLM | STM32L4_PLLCFG_PLLN | STM32L4_PLLCFG_PLLP
                 | STM32L4_PLLCFG_PLLQ | STM32L4_PLLCFG_PLLR);

#if defined(STM32L4_BOARD_USEHSI)
      regval |= RCC_PLLCFG_PLLSRC_HSI;
#elif defined(STM32L4_BOARD_USEMSI)
      regval |= RCC_PLLCFG_PLLSRC_MSI;
#elif defined(STM32L4_BOARD_USEHSE)
      regval |= RCC_PLLCFG_PLLSRC_HSE;
#else
      #undef USE_MAIN_PLL
#endif

#ifdef USE_MAIN_PLL
      /* Configure Main PLL */

#ifdef STM32L4_PLLCFG_PLLP_ENABLED
      regval |= RCC_PLLCFG_PLLPEN;
#endif
#ifdef STM32L4_PLLCFG_PLLQ_ENABLED
      regval |= RCC_PLLCFG_PLLQEN;
#endif
#ifdef STM32L4_PLLCFG_PLLR_ENABLED
      regval |= RCC_PLLCFG_PLLREN;
#endif

      putreg32(regval, STM32L4_RCC_PLLCFG);

      /* Enable the main PLL */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32L4_RCC_CR);

      /* Wait until the PLL is ready */

      while (1)
        {
          regval = getreg32(STM32L4_RCC_CR);
          regval &= RCC_CR_PLLRDY;

          if (regval != 0)
            {
              break;
            }
        }
#endif /* USE_MAIN_PLL */

#ifdef CONFIG_STM32L4_SAI1PLL
      /* Configure SAI1 PLL */

      regval  = getreg32(STM32L4_RCC_PLLSAI1CFG);

      /* Set the PLL dividers and multipliers to configure the SAI1 PLL */

      regval = (STM32L4_PLLSAI1CFG_PLLN | STM32L4_PLLSAI1CFG_PLLP
                 | STM32L4_PLLSAI1CFG_PLLQ | STM32L4_PLLSAI1CFG_PLLR);

#ifdef STM32L4_PLLSAI1CFG_PLLP_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLPEN;
#endif
#ifdef STM32L4_PLLSAI1CFG_PLLQ_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLQEN;
#endif
#ifdef STM32L4_PLLSAI1CFG_PLLR_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLREN;
#endif

      putreg32(regval, STM32L4_RCC_PLLSAI1CFG);

      /* Enable the SAI1 PLL */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_PLLSAI1ON;
      putreg32(regval, STM32L4_RCC_CR);

       /* Wait until the PLL is ready */

      while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLSAI1RDY) == 0)
        {
        }
#endif /* CONFIG_STM32L4_SAI1PLL */

#ifdef CONFIG_STM32L4_SAI2PLL
      /* Configure SAI2 PLL */

      regval  = getreg32(STM32L4_RCC_PLLSAI2CFG);

      /* Set the PLL dividers and multipliers to configure the SAI2 PLL */

      regval = (STM32L4_PLLSAI2CFG_PLLN | STM32L4_PLLSAI2CFG_PLLP |
                STM32L4_PLLSAI2CFG_PLLR);

#ifdef STM32L4_PLLSAI2CFG_PLLP_ENABLED
      regval |= RCC_PLLSAI2CFG_PLLPEN;
#endif
#ifdef STM32L4_PLLSAI2CFG_PLLR_ENABLED
      regval |= RCC_PLLSAI2CFG_PLLREN;
#endif

      putreg32(regval, STM32L4_RCC_PLLSAI2CFG);

      /* Enable the SAI2 PLL */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_PLLSAI2ON;
      putreg32(regval, STM32L4_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLSAI2RDY) == 0)
        {
        }
#endif /* CONFIG_STM32L4_SAI2PLL */

      /* Enable FLASH prefetch, instruction cache, data cache, and 4 wait states */

#ifdef CONFIG_STM32L4_FLASH_PREFETCH
      regval = (FLASH_ACR_LATENCY_4 | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
#else
      regval = (FLASH_ACR_LATENCY_4 | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
#endif
      putreg32(regval, STM32L4_FLASH_ACR);

#ifdef USE_MAIN_PLL
      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32L4_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32L4_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32L4_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
        {
        }
#endif /* USE_MAIN_PLL */

#if defined(CONFIG_STM32L4_IWDG) || defined(CONFIG_RTC_LSICLOCK)
      /* Low speed internal clock source LSI */

      stm32l4_rcc_enablelsi();
#endif

#if defined(STM32L4_USE_LSE)
      /* Low speed external clock source LSE
       *
       * TODO: There is another case where the LSE needs to
       * be enabled: if the MCO1 pin selects LSE as source.
       * XXX and other cases, like automatic trimming of MSI for USB use
       */

      /* ensure Power control is enabled since it is indirectly required
       * to alter the LSE parameters.
       */
      stm32l4_pwr_enableclk(true);

      /* XXX other LSE settings must be made before turning on the oscillator
       * and we need to ensure it is first off before doing so.
       */

      /* Turn on the LSE oscillator
       * XXX this will almost surely get moved since we also want to use
       * this for automatically trimming MSI, etc.
       */

      stm32l4_rcc_enablelse();

#  if defined(STM32L4_BOARD_USEMSI)
      /* Now that LSE is up, auto trim the MSI */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_MSIPLLEN;
      putreg32(regval, STM32L4_RCC_CR);
#  endif
#endif
    }
}
#endif
