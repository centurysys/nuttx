/****************************************************************************
 * arch/arm/src/stm32f0/stm32f0_exti_alarm.c
 *
 *   Copyright (C) 2009, 2012, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Diego Sanchez <dsanchez@nx-engineering.com>
 *           dev@ziggurat29.com (adaptation to stm32f0)
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32f0_gpio.h"
#include "stm32f0_exti.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_PERIODIC)
enum {
  RTC_ALARM = 0,
  RTC_TAMPER,
  RTC_WAKEUP,
  RTC_MAX
};

static uint32_t exti_rtc_bit[] = {
  EXTI_RTC_ALARM,
  EXTI_RTC_TAMPER,
  EXTI_RTC_WAKEUP
};

/* Interrupt handlers attached to the RTC EXTI */

static xcpt_t g_rtc_callback[RTC_MAX];
static void  *g_callback_arg[RTC_MAX];
static uint8_t irq_attached;


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32f0_exti_rtc_isr
 *
 * Description:
 *   EXTI ALARM interrupt service routine/dispatcher
 *
 ****************************************************************************/

static int stm32f0_exti_rtc_isr(int irq, void *context, FAR void *arg)
{
  int ret = OK;
  int exti_line;
  uint32_t reg, clr;

  reg = getreg32(STM32F0_EXTI_PR);
  reg &= (EXTI_RTC_ALARM | EXTI_RTC_TAMPER | EXTI_RTC_WAKEUP);

  while (reg)
    {
      clr = 0;
      exti_line = 0;

      if (reg & EXTI_RTC_ALARM)
        {
          exti_line = RTC_ALARM;
          clr = EXTI_RTC_ALARM;
        }
      else if (reg & EXTI_RTC_TAMPER)
        {
          exti_line = RTC_TAMPER;
          clr = EXTI_RTC_TAMPER;
        }
      else if (reg & EXTI_RTC_WAKEUP)
        {
          exti_line = RTC_WAKEUP;
          clr = EXTI_RTC_WAKEUP;
        }

      if (!exti_line)
          break;

      /* Dispatch the interrupt to the handler */
      if (g_rtc_callback[exti_line] != NULL)
        {
          ret = g_rtc_callback[exti_line](irq, context, g_callback_arg[exti_line]);
        }

      /* Clear the pending EXTI interrupt */
      putreg32(clr, STM32F0_EXTI_PR);
      reg &= ~clr;
    }

  return ret;
}

static int stm32f0_exti_rtc(int source, bool risingedge, bool fallingedge, bool event,
                            xcpt_t func, void *arg)
{
  g_rtc_callback[source] = func;
  g_callback_arg[source] = arg;

  /* Install external interrupt handlers (if not already attached) */

  if (func)
    {
      if (irq_attached == 0)
        {
          irq_attach(STM32F0_IRQ_RTC, stm32f0_exti_rtc_isr, NULL);
          up_enable_irq(STM32F0_IRQ_RTC);
        }
    }
  else
    {
      irq_attached &= ~(1 << source);

      if (irq_attached == 0)
        {
          up_disable_irq(STM32F0_IRQ_RTC);
        }
    }

  /* Configure rising/falling edges */

  modifyreg32(STM32F0_EXTI_RTSR,
              risingedge ? 0 : exti_rtc_bit[source],
              risingedge ? exti_rtc_bit[source] : 0);
  modifyreg32(STM32F0_EXTI_FTSR,
              fallingedge ? 0 : exti_rtc_bit[source],
              fallingedge ? exti_rtc_bit[source] : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32F0_EXTI_EMR,
              event ? 0 : exti_rtc_bit[source],
              event ? exti_rtc_bit[source] : 0);
  modifyreg32(STM32F0_EXTI_IMR,
              func ? 0 : exti_rtc_bit[source],
              func ? exti_rtc_bit[source] : 0);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32f0_exti_alarm
 *
 * Description:
 *   Sets/clears EXTI alarm interrupt.
 *
 * Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edget
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returns:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int stm32f0_exti_alarm(bool risingedge, bool fallingedge, bool event,
                       xcpt_t func, void *arg)
{
  return stm32f0_exti_rtc(RTC_ALARM, risingedge, fallingedge, event,
                          func, arg);
}
#endif

/****************************************************************************
 * Name: stm32f0_exti_wakeup
 *
 * Description:
 *   Sets/clears EXTI wakeup interrupt.
 *
 * Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edges
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returns:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
int stm32f0_exti_wakeup(bool risingedge, bool fallingedge, bool event,
                        xcpt_t func, void *arg)
{
  return stm32f0_exti_wakeup(RTC_WAKEUP, risingedge, fallingedge, event,
                             func, arg);
}
#endif
