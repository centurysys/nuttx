/************************************************************************************
 * arch/arm/src/stm32l4/stm32l4_dma.h
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Sebastien Lorquet <sebastien@lorquet.fr>
 *           dev@ziggurat29.com
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

#ifndef __ARCH_ARM_SRC_STM32L4_STM32L4_DMA_H
#define __ARCH_ARM_SRC_STM32L4_STM32L4_DMA_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include "chip.h"

/* Include the correct DMA register definitions for this STM32 family */

#if defined(CONFIG_STM32L4_STM32L4X2) || defined(CONFIG_STM32L4_STM32L4X3)
#  include "chip/stm32l4x3xx_dma.h"
#elif defined(CONFIG_STM32L4_STM32L4X5)
#  include "chip/stm32l4x5xx_dma.h"
#elif defined(CONFIG_STM32L4_STM32L4X6)
#  include "chip/stm32l4x6xx_dma.h"
#else
#  error "Unsupported STM32L4 chip"
#endif

/* These definitions provide the bit encoding of the 'status' parameter passed to the
 * DMA callback function (see dma_callback_t).
 */

#  define DMA_STATUS_TEIF         DMA_CHAN_TEIF_BIT     /* Channel Transfer Error */
#  define DMA_STATUS_HTIF         DMA_CHAN_HTIF_BIT     /* Channel Half Transfer */
#  define DMA_STATUS_TCIF         DMA_CHAN_TCIF_BIT     /* Channel Transfer Complete */

#define DMA_STATUS_ERROR          (DMA_STATUS_TEIF)
#define DMA_STATUS_SUCCESS        (DMA_STATUS_TCIF|DMA_STATUS_HTIF)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* DMA_HANDLE provides an opaque are reference that can be used to represent a DMA
 * channel.
 */

typedef FAR void *DMA_HANDLE;

/* Description:
 *   This is the type of the callback that is used to inform the user of the
 *   completion of the DMA.
 *
 * Input Parameters:
 *   handle - Refers tot he DMA channel
 *   status - A bit encoded value that provides the completion status.  See the
 *            DMASTATUS_* definitions above.
 *   arg    - A user-provided value that was provided when stm32l4_dmastart() was
 *            called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

#ifdef CONFIG_DEBUG_DMA_INFO
struct stm32l4_dmaregs_s
{
  uint32_t isr;       /* Interrupt Status Register; each channel gets 4 bits */
  uint32_t cselr;     /* Channel Selection Register; chooses peripheral bound */
  uint32_t ccr;       /* Channel Configuration Register; determines functionality */
  uint32_t cndtr;     /* Channel Count Register; determines number of transfers */
  uint32_t cpar;      /* Channel Peripheral Address Register; determines start */
  uint32_t cmar;      /* Channel Memory Address Register; determines start */
};
#endif

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
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: stm32l4_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chan' argument.
 *   DMA channels are shared on the STM32L4:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   stm32l4_dma.h.
 *
 *   If the DMA channel is not available, then stm32l4_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   stm32l4_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the stm32l4_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Input Parameters:
 *   chan - Identifies the stream/channel resource
 *     This is a bit encoded value as provided by the DMACHAN_* definitions
 *     in chip/stm32l4x6xx_dma.h
 *
 * Returned Value:
 *   Provided that 'chan' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'chan' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold the DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is not
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32l4_dmachannel(unsigned int chan);

/****************************************************************************
 * Name: stm32l4_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to stm32l4_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until stm32l4_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void stm32l4_dmafree(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32l4_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32l4_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t ccr);

/****************************************************************************
 * Name: stm32l4_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32l4_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half);

/****************************************************************************
 * Name: stm32l4_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32l4_dmastop() is called, the DMA channel is
 *   reset and stm32l4_dmasetup() must be called before stm32l4_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

void stm32l4_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32l4_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

size_t stm32l4_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32l4_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address with the given configuration. This depends on the internal
 *   connections in the ARM bus matrix of the processor. Note that this
 *   only applies to memory addresses, it will return false for any peripheral
 *   address.
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_DMACAPABLE
bool stm32l4_dmacapable(uintptr_t maddr, uint32_t count, uint32_t ccr);
#else
#  define stm32l4_dmacapable(maddr, count, ccr) (true)
#endif

/****************************************************************************
 * Name: stm32l4_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32l4_dmasample(DMA_HANDLE handle, struct stm32l4_dmaregs_s *regs);
#else
#  define stm32l4_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: stm32l4_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32l4_dmadump(DMA_HANDLE handle, const struct stm32l4_dmaregs_s *regs,
                   const char *msg);
#else
#  define stm32l4_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32L4_STM32L4_DMA_H */
