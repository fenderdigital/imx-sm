/*
** ###################################################################
**
**     Copyright 2023-2024 NXP
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of the copyright holder nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** ###################################################################
*/

/*==========================================================================*/
/* BSS-allocated diagnostic state owned by this TU.                         */
/* All symbols are declared `extern` in dev_sm_diagnostics.h and consumed   */
/* by the header inlines, by dev_sm_handlers.c, and by the SCMI MISC       */
/* 0x30/0x31/0x32 handlers in sm/rpc/scmi/rpc_scmi_misc.c.                 */
/*==========================================================================*/

#include "dev_sm.h"
#include "dev_sm_diagnostics.h"

#ifdef DEV_SM_DIAGNOSTICS

volatile uint32_t  s_smShmBlock[12]
    __attribute__((section(".bss"), used)) = {0U, 0U, 0U, 0U,
                                               0U, 0U, 0U, 0U,
                                               0U, 0U, 0U, 0U};

volatile sm_evt_t  s_smEventRing[SM_EVENT_RING_SIZE]
    __attribute__((section(".bss"), used));
volatile uint32_t  s_smEventRingHead
    __attribute__((section(".bss"), used)) = 0U;

volatile uint32_t  s_smGpcReqCount
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smGpcMaxCyc
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smEleIrqCount[3]
    __attribute__((section(".bss"), used)) = {0U, 0U, 0U};
volatile uint32_t  s_smFccuCount
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smLastGpcMix
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smLastGpcMs
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smLastGpcCyc
    __attribute__((section(".bss"), used)) = 0U;

volatile uint32_t  s_smSystickGapMaxCyc
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smSystickGapCount
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smEvtCounts[8]
    __attribute__((section(".bss"), used)) = {0U, 0U, 0U, 0U,
                                               0U, 0U, 0U, 0U};
volatile uint32_t  s_smLastSystickCyc
    __attribute__((section(".bss"), used)) = 0U;
volatile uint32_t  s_smShmUpdateCount
    __attribute__((section(".bss"), used)) = 0U;

/* Seqlock generation counter for s_smEventRing writers.
 * Even = idle, odd = writer mid-update.  Consumed by SCMI MISC 0x32
 * (MiscDiagRingRead) to detect torn reads across writer pre-emption. */
volatile uint32_t  s_smRingWriterSeq
    __attribute__((section(".bss"), used)) = 0U;

#endif /* DEV_SM_DIAGNOSTICS */
