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
/*!
 * @addtogroup DEV_SM_MX95_DIAGNOSTICS
 * @{
 *
 * @file
 * @brief
 *
 * Header file containing SM diagnostics instrumentation. All
 * contents are gated by the DEV_SM_DIAGNOSTICS compile switch; when
 * undefined, IrqPrioUpdate() elides to a no-op and all event ring,
 * shared-memory, and gap-tracking state is excluded from the build.
 */
/*==========================================================================*/

#ifndef DEV_SM_DIAGNOSTICS_H
#define DEV_SM_DIAGNOSTICS_H

#include "dev_sm_handlers.h"

/* SM event class identifiers - defined unconditionally so call sites
 * referencing them still compile when DEV_SM_DIAGNOSTICS is off
 * (sm_evt_log() becomes a no-op stub in that case). */
#define SM_EVT_GPC_REQ_ENTRY        0x01U
#define SM_EVT_GPC_REQ_EXIT         0x02U
#define SM_EVT_ELE_GROUP1           0x10U
#define SM_EVT_ELE_GROUP2           0x11U
#define SM_EVT_ELE_GROUP3           0x12U
#define SM_EVT_FCCU_ALARM           0x20U
#define SM_EVT_SYSTICK_1K           0x30U
#define SM_EVT_SM_GAP_EXCEEDED      0x31U
#define SM_EVT_CLOCK_RATE_SET       0x40U
#define SM_EVT_POWER_STATE_SET      0x41U

#ifdef DEV_SM_DIAGNOSTICS

/* SM event ring sizing */
#define SM_EVENT_RING_SIZE      128U
#define SM_EVENT_RING_MASK      (SM_EVENT_RING_SIZE - 1U)

/* Shared-memory diagnostics base.
 * Layout: [0]=magic 0x534D4731, [1]=version, [2]=ms,
 *         [3]=gap_max_cyc, [4]=gap_count, [5]=gpc_req_cnt,
 *         [6]=gpc_max_cyc, [7]=event_ring_head,
 *         [8..10]=ele_irq[0..2], [11]=last_update_ms */
#define SM_SHM_BASE (&s_smShmBlock[0])

/* SM event ring entry */
typedef struct {
    uint32_t ms;
    uint32_t cyccnt;
    uint8_t  evt;
    uint8_t  mix_idx;
    uint8_t  stat;
    uint8_t  pad;
} sm_evt_t;

/* Externally-linked diagnostic state defined in dev_sm_handlers.c */
extern uint64_t s_smTimeMsec;
extern volatile uint32_t s_smShmBlock[12];
extern volatile sm_evt_t s_smEventRing[SM_EVENT_RING_SIZE];
extern volatile uint32_t s_smEventRingHead;
extern volatile uint32_t s_smRingWriterSeq;
extern volatile uint32_t s_smEvtCounts[8];
extern volatile uint32_t s_smGpcReqCount;
extern volatile uint32_t s_smGpcMaxCyc;
extern volatile uint32_t s_smEleIrqCount[3];
extern volatile uint32_t s_smFccuCount;
extern volatile uint32_t s_smLastGpcMix;
extern volatile uint32_t s_smLastGpcMs;
extern volatile uint32_t s_smLastGpcCyc;
extern volatile uint32_t s_smSystickGapMaxCyc;
extern volatile uint32_t s_smSystickGapCount;
extern volatile uint32_t s_smLastSystickCyc;
extern volatile uint32_t s_smShmUpdateCount;

/* Dynamic IRQ priority adjustment implementation */
void IrqPrioUpdate_Impl(irq_prio_info_t *pInfo);

/* Read DWT CYCCNT */
static inline uint32_t sm_cyccnt(void)
{
    return *((volatile const uint32_t *)0xE0001004U);
}

/* Publish key SM diagnostics to shared memory for A55 /dev/mem read */
static inline void sm_update_shm(void)
{
    SM_SHM_BASE[0] = 0x534D4731U;  /* magic "SMG1" */
    SM_SHM_BASE[1] = 1U;           /* version */
    SM_SHM_BASE[2] = (uint32_t)s_smTimeMsec;
    SM_SHM_BASE[3] = s_smSystickGapMaxCyc;
    SM_SHM_BASE[4] = s_smSystickGapCount;
    SM_SHM_BASE[5] = s_smGpcReqCount;
    SM_SHM_BASE[6] = s_smGpcMaxCyc;
    SM_SHM_BASE[7] = s_smEventRingHead;
    SM_SHM_BASE[8] = s_smEleIrqCount[0];
    SM_SHM_BASE[9] = s_smEleIrqCount[1];
    SM_SHM_BASE[10] = s_smEleIrqCount[2];
    SM_SHM_BASE[11] = s_smShmUpdateCount++;
}

/* Append to event ring; seqlock-protected so SCMI MISC 0x32 can
 * detect torn reads across writer pre-emption. */
static inline void sm_evt_log(uint8_t evt, uint8_t mix, uint8_t stat)
{
    /* Mask interrupts so a nested writer cannot tear the record body.
     * ~30ns @200MHz; FCCU IRQ already higher priority and unaffected.
     * primask save/restore preserves nesting semantics. */
    uint32_t primask;
    __asm volatile ("mrs %0, primask\n\tcpsid i"
                    : "=r"(primask) :: "memory");

    /* Enter seqlock: bump generation to odd, publish before body */
    uint32_t seq = s_smRingWriterSeq + 1U;
    s_smRingWriterSeq = seq;
    __asm volatile ("dmb ish" ::: "memory");

    uint32_t idx = s_smEventRingHead & SM_EVENT_RING_MASK;
    volatile sm_evt_t *e = &s_smEventRing[idx];
    e->ms = (uint32_t)s_smTimeMsec;
    e->cyccnt = sm_cyccnt();
    e->evt = evt;
    e->mix_idx = mix;
    e->stat = stat;
    e->pad = 0U;

    /* Publish body before head advance; head advance before seqlock exit */
    __asm volatile ("dmb ish" ::: "memory");
    s_smEventRingHead++;
    __asm volatile ("dmb ish" ::: "memory");

    /* Exit seqlock: bump generation back to even */
    s_smRingWriterSeq = seq + 1U;
    __asm volatile ("dmb ish" ::: "memory");

    /* Per-event-class fast counter consumed by SCMI MISC 0x31 reply. */
    s_smEvtCounts[(evt >> 4) & 0x7U]++;

    /* Restore PRIMASK (re-enables interrupts if they were enabled) */
    __asm volatile ("msr primask, %0" :: "r"(primask) : "memory");
}

/* Always-inline wrapper: dispatches to the real implementation. */
__attribute__((always_inline)) static inline void IrqPrioUpdate(
    irq_prio_info_t *pInfo)
{
    IrqPrioUpdate_Impl(pInfo);
}

#else /* DEV_SM_DIAGNOSTICS */

/* Diagnostics disabled: all instrumentation elides to no-ops. */

__attribute__((always_inline)) static inline uint32_t sm_cyccnt(void)
{
    return 0U;
}

__attribute__((always_inline)) static inline void sm_update_shm(void)
{
}

__attribute__((always_inline)) static inline void sm_evt_log(uint8_t evt,
    uint8_t mix, uint8_t stat)
{
    (void)evt;
    (void)mix;
    (void)stat;
}

__attribute__((always_inline)) static inline void IrqPrioUpdate(
    irq_prio_info_t *pInfo)
{
    (void)pInfo;
}

#endif /* DEV_SM_DIAGNOSTICS */

#endif /* DEV_SM_DIAGNOSTICS_H */

/** @} */
