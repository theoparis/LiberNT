/*
 * PROJECT:     LiberNT HAL — ARM64 Apple AIC platform support
 * LICENSE:     GPL-3.0-or-later (https://spdx.org/licenses/GPL-3.0-or-later)
 * PURPOSE:     Interrupt-controller backend for Apple SoCs without a GIC.
 *              The controller is found through the firmware devicetree that
 *              the loader copies into the ARM64 loader block.
 * COPYRIGHT:   Copyright 2026 Ahmed ARIF <arif.ing@outlook.com>
 */

#include <ntifs.h>
#include <arc/arc.h>
#include <ndk/kefuncs.h>
#include <ioaccess.h>
#include <halfuncs.h>
#include <reactos/arm64/sysreg.h>
#include <reactos/arm64/fdt.h>
#include "halext.h"
#include "apple_aic.h"

#define NDEBUG
#include <debug.h>

#define APPLE_AIC_SPURIOUS_INTID    1023
#define APPLE_AIC_HW_INTID_BASE     32
#define APPLE_AIC_MAX_HW_IRQ        988
#define APPLE_AIC_DEFAULT_PRIORITY  0xA0
#define APPLE_AIC_SGI_COUNT         4
#define APPLE_AIC_TIMER_COUNT       2
#define APPLE_AIC_TIMER_VIRT_INTID  27
#define APPLE_AIC_TIMER_PHYS_INTID  30
#define APPLE_AIC_IRQ_TARGET_CPU    0

#define AIC_INFO                    0x0004
#define AIC_INFO_NR_IRQ             0xFFFF
#define AIC_EVENT                   0x2004
#define AIC_TARGET_CPU              0x3000
#define AIC_MAX_IRQ                 0x400

#define AIC2_INFO1                  0x0004
#define AIC2_INFO1_NR_IRQ           0xFFFF
#define AIC2_INFO3                  0x000C
#define AIC2_INFO3_MAX_IRQ          0xFFFF
#define AIC2_CONFIG                 0x0014
#define AIC2_CONFIG_ENABLE          0x1
#define AIC2_IRQ_CFG                0x2000

#define AIC_EVENT_NUM(Event)        ((Event) & 0xFFFF)
#define AIC_EVENT_TYPE(Event)       (((Event) >> 16) & 0xFF)
#define AIC_EVENT_DIE(Event)        (((Event) >> 24) & 0xFF)
#define AIC_EVENT_TYPE_IRQ          1

#define IPI_RR_CPU(Cpu)             ((ULONGLONG)(Cpu) & 0xFF)
#define IPI_RR_CLUSTER(Cluster)     (((ULONGLONG)(Cluster) & 0xFF) << 16)
#define IPI_SR_PENDING              0x1ULL

#define CNT_CTL_ENABLE              0x1ULL
#define CNT_CTL_IMASK               0x2ULL
#define CNT_CTL_ISTATUS             0x4ULL

#define MPIDR_AFF0(Mpidr)           ((ULONG)((Mpidr) & 0xFF))
#define MPIDR_AFF1(Mpidr)           ((ULONG)(((Mpidr) >> 8) & 0xFF))

BOOLEAN AppleAicDetected = FALSE;

static ULONGLONG AppleAicBase;
static ULONGLONG AppleAicLength;
static ULONGLONG AppleAicEventBase;
static ULONGLONG AppleAicEventLength;
static ULONG AppleAicVersion;
static ULONG AppleAicNrIrq;
static ULONG AppleAicEventOffset;
static ULONG AppleAicMaskSet;
static ULONG AppleAicMaskClr;
static ULONG AppleAicSwClr;
static BOOLEAN AppleAicHwReady;

static volatile UCHAR AppleAicPmr[MAXIMUM_PROCESSORS];
static volatile LONG AppleAicSgiPending[MAXIMUM_PROCESSORS];
static volatile LONG AppleAicTimerEnabled[MAXIMUM_PROCESSORS];
static volatile LONG AppleAicTimerDeferred[MAXIMUM_PROCESSORS];
static ULONGLONG AppleAicCpuMpidr[MAXIMUM_PROCESSORS];

static volatile LONG AppleAicHwEnabled[(APPLE_AIC_MAX_HW_IRQ + 31) / 32];
static volatile LONG AppleAicHwDeferred[(APPLE_AIC_MAX_HW_IRQ + 31) / 32];
static volatile LONG AppleAicHwDeferredCount;

static UCHAR AppleAicSgiPriority[APPLE_AIC_SGI_COUNT] =
{
    HAL_ARM64_SGI_IPI_PRIORITY,
    HAL_ARM64_SGI_APC_PRIORITY,
    HAL_ARM64_SGI_DPC_PRIORITY,
    HAL_ARM64_SGI_FREEZE_PRIORITY
};
static UCHAR AppleAicTimerPriority[APPLE_AIC_TIMER_COUNT];
static UCHAR AppleAicHwPriority[APPLE_AIC_MAX_HW_IRQ];

static const UCHAR AppleAicIrqlToPmr[HIGH_LEVEL + 1] =
{
    0xFF, 0xE0, 0xD0,
    0xC0, 0xB0, 0xA0, 0x90, 0x80,
    0x70, 0x60, 0x50, 0x40, 0x30,
    0x20,
    0x10,
    0x00
};

FORCEINLINE
ULONG
AppleAicCurrentCpu(VOID)
{
    ULONG Cpu = KeGetCurrentProcessorNumber();

    return (Cpu < MAXIMUM_PROCESSORS) ? Cpu : 0;
}

FORCEINLINE
ULONGLONG
AppleAicMaskInterrupts(VOID)
{
    ULONGLONG Daif;

    Daif = ARM64_READ_SYSREG(DAIF);
    ARM64_DAIFSET(3);
    return Daif;
}

FORCEINLINE
VOID
AppleAicRestoreInterrupts(
    _In_ ULONGLONG Daif)
{
    ARM64_WRITE_SYSREG(DAIF, Daif);
}

FORCEINLINE
ULONG
AppleAicRead(
    _In_ ULONG Offset)
{
    return *HalpMmio((ULONG_PTR)AppleAicBase, Offset);
}

FORCEINLINE
VOID
AppleAicWrite(
    _In_ ULONG Offset,
    _In_ ULONG Value)
{
    *HalpMmio((ULONG_PTR)AppleAicBase, Offset) = Value;
}

FORCEINLINE
ULONG
AppleAicReadEvent(VOID)
{
    ULONG Event;

    Event = *HalpMmio((ULONG_PTR)AppleAicEventBase, AppleAicEventOffset);
    ARM64_DSB(SY);
    return Event;
}

static
LONG
AppleAicIntIdToTimer(
    _In_ ULONG IntId)
{
    if (IntId == APPLE_AIC_TIMER_VIRT_INTID)
        return 0;
    if (IntId == APPLE_AIC_TIMER_PHYS_INTID)
        return 1;
    return -1;
}

static
ULONGLONG
AppleAicReadTimerCtl(
    _In_ LONG Timer)
{
    return (Timer == 0) ? ARM64_READ_SYSREG(CNTV_CTL_EL0) :
                          ARM64_READ_SYSREG(CNTP_CTL_EL0);
}

static
VOID
AppleAicWriteTimerCtl(
    _In_ LONG Timer,
    _In_ ULONGLONG Ctl)
{
    if (Timer == 0)
        ARM64_WRITE_SYSREG(CNTV_CTL_EL0, Ctl);
    else
        ARM64_WRITE_SYSREG(CNTP_CTL_EL0, Ctl);
    ARM64_ISB();
}

static
VOID
AppleAicSetTimerImask(
    _In_ LONG Timer,
    _In_ BOOLEAN Mask)
{
    ULONGLONG Ctl = AppleAicReadTimerCtl(Timer);

    if (!(Ctl & CNT_CTL_ENABLE))
        return;

    Ctl = Mask ? (Ctl | CNT_CTL_IMASK) : (Ctl & ~CNT_CTL_IMASK);
    AppleAicWriteTimerCtl(Timer, Ctl);
}

static
VOID
AppleAicRaiseIpi(
    _In_ ULONG Cpu)
{
    ULONGLONG MyMpidr = ARM64_READ_SYSREG(MPIDR_EL1);
    ULONGLONG TargetMpidr;

    if (Cpu == AppleAicCurrentCpu())
        TargetMpidr = MyMpidr;
    else
        TargetMpidr = AppleAicCpuMpidr[Cpu];

    if (Cpu != 0 && Cpu != AppleAicCurrentCpu() && TargetMpidr == 0)
        return;

    if (MPIDR_AFF1(TargetMpidr) == MPIDR_AFF1(MyMpidr))
    {
        ARM64_WRITE_SYSREG(S3_5_C15_C0_0, IPI_RR_CPU(MPIDR_AFF0(TargetMpidr)));
    }
    else
    {
        ARM64_WRITE_SYSREG(S3_5_C15_C0_1,
                           IPI_RR_CPU(MPIDR_AFF0(TargetMpidr)) |
                           IPI_RR_CLUSTER(MPIDR_AFF1(TargetMpidr)));
    }
    ARM64_ISB();
}

static
BOOLEAN
AppleAicSgiEligible(
    _In_ ULONG Cpu,
    _In_ ULONG Pmr)
{
    ULONG Pending = (ULONG)AppleAicSgiPending[Cpu];
    ULONG Sgi;

    for (Sgi = 0; Sgi < APPLE_AIC_SGI_COUNT; Sgi++)
    {
        if ((Pending & (1u << Sgi)) && AppleAicSgiPriority[Sgi] < Pmr)
            return TRUE;
    }

    return FALSE;
}

static
VOID
AppleAicSetHwDeferred(
    _In_ ULONG Irq)
{
    LONG Bit = (LONG)(1u << (Irq & 31));
    LONG Old = InterlockedOr(&AppleAicHwDeferred[Irq >> 5], Bit);

    if (!(Old & Bit))
        InterlockedIncrement(&AppleAicHwDeferredCount);
}

static
VOID
AppleAicClearHwDeferred(
    _In_ ULONG Irq)
{
    LONG Bit = (LONG)(1u << (Irq & 31));
    LONG Old = InterlockedAnd(&AppleAicHwDeferred[Irq >> 5], ~Bit);

    if (Old & Bit)
        InterlockedDecrement(&AppleAicHwDeferredCount);
}

static
VOID
AppleAicMaskHw(
    _In_ ULONG Irq)
{
    if (!AppleAicHwReady)
        return;

    AppleAicWrite(AppleAicMaskSet + ((Irq >> 5) * 4), 1u << (Irq & 31));
    ARM64_DSB(SY);
}

static
VOID
AppleAicUnmaskHw(
    _In_ ULONG Irq)
{
    if (!AppleAicHwReady)
        return;

    AppleAicWrite(AppleAicMaskClr + ((Irq >> 5) * 4), 1u << (Irq & 31));
    ARM64_DSB(SY);
}

static
BOOLEAN
AppleAicHwIsEnabled(
    _In_ ULONG Irq)
{
    return ((ULONG)AppleAicHwEnabled[Irq >> 5] & (1u << (Irq & 31))) != 0;
}

static
ULONG
AppleAicAcknowledgeInterrupt(VOID)
{
    ULONG Cpu = AppleAicCurrentCpu();
    ULONG Pmr = AppleAicPmr[Cpu];
    ULONG BestPriority = Pmr;
    ULONG Best = APPLE_AIC_SPURIOUS_INTID;
    ULONG Pending;
    ULONG Sgi;
    LONG Timer;

    if (ARM64_READ_SYSREG(S3_5_C15_C1_1) & IPI_SR_PENDING)
    {
        ARM64_WRITE_SYSREG(S3_5_C15_C1_1, IPI_SR_PENDING);
        ARM64_ISB();
    }

    Pending = (ULONG)AppleAicSgiPending[Cpu];
    for (Sgi = 0; Sgi < APPLE_AIC_SGI_COUNT; Sgi++)
    {
        ULONG Priority;

        if (!(Pending & (1u << Sgi)))
            continue;

        Priority = AppleAicSgiPriority[Sgi];
        if (Priority < Pmr &&
            (Priority < BestPriority || (Priority == BestPriority && Sgi < Best)))
        {
            BestPriority = Priority;
            Best = Sgi;
        }
    }

    for (Timer = 0; Timer < APPLE_AIC_TIMER_COUNT; Timer++)
    {
        ULONGLONG Ctl = AppleAicReadTimerCtl(Timer);
        ULONG IntId = (Timer == 0) ? APPLE_AIC_TIMER_VIRT_INTID : APPLE_AIC_TIMER_PHYS_INTID;
        ULONG Priority;

        if ((Ctl & (CNT_CTL_ENABLE | CNT_CTL_IMASK | CNT_CTL_ISTATUS)) !=
            (CNT_CTL_ENABLE | CNT_CTL_ISTATUS))
        {
            continue;
        }

        if (!((ULONG)AppleAicTimerEnabled[Cpu] & (1u << Timer)))
        {
            AppleAicWriteTimerCtl(Timer, Ctl | CNT_CTL_IMASK);
            continue;
        }

        Priority = AppleAicTimerPriority[Timer];
        if (Priority >= Pmr)
        {
            AppleAicWriteTimerCtl(Timer, Ctl | CNT_CTL_IMASK);
            InterlockedOr(&AppleAicTimerDeferred[Cpu], (LONG)(1u << Timer));
            continue;
        }

        if (Priority < BestPriority || (Priority == BestPriority && IntId < Best))
        {
            BestPriority = Priority;
            Best = IntId;
        }
    }

    if (Best != APPLE_AIC_SPURIOUS_INTID)
    {
        if (Best < APPLE_AIC_SGI_COUNT)
        {
            InterlockedAnd(&AppleAicSgiPending[Cpu], (LONG)~(1u << Best));
        }
        else
        {
            AppleAicSetTimerImask(AppleAicIntIdToTimer(Best), TRUE);
        }

        if (AppleAicSgiEligible(Cpu, Pmr))
            AppleAicRaiseIpi(Cpu);

        return Best;
    }

    if (!AppleAicHwReady)
        return APPLE_AIC_SPURIOUS_INTID;

    for (;;)
    {
        ULONG Event = AppleAicReadEvent();
        ULONG Irq;

        if (Event == 0)
            break;

        if (AIC_EVENT_TYPE(Event) != AIC_EVENT_TYPE_IRQ || AIC_EVENT_DIE(Event) != 0)
            continue;

        Irq = AIC_EVENT_NUM(Event);
        if (Irq >= AppleAicNrIrq)
            continue;

        if (!AppleAicHwIsEnabled(Irq))
            continue;

        if (AppleAicHwPriority[Irq] < Pmr)
            return APPLE_AIC_HW_INTID_BASE + Irq;

        AppleAicSetHwDeferred(Irq);
    }

    return APPLE_AIC_SPURIOUS_INTID;
}

static
VOID
AppleAicEndInterrupt(
    _In_ ULONG IntId)
{
    ULONG Cpu = AppleAicCurrentCpu();
    LONG Timer = AppleAicIntIdToTimer(IntId);

    if (Timer >= 0)
    {
        if ((ULONG)AppleAicTimerEnabled[Cpu] & (1u << Timer))
            AppleAicSetTimerImask(Timer, FALSE);
        return;
    }

    if (IntId >= APPLE_AIC_HW_INTID_BASE &&
        IntId < APPLE_AIC_HW_INTID_BASE + AppleAicNrIrq)
    {
        ULONG Irq = IntId - APPLE_AIC_HW_INTID_BASE;
        ULONG Bit = 1u << (Irq & 31);

        if (AppleAicHwIsEnabled(Irq) &&
            !((ULONG)AppleAicHwDeferred[Irq >> 5] & Bit))
        {
            AppleAicUnmaskHw(Irq);
        }
    }
}

static
VOID
AppleAicEnableInterrupt(
    _In_ ULONG IntId)
{
    LONG Timer = AppleAicIntIdToTimer(IntId);
    ULONGLONG Daif;

    if (Timer >= 0)
    {
        ULONG Cpu;

        Daif = AppleAicMaskInterrupts();
        Cpu = AppleAicCurrentCpu();
        InterlockedAnd(&AppleAicTimerDeferred[Cpu], (LONG)~(1u << Timer));
        InterlockedOr(&AppleAicTimerEnabled[Cpu], (LONG)(1u << Timer));
        AppleAicSetTimerImask(Timer, FALSE);
        AppleAicRestoreInterrupts(Daif);
        return;
    }

    if (IntId >= APPLE_AIC_HW_INTID_BASE &&
        IntId < APPLE_AIC_HW_INTID_BASE + APPLE_AIC_MAX_HW_IRQ)
    {
        ULONG Irq = IntId - APPLE_AIC_HW_INTID_BASE;

        InterlockedOr(&AppleAicHwEnabled[Irq >> 5], (LONG)(1u << (Irq & 31)));
        AppleAicClearHwDeferred(Irq);
        AppleAicUnmaskHw(Irq);
        return;
    }

    if (IntId >= APPLE_AIC_SGI_COUNT)
        DPRINT1("[AppleAIC] EnableInterrupt: INTID %lu not supported\n", IntId);
}

static
VOID
AppleAicDisableInterrupt(
    _In_ ULONG IntId)
{
    LONG Timer = AppleAicIntIdToTimer(IntId);
    ULONGLONG Daif;

    if (Timer >= 0)
    {
        ULONG Cpu;

        Daif = AppleAicMaskInterrupts();
        Cpu = AppleAicCurrentCpu();
        InterlockedAnd(&AppleAicTimerEnabled[Cpu], (LONG)~(1u << Timer));
        InterlockedAnd(&AppleAicTimerDeferred[Cpu], (LONG)~(1u << Timer));
        AppleAicSetTimerImask(Timer, TRUE);
        AppleAicRestoreInterrupts(Daif);
        return;
    }

    if (IntId >= APPLE_AIC_HW_INTID_BASE &&
        IntId < APPLE_AIC_HW_INTID_BASE + APPLE_AIC_MAX_HW_IRQ)
    {
        ULONG Irq = IntId - APPLE_AIC_HW_INTID_BASE;

        InterlockedAnd(&AppleAicHwEnabled[Irq >> 5], (LONG)~(1u << (Irq & 31)));
        AppleAicClearHwDeferred(Irq);
        AppleAicMaskHw(Irq);
    }
}

static
VOID
AppleAicSetInterruptPriority(
    _In_ ULONG IntId,
    _In_ ULONG Priority)
{
    LONG Timer;

    if (IntId < APPLE_AIC_SGI_COUNT)
    {
        AppleAicSgiPriority[IntId] = (UCHAR)Priority;
        return;
    }

    Timer = AppleAicIntIdToTimer(IntId);
    if (Timer >= 0)
    {
        AppleAicTimerPriority[Timer] = (UCHAR)Priority;
        return;
    }

    if (IntId >= APPLE_AIC_HW_INTID_BASE &&
        IntId < APPLE_AIC_HW_INTID_BASE + APPLE_AIC_MAX_HW_IRQ)
    {
        AppleAicHwPriority[IntId - APPLE_AIC_HW_INTID_BASE] = (UCHAR)Priority;
    }
}

static
VOID
AppleAicSetTriggerMode(
    _In_ ULONG IntId,
    _In_ BOOLEAN EdgeTriggered)
{
    UNREFERENCED_PARAMETER(IntId);
    UNREFERENCED_PARAMETER(EdgeTriggered);
}

static
VOID
AppleAicSetPriorityMask(
    _In_ KIRQL Irql)
{
    ULONG Pmr = AppleAicIrqlToPmr[(Irql <= HIGH_LEVEL) ? Irql : HIGH_LEVEL];
    ULONGLONG Daif;
    ULONG Cpu;

    Daif = AppleAicMaskInterrupts();
    Cpu = AppleAicCurrentCpu();

    AppleAicPmr[Cpu] = (UCHAR)Pmr;
    ARM64_DSB(SY);

    if (AppleAicTimerDeferred[Cpu] != 0)
    {
        LONG Timer;

        for (Timer = 0; Timer < APPLE_AIC_TIMER_COUNT; Timer++)
        {
            ULONG Bit = 1u << Timer;

            if (!((ULONG)AppleAicTimerDeferred[Cpu] & Bit) ||
                AppleAicTimerPriority[Timer] >= Pmr)
            {
                continue;
            }

            InterlockedAnd(&AppleAicTimerDeferred[Cpu], (LONG)~Bit);
            if ((ULONG)AppleAicTimerEnabled[Cpu] & Bit)
                AppleAicSetTimerImask(Timer, FALSE);
        }
    }

    if (AppleAicSgiPending[Cpu] != 0 && AppleAicSgiEligible(Cpu, Pmr))
        AppleAicRaiseIpi(Cpu);

    if (Cpu == APPLE_AIC_IRQ_TARGET_CPU && AppleAicHwDeferredCount != 0)
    {
        ULONG Word;

        for (Word = 0; Word < RTL_NUMBER_OF(AppleAicHwDeferred); Word++)
        {
            ULONG Deferred = (ULONG)AppleAicHwDeferred[Word];

            while (Deferred != 0)
            {
                ULONG Bit = Deferred & (0 - Deferred);
                ULONG Irq = Word * 32;
                ULONG Probe = Bit;

                while (Probe >>= 1)
                    Irq++;

                Deferred &= ~Bit;
                if (AppleAicHwPriority[Irq] >= Pmr)
                    continue;

                AppleAicClearHwDeferred(Irq);
                if (AppleAicHwIsEnabled(Irq))
                    AppleAicUnmaskHw(Irq);
            }
        }
    }

    AppleAicRestoreInterrupts(Daif);
}

static
ULONG
AppleAicGetPriorityMask(VOID)
{
    return AppleAicPmr[AppleAicCurrentCpu()];
}

static
VOID
AppleAicSendSgi(
    _In_ KAFFINITY TargetSet,
    _In_ ULONG SgiId)
{
    ULONG Cpu;

    if (SgiId >= APPLE_AIC_SGI_COUNT)
        return;

    for (Cpu = 0; Cpu < MAXIMUM_PROCESSORS && Cpu < 64; Cpu++)
    {
        if (TargetSet & ((KAFFINITY)1 << Cpu))
            InterlockedOr(&AppleAicSgiPending[Cpu], (LONG)(1u << SgiId));
    }

    ARM64_DSB(SY);

    for (Cpu = 0; Cpu < MAXIMUM_PROCESSORS && Cpu < 64; Cpu++)
    {
        if (TargetSet & ((KAFFINITY)1 << Cpu))
            AppleAicRaiseIpi(Cpu);
    }
}

static
VOID
AppleAicInitializeProcessor(
    _In_ ULONG ProcessorNumber)
{
    if (ProcessorNumber >= MAXIMUM_PROCESSORS)
        return;

    AppleAicCpuMpidr[ProcessorNumber] = ARM64_READ_SYSREG(MPIDR_EL1);
    AppleAicPmr[ProcessorNumber] = 0x00;
    AppleAicTimerDeferred[ProcessorNumber] = 0;

    ARM64_WRITE_SYSREG(S3_5_C15_C1_1, IPI_SR_PENDING);
    ARM64_ISB();
}

static
BOOLEAN
AppleAicPhase0Initialize(
    _In_opt_ PLOADER_PARAMETER_BLOCK LoaderBlock)
{
    ULONG MaxIrq;
    ULONG Words;
    ULONG Index;
    ULONG Offset;

    UNREFERENCED_PARAMETER(LoaderBlock);

    if (AppleAicVersion == 2)
    {
        AppleAicNrIrq = AppleAicRead(AIC2_INFO1) & AIC2_INFO1_NR_IRQ;
        MaxIrq = AppleAicRead(AIC2_INFO3) & AIC2_INFO3_MAX_IRQ;
        Offset = AIC2_IRQ_CFG + (sizeof(ULONG) * MaxIrq);
        AppleAicEventOffset = 0;
    }
    else
    {
        AppleAicNrIrq = AppleAicRead(AIC_INFO) & AIC_INFO_NR_IRQ;
        MaxIrq = AIC_MAX_IRQ;
        Offset = AIC_TARGET_CPU + (sizeof(ULONG) * MaxIrq);
        AppleAicEventOffset = AIC_EVENT;
    }

    if (AppleAicNrIrq > APPLE_AIC_MAX_HW_IRQ)
        AppleAicNrIrq = APPLE_AIC_MAX_HW_IRQ;

    Words = MaxIrq >> 5;
    AppleAicSwClr = Offset + (sizeof(ULONG) * Words);
    AppleAicMaskSet = AppleAicSwClr + (sizeof(ULONG) * Words);
    AppleAicMaskClr = AppleAicMaskSet + (sizeof(ULONG) * Words);

    Words = (AppleAicNrIrq + 31) >> 5;
    for (Index = 0; Index < Words; Index++)
    {
        AppleAicWrite(AppleAicMaskSet + (Index * 4), 0xFFFFFFFF);
        AppleAicWrite(AppleAicSwClr + (Index * 4), 0xFFFFFFFF);
    }

    if (AppleAicVersion == 1)
    {
        for (Index = 0; Index < AppleAicNrIrq; Index++)
            AppleAicWrite(AIC_TARGET_CPU + (Index * 4), 1u << APPLE_AIC_IRQ_TARGET_CPU);
    }
    else
    {
        AppleAicWrite(AIC2_CONFIG, AppleAicRead(AIC2_CONFIG) | AIC2_CONFIG_ENABLE);
    }

    ARM64_DSB(SY);

    AppleAicInitializeProcessor(AppleAicCurrentCpu());

    DPRINT1("[AppleAIC] AIC%lu initialized: %lu IRQs\n", AppleAicVersion, AppleAicNrIrq);

    return TRUE;
}

static
BOOLEAN
AppleAicPhase1MapRuntimeMmio(VOID)
{
    PHYSICAL_ADDRESS Pa;
    PVOID Va;
    ULONG Index;

    Pa.QuadPart = AppleAicBase;
    Va = MmMapIoSpace(Pa, (SIZE_T)AppleAicLength, MmNonCached);
    if (!Va)
        return FALSE;

    if (AppleAicVersion == 2)
    {
        PHYSICAL_ADDRESS EventPa;
        PVOID EventVa;

        EventPa.QuadPart = AppleAicEventBase;
        EventVa = MmMapIoSpace(EventPa, (SIZE_T)AppleAicEventLength, MmNonCached);
        if (!EventVa)
        {
            MmUnmapIoSpace(Va, (SIZE_T)AppleAicLength);
            return FALSE;
        }
        AppleAicEventBase = (ULONGLONG)(ULONG_PTR)EventVa;
    }

    AppleAicBase = (ULONGLONG)(ULONG_PTR)Va;
    if (AppleAicVersion != 2)
        AppleAicEventBase = AppleAicBase;

    ARM64_DSB(SY);
    AppleAicHwReady = TRUE;

    for (Index = 0; Index < AppleAicNrIrq; Index++)
    {
        if (AppleAicHwIsEnabled(Index))
            AppleAicUnmaskHw(Index);
    }

    return TRUE;
}

static const HAL_ARM64_INTERRUPT_CONTROLLER AppleAicInterruptOps =
{
    "Apple AIC",
    AppleAicAcknowledgeInterrupt,
    AppleAicEndInterrupt,
    AppleAicEnableInterrupt,
    AppleAicDisableInterrupt,
    AppleAicSetInterruptPriority,
    AppleAicSetTriggerMode,
    AppleAicSetPriorityMask,
    AppleAicGetPriorityMask,
    AppleAicSendSgi,
    AppleAicInitializeProcessor,
    AppleAicPhase0Initialize,
    AppleAicPhase1MapRuntimeMmio,
};

BOOLEAN
AppleAicInterruptProbe(
    _In_opt_ PLOADER_PARAMETER_BLOCK LoaderBlock)
{
    static PCSTR Compatibles[] = {"apple,aic2", "apple,t8103-aic"};
    static const ULONG Versions[] = {2, 1};
    PVOID Tree;
    ULONGLONG Midr;
    ULONG Index;

    if (AppleAicDetected)
        return TRUE;

    if (!LoaderBlock)
        return FALSE;

    Midr = ARM64_READ_SYSREG(MIDR_EL1);
    if (((Midr >> 24) & 0xFF) != APPLE_AIC_MIDR_IMPLEMENTER)
        return FALSE;

    Tree = (PVOID)(ULONG_PTR)LoaderBlock->u.Arm64.DeviceTree;
    if (!Tree || LoaderBlock->u.Arm64.DeviceTreeSize == 0)
        return FALSE;

    for (Index = 0; Index < RTL_NUMBER_OF(Compatibles); Index++)
    {
        if (Arm64FdtFindReg(Tree, Compatibles[Index], 0, &AppleAicBase, &AppleAicLength))
        {
            AppleAicVersion = Versions[Index];
            break;
        }
    }

    if (Index == RTL_NUMBER_OF(Compatibles) || AppleAicBase == 0 || AppleAicLength == 0)
        return FALSE;

    if (AppleAicVersion == 2)
    {
        if (!Arm64FdtFindReg(Tree, Compatibles[0], 1, &AppleAicEventBase, &AppleAicEventLength) ||
            AppleAicEventBase == 0 || AppleAicEventLength == 0)
        {
            return FALSE;
        }
    }
    else
    {
        AppleAicEventBase = AppleAicBase;
        AppleAicEventLength = AppleAicLength;
    }

    for (Index = 0; Index < APPLE_AIC_MAX_HW_IRQ; Index++)
        AppleAicHwPriority[Index] = APPLE_AIC_DEFAULT_PRIORITY;
    for (Index = 0; Index < APPLE_AIC_TIMER_COUNT; Index++)
        AppleAicTimerPriority[Index] = APPLE_AIC_DEFAULT_PRIORITY;

    AppleAicDetected = TRUE;
    HalpArm64RegisterInterruptController(&AppleAicInterruptOps);

    DPRINT1("[AppleAIC] AIC%lu at 0x%llx length 0x%llx\n",
            AppleAicVersion,
            AppleAicBase,
            AppleAicLength);

    return TRUE;
}
