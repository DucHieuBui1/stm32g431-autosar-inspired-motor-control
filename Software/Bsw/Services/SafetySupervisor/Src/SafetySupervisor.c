#include "SafetySupervisor.h"
#include "SafetySupervisor_Cfg.h"

#include "Adc.h"
#include "MotorControlCdd.h"
#include "Os.h"
#include "StateManager.h"

#define SAFETY_SUPERVISOR_MAILBOX_SLOT_COUNT        3U
#define SAFETY_SUPERVISOR_MAILBOX_FREE              0U
#define SAFETY_SUPERVISOR_MAILBOX_WRITING           1U
#define SAFETY_SUPERVISOR_MAILBOX_READY             2U
#define SAFETY_SUPERVISOR_MAILBOX_READING           3U

#ifndef SAFETY_SUPERVISOR_TEST_MAILBOX_HOOK
#define SAFETY_SUPERVISOR_TEST_MAILBOX_HOOK(state, slotIndex) \
    do                                                            \
    {                                                             \
        (void)(state);                                            \
        (void)(slotIndex);                                        \
    } while (0)
#endif

#ifndef SAFETY_SUPERVISOR_TEST_FAULT_READ_HOOK
#define SAFETY_SUPERVISOR_TEST_FAULT_READ_HOOK() ((void)0)
#endif

typedef enum
{
    SAFETY_SUPERVISOR_UNINITIALIZED = 0,
    SAFETY_SUPERVISOR_NEEDS_PRIME,
    SAFETY_SUPERVISOR_PENDING,
    SAFETY_SUPERVISOR_FAULTED
} SafetySupervisor_StateType;

typedef struct
{
    _Alignas(4) uint32 state;
    SafetySupervisor_SnapshotType payload;
} SafetySupervisor_MailboxSlotType;

_Static_assert(__atomic_always_lock_free(sizeof(uint32), 0),
               "Safety mailbox/event atomics must always be lock-free");
_Static_assert(_Alignof(SafetySupervisor_MailboxSlotType) >= 4U,
               "Safety mailbox slots must be naturally aligned");

STATIC SafetySupervisor_StateType SafetySupervisor_State;
STATIC SafetySupervisor_MailboxSlotType
    SafetySupervisor_Mailbox[SAFETY_SUPERVISOR_MAILBOX_SLOT_COUNT];
_Alignas(4) STATIC uint32 SafetySupervisor_FaultEvent;
_Alignas(4) STATIC uint32 SafetySupervisor_AliveCounter;
_Alignas(4) STATIC uint32 SafetySupervisor_DropCount;
STATIC uint32 SafetySupervisor_PendingStartTick;
STATIC uint32 SafetySupervisor_LastAcquisitionSequence;
STATIC boolean SafetySupervisor_SequenceEstablished;
STATIC SafetySupervisor_ConfigType SafetySupervisor_ActiveConfig;
STATIC boolean SafetySupervisor_ConfigValid;

STATIC void SafetySupervisor_PublishFault(uint16 cause)
{
    uint32 previous = __atomic_load_n(&SafetySupervisor_FaultEvent,
                                      __ATOMIC_RELAXED);
    uint32 sequence = (previous >> 16U) + 1U;
    uint32 event = (sequence << 16U) | (uint32)cause;

    MotorControlCdd_EmergencyOff();
    __atomic_store_n(&SafetySupervisor_FaultEvent, event, __ATOMIC_RELEASE);
    StateManager_NotifyCriticalFault(
        STATE_MANAGER_CRITICAL_CLASS_SAFETY);
    SafetySupervisor_State = SAFETY_SUPERVISOR_FAULTED;
}

STATIC void SafetySupervisor_PublishSnapshot(
    const SafetySupervisor_SnapshotType *snapshot)
{
    uint32 slotIndex;

    for (slotIndex = 0U;
         slotIndex < SAFETY_SUPERVISOR_MAILBOX_SLOT_COUNT;
         slotIndex++)
    {
        SafetySupervisor_MailboxSlotType *slot =
            &SafetySupervisor_Mailbox[slotIndex];
        uint32 expected = SAFETY_SUPERVISOR_MAILBOX_FREE;

        if (__atomic_compare_exchange_n(
                &slot->state, &expected,
                SAFETY_SUPERVISOR_MAILBOX_WRITING, FALSE,
                __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
        {
            SAFETY_SUPERVISOR_TEST_MAILBOX_HOOK(
                SAFETY_SUPERVISOR_MAILBOX_WRITING, slotIndex);
            slot->payload = *snapshot;
            __atomic_store_n(&slot->state,
                             SAFETY_SUPERVISOR_MAILBOX_READY,
                             __ATOMIC_RELEASE);
            return;
        }
    }

    (void)__atomic_fetch_add(&SafetySupervisor_DropCount, 1U,
                             __ATOMIC_RELAXED);
}

Std_ReturnType SafetySupervisor_Init(void)
{
    uint32 slotIndex;

    if (SafetySupervisor_State != SAFETY_SUPERVISOR_UNINITIALIZED)
    {
        return E_NOT_OK;
    }

    if (SafetySupervisor_CfgGet(&SafetySupervisor_ActiveConfig) != E_OK)
    {
        SafetySupervisor_ConfigValid = FALSE;
        SafetySupervisor_PublishFault(SAFETY_SUPERVISOR_EVENT_CAUSE_INTERNAL);
        return E_NOT_OK;
    }

    for (slotIndex = 0U;
         slotIndex < SAFETY_SUPERVISOR_MAILBOX_SLOT_COUNT;
         slotIndex++)
    {
        __atomic_store_n(&SafetySupervisor_Mailbox[slotIndex].state,
                         SAFETY_SUPERVISOR_MAILBOX_FREE,
                         __ATOMIC_RELAXED);
    }

    __atomic_store_n(&SafetySupervisor_FaultEvent, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SafetySupervisor_AliveCounter, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SafetySupervisor_DropCount, 0U, __ATOMIC_RELAXED);
    SafetySupervisor_PendingStartTick = 0U;
    SafetySupervisor_LastAcquisitionSequence = 0U;
    SafetySupervisor_SequenceEstablished = FALSE;
    SafetySupervisor_ConfigValid = TRUE;
    SafetySupervisor_State = SAFETY_SUPERVISOR_NEEDS_PRIME;
    return E_OK;
}

void SafetySupervisor_MainFunction(void)
{
    Adc_SafetyGroupResultType adcResult;
    Adc_SafetyGroupStatusType adcStatus;
    uint32 currentTick;

    if (SafetySupervisor_State == SAFETY_SUPERVISOR_UNINITIALIZED)
    {
        return;
    }

    (void)__atomic_fetch_add(&SafetySupervisor_AliveCounter, 1U,
                             __ATOMIC_RELEASE);

    if (SafetySupervisor_State == SAFETY_SUPERVISOR_FAULTED)
    {
        return;
    }

    currentTick = Os_GetCounterValue();
    if (SafetySupervisor_State == SAFETY_SUPERVISOR_NEEDS_PRIME)
    {
        if (Adc_StartSafetyGroup() != E_OK)
        {
            SafetySupervisor_PublishFault(
                SAFETY_SUPERVISOR_EVENT_CAUSE_START_FAILED);
            return;
        }

        SafetySupervisor_PendingStartTick = currentTick;
        SafetySupervisor_State = SAFETY_SUPERVISOR_PENDING;
        return;
    }

    adcStatus = Adc_PollSafetyGroup(&adcResult);
    if (adcStatus == ADC_SAFETY_GROUP_PENDING)
    {
        if ((uint32)(currentTick - SafetySupervisor_PendingStartTick) >= 1U)
        {
            SafetySupervisor_PublishFault(
                SAFETY_SUPERVISOR_EVENT_CAUSE_TIMEOUT);
        }
        return;
    }

    if (adcStatus != ADC_SAFETY_GROUP_COMPLETED)
    {
        SafetySupervisor_PublishFault(
            SAFETY_SUPERVISOR_EVENT_CAUSE_ADC_FAILED);
        return;
    }

    if (((SafetySupervisor_SequenceEstablished != TRUE) &&
         (adcResult.acquisitionSequence == 0U)) ||
        ((SafetySupervisor_SequenceEstablished == TRUE) &&
         (adcResult.acquisitionSequence !=
          (SafetySupervisor_LastAcquisitionSequence + 1U))))
    {
        SafetySupervisor_PublishFault(
            SAFETY_SUPERVISOR_EVENT_CAUSE_SEQUENCE);
        return;
    }

    {
        SafetySupervisor_SnapshotType snapshot;
        SafetySupervisor_ConfigType config;
        uint32 generationId;

        if ((SafetySupervisor_CfgGet(&config) != E_OK) ||
            (MotorControlCdd_GetCurrentGenerationId(&generationId) != E_OK))
        {
            SafetySupervisor_PublishFault(
                SAFETY_SUPERVISOR_EVENT_CAUSE_INTERNAL);
            return;
        }

        snapshot.vbusRaw = adcResult.vbusRaw;
        snapshot.temperatureRaw = adcResult.temperatureRaw;
        snapshot.acquisitionSequence = adcResult.acquisitionSequence;
        snapshot.osTick = currentTick;
        snapshot.pwmGenerationId = generationId;
        snapshot.acquisitionValid = TRUE;
        snapshot.limitsValid =
            ((config.measurementApproved == TRUE) &&
             (adcResult.vbusRaw >= config.vbusRawMinimum) &&
             (adcResult.vbusRaw <= config.vbusRawMaximum) &&
             (adcResult.temperatureRaw >= config.temperatureRawMinimum) &&
             (adcResult.temperatureRaw <= config.temperatureRawMaximum)) ?
            TRUE : FALSE;
        SafetySupervisor_ActiveConfig = config;
        SafetySupervisor_ConfigValid = TRUE;
        SafetySupervisor_PublishSnapshot(&snapshot);
        {
            MotorControlCdd_FastSafetySnapshotType fastSnapshot;

            fastSnapshot.vbusRaw = snapshot.vbusRaw;
            fastSnapshot.temperatureRaw = snapshot.temperatureRaw;
            fastSnapshot.acquisitionSequence = snapshot.acquisitionSequence;
            fastSnapshot.osTick = snapshot.osTick;
            fastSnapshot.pwmGenerationId = snapshot.pwmGenerationId;
            fastSnapshot.acquisitionValid = snapshot.acquisitionValid;
            fastSnapshot.limitsValid = snapshot.limitsValid;
            fastSnapshot.structuralValid = TRUE;
            fastSnapshot.measurementApproved = config.measurementApproved;
            fastSnapshot.identity = config.identity;
            if (MotorControlCdd_PublishSafetySnapshot(&fastSnapshot) != E_OK)
            {
                SafetySupervisor_PublishFault(
                    SAFETY_SUPERVISOR_EVENT_CAUSE_INTERNAL);
                return;
            }
        }
    }

    SafetySupervisor_LastAcquisitionSequence = adcResult.acquisitionSequence;
    SafetySupervisor_SequenceEstablished = TRUE;

    if (Adc_StartSafetyGroup() != E_OK)
    {
        SafetySupervisor_PublishFault(
            SAFETY_SUPERVISOR_EVENT_CAUSE_START_FAILED);
        return;
    }

    SafetySupervisor_PendingStartTick = currentTick;
}

Std_ReturnType SafetySupervisor_GetLatestSnapshot(
    SafetySupervisor_SnapshotType *Snapshot)
{
    SafetySupervisor_SnapshotType newestSnapshot;
    boolean snapshotFound = FALSE;
    uint32 slotIndex;

    if (Snapshot == NULL_PTR)
    {
        return E_NOT_OK;
    }

    for (slotIndex = 0U;
         slotIndex < SAFETY_SUPERVISOR_MAILBOX_SLOT_COUNT;
         slotIndex++)
    {
        SafetySupervisor_MailboxSlotType *slot =
            &SafetySupervisor_Mailbox[slotIndex];
        uint32 expected = SAFETY_SUPERVISOR_MAILBOX_READY;

        if (__atomic_compare_exchange_n(
                &slot->state, &expected,
                SAFETY_SUPERVISOR_MAILBOX_READING, FALSE,
                __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
        {
            SafetySupervisor_SnapshotType candidate;

            SAFETY_SUPERVISOR_TEST_MAILBOX_HOOK(
                SAFETY_SUPERVISOR_MAILBOX_READING, slotIndex);
            candidate = slot->payload;
            __atomic_store_n(&slot->state,
                             SAFETY_SUPERVISOR_MAILBOX_FREE,
                             __ATOMIC_RELEASE);
            if ((snapshotFound != TRUE) ||
                (((uint32)(candidate.acquisitionSequence -
                           newestSnapshot.acquisitionSequence) != 0U) &&
                 ((uint32)(candidate.acquisitionSequence -
                           newestSnapshot.acquisitionSequence) <
                  0x80000000UL)))
            {
                newestSnapshot = candidate;
                snapshotFound = TRUE;
            }
        }
    }

    if (snapshotFound != TRUE)
    {
        return E_NOT_OK;
    }

    *Snapshot = newestSnapshot;
    return E_OK;
}

Std_ReturnType SafetySupervisor_GetReadiness(
    SafetySupervisor_ReadinessType *Readiness)
{
    SafetySupervisor_ConfigType config;

    if (Readiness == NULL_PTR)
    {
        return E_NOT_OK;
    }
    *Readiness = (SafetySupervisor_ReadinessType){ 0 };
    if (SafetySupervisor_CfgGet(&config) != E_OK)
    {
        return E_OK;
    }
    Readiness->identity = config.identity;
    Readiness->structuralValid = TRUE;
    Readiness->measurementApproved = config.measurementApproved;
    return E_OK;
}

boolean SafetySupervisor_IsSnapshotReady(
    const SafetySupervisor_SnapshotType *Snapshot,
    uint32 CurrentTick)
{
    if ((Snapshot != NULL_PTR) &&
        (SafetySupervisor_ConfigValid == TRUE) &&
        (Snapshot->acquisitionValid == TRUE) &&
        (Snapshot->limitsValid == TRUE) &&
        ((uint32)(CurrentTick - Snapshot->osTick) <=
         SafetySupervisor_ActiveConfig.readySnapshotMaximumAgeMs))
    {
        return TRUE;
    }
    return FALSE;
}

uint32 SafetySupervisor_GetFaultEvent(void)
{
    uint32 event = __atomic_load_n(&SafetySupervisor_FaultEvent,
                                   __ATOMIC_ACQUIRE);

    SAFETY_SUPERVISOR_TEST_FAULT_READ_HOOK();
    return event;
}

uint32 SafetySupervisor_GetAliveCounter(void)
{
    return __atomic_load_n(&SafetySupervisor_AliveCounter, __ATOMIC_ACQUIRE);
}

Std_ReturnType SafetySupervisor_GetDropCount(uint32 *DropCount)
{
    if (DropCount == NULL_PTR)
    {
        return E_NOT_OK;
    }

    *DropCount = __atomic_load_n(&SafetySupervisor_DropCount,
                                 __ATOMIC_ACQUIRE);
    return E_OK;
}
