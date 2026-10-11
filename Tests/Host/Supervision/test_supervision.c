#include <limits.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "Adc.h"
#include "Mcu.h"
#include "MotorControlCdd.h"
#include "Os.h"
#include "Os_Cfg.h"
#include "SafetySupervisor.h"
#include "SafetySupervisor_Cfg.h"
#include "SerialCommunicationCdd.h"
#include "StateManager.h"
#include "Wdg.h"
#include "WdgM.h"

static Std_ReturnType Test_AdcStartReturn;
static Adc_SafetyGroupStatusType Test_AdcPollStatus;
static Adc_SafetyGroupResultType Test_AdcResult;
static uint32 Test_Tick;
static boolean Test_OsRunning;
static uint32 Test_GenerationId;
static Std_ReturnType Test_GenerationLookupReturn;
static Std_ReturnType Test_FastSafetyPublishReturn;
static uint32 Test_FastSafetyPublishCount;
static MotorControlCdd_FastSafetySnapshotType Test_LastFastSafetySnapshot;
static MotorControlCdd_FaultEventsType Test_MotorEvents;
static uint32 Test_ClockEvents;
static uint32 Test_EmergencyOffCount;
static uint32 Test_WdgTriggerCount;
static boolean Test_ReadWhileWriting;
static boolean Test_PublishWhileReading;
static Std_ReturnType Test_ConcurrentReadResult;
static boolean Test_SerialRequestAvailable;
static SerialCommunicationCdd_RequestType Test_SerialRequest;
static SerialCommunicationCdd_ResponseType Test_SerialResponse;
static SerialCommunicationCdd_DiagnosticsType Test_SerialDiagnostics;
static uint32 Test_SerialMainCount;
static uint32 Test_SerialReleaseCount;
static uint32 Test_SerialResponseCount;
static boolean Test_CalibrationActive;
static boolean Test_CalibrationComplete;
static boolean Test_CalibrationCleanupPending;
static Std_ReturnType Test_CalibrationStartReturn;
static Std_ReturnType Test_CalibrationStepReturn;
static MotorControlCdd_CalibrationAbortModeType Test_LastAbortMode;
static uint32 Test_CalibrationStartCount;
static uint32 Test_CalibrationStepCount;
static uint32 Test_CalibrationCleanupCount;
static uint32 Test_CalibrationCleanupSubstate;
static uint32 Test_LastCleanupSubstate;
static uint32 Test_CalibrationActiveSubstate;
static uint32 Test_AbortObservedSubstate;
static MotorControlCdd_CalibrationEnvironmentType Test_LastEnvironment;
static boolean Test_SafetyConfigApproved;
static boolean Test_MotorReadinessApproved;
static boolean Test_MotorOffsetsInstalled;
static boolean Test_CompleteCalibrationOnStep;
static boolean Test_MotorIdentityMismatch;
static boolean Test_MotorReadinessStructuralInvalid;
static boolean Test_MotorReadinessCallFails;
static boolean Test_ResultCallFails;
static boolean Test_ResultInvalid;
static boolean Test_ResultApprovalMismatch;
static boolean Test_ResultOffsetsMismatch;
static boolean Test_ResultIdentityMismatch;
static uint32 Test_ClockReadCount;
static uint32 Test_ClockEventOnRead;
static uint32 Test_CalibrationHookStage;
static uint32 Test_CalibrationHookClass;
static uint32 Test_FaultReadInjectionStage;
static uint32 Test_FaultReadInjectionOccurrence;
static uint32 Test_TransitionHookStage;
static uint32 Test_TransitionHookClass;
static uint32 Test_FaultCasArrivalMask[6U];
static boolean Test_FaultCasCorruptEveryAttempt;
static uint32 Test_FaultCasHookCount;
static boolean Test_FatalJumpArmed;
static boolean Test_FatalObserved;
static jmp_buf Test_FatalJump;

#define TEST_FAULT_READ_MOTOR_BREAK       1U
#define TEST_FAULT_READ_MOTOR_UPDATE      2U
#define TEST_FAULT_READ_MOTOR_ADC         3U
#define TEST_FAULT_READ_MOTOR_CALIBRATION 4U
#define TEST_FAULT_READ_CLOCK             5U
#define TEST_FAULT_READ_SAFETY            6U
#define TEST_FAULT_READ_WDGM              7U

#define TEST_CALIBRATION_SUBSTATE_ACQUIRE         1U
#define TEST_CALIBRATION_SUBSTATE_RESTORE_REQUEST 2U
#define TEST_CALIBRATION_SUBSTATE_RESTORE_VERIFY  3U

static void Test_MailboxHook(uint32 state, uint32 slotIndex);
static void Test_SafetyConfigHook(SafetySupervisor_ConfigType *config);
static void Test_StateCalibrationHook(uint32 stage);
static void Test_FaultReadHook(uint32 stage);
static void Test_TransitionHook(uint32 stage);
static void Test_FaultCasHook(uint32 attempt, uint32 observed);
static void Test_FatalHook(void);
static void Test_PublishCriticalClass(uint32 criticalClass);

#define __atomic_always_lock_free(size, pointer) 1
#define SAFETY_SUPERVISOR_TEST_MAILBOX_HOOK(state, slotIndex) \
    Test_MailboxHook((state), (slotIndex))
#define SAFETY_SUPERVISOR_CFG_TEST_HOOK(Config) Test_SafetyConfigHook(Config)
#define SAFETY_SUPERVISOR_TEST_FAULT_READ_HOOK() \
    Test_FaultReadHook(TEST_FAULT_READ_SAFETY)
#define WDGM_TEST_FAULT_READ_HOOK() \
    Test_FaultReadHook(TEST_FAULT_READ_WDGM)
#define STATE_MANAGER_TEST_CALIBRATION_HOOK(stage) \
    Test_StateCalibrationHook((stage))
#define STATE_MANAGER_TEST_TRANSITION_HOOK(stage) \
    Test_TransitionHook((stage))
#define STATE_MANAGER_TEST_FAULT_CAS_HOOK(attempt, observed) \
    Test_FaultCasHook((attempt), (observed))
#define STATE_MANAGER_TEST_FATAL_HOOK() Test_FatalHook()
#include "../../../Software/Bsw/Services/SafetySupervisor/Src/SafetySupervisor_Cfg.c"
#include "../../../Software/Bsw/Services/SafetySupervisor/Src/SafetySupervisor.c"
#include "../../../Software/Bsw/Services/WdgM/Src/WdgM.c"
#include "../../../Software/Asw/CommandArbitration/Src/CommandArbitration.c"
#include "../../../Software/Rte/Src/Rte_CommandPolicy.c"
#include "../../../Software/Bsw/Services/StateManager/Src/StateManager.c"

static void Test_SafetyConfigHook(SafetySupervisor_ConfigType *config)
{
    if (Test_SafetyConfigApproved == TRUE)
    {
        config->measurementApproved = TRUE;
        config->vbusRawMinimum = 100U;
        config->vbusRawMaximum = 4000U;
        config->temperatureRawMinimum = 100U;
        config->temperatureRawMaximum = 4000U;
        config->identity.evidenceSha256[0] = 1U;
    }
}

static void Test_StateCalibrationHook(uint32 stage)
{
    if (stage == Test_CalibrationHookStage)
    {
        Test_PublishCriticalClass(Test_CalibrationHookClass);
        Test_CalibrationHookStage = 0U;
    }
}

static uint32 Test_NextDetailedEvent(uint32 previous, uint16 cause)
{
    uint32 sequence = (previous >> 16U) + 1U;

    return (sequence << 16U) | (uint32)cause;
}

static void Test_PublishCriticalClass(uint32 criticalClass)
{
    if ((criticalClass & STATE_MANAGER_CRITICAL_CLASS_CLOCK) != 0U)
    {
        Test_ClockEvents |= MCU_CLOCK_FAULT_EVENT_CSS;
    }
    if ((criticalClass & STATE_MANAGER_CRITICAL_CLASS_MOTOR) != 0U)
    {
        if (Test_FaultReadInjectionStage ==
            TEST_FAULT_READ_MOTOR_UPDATE)
        {
            Test_MotorEvents.updateEvent = Test_NextDetailedEvent(
                Test_MotorEvents.updateEvent,
                MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_UPDATE);
        }
        else if (Test_FaultReadInjectionStage ==
                 TEST_FAULT_READ_MOTOR_ADC)
        {
            Test_MotorEvents.adcEvent = Test_NextDetailedEvent(
                Test_MotorEvents.adcEvent,
                MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC);
        }
        else if (Test_FaultReadInjectionStage ==
                 TEST_FAULT_READ_MOTOR_CALIBRATION)
        {
            Test_MotorEvents.calibrationEvent = Test_NextDetailedEvent(
                Test_MotorEvents.calibrationEvent,
                MOTOR_CONTROL_CDD_EVENT_CAUSE_CALIBRATION);
        }
        else
        {
            Test_MotorEvents.breakEvent = Test_NextDetailedEvent(
                Test_MotorEvents.breakEvent,
                MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK);
        }
    }
    if ((criticalClass & STATE_MANAGER_CRITICAL_CLASS_SAFETY) != 0U)
    {
        SafetySupervisor_FaultEvent = Test_NextDetailedEvent(
            SafetySupervisor_FaultEvent,
            SAFETY_SUPERVISOR_EVENT_CAUSE_INTERNAL);
    }
    if ((criticalClass & STATE_MANAGER_CRITICAL_CLASS_WDGM) != 0U)
    {
        WdgM_FaultEvent = Test_NextDetailedEvent(
            WdgM_FaultEvent, WDGM_EVENT_CAUSE_OS_TIME);
    }
    StateManager_NotifyCriticalFault(criticalClass);
}

static void Test_FaultReadHook(uint32 stage)
{
    if (stage == Test_FaultReadInjectionStage)
    {
        if (Test_FaultReadInjectionOccurrence > 1U)
        {
            Test_FaultReadInjectionOccurrence--;
            return;
        }
        if ((stage >= TEST_FAULT_READ_MOTOR_BREAK) &&
            (stage <= TEST_FAULT_READ_MOTOR_CALIBRATION))
        {
            Test_PublishCriticalClass(
                STATE_MANAGER_CRITICAL_CLASS_MOTOR);
        }
        else if (stage == TEST_FAULT_READ_CLOCK)
        {
            Test_PublishCriticalClass(
                STATE_MANAGER_CRITICAL_CLASS_CLOCK);
        }
        else if (stage == TEST_FAULT_READ_SAFETY)
        {
            Test_PublishCriticalClass(
                STATE_MANAGER_CRITICAL_CLASS_SAFETY);
        }
        else if (stage == TEST_FAULT_READ_WDGM)
        {
            Test_PublishCriticalClass(
                STATE_MANAGER_CRITICAL_CLASS_WDGM);
        }
        else
        {
            /* No injection for an unknown stage. */
        }
        Test_FaultReadInjectionStage = 0U;
        Test_FaultReadInjectionOccurrence = 1U;
    }
}

static void Test_TransitionHook(uint32 stage)
{
    if (stage == Test_TransitionHookStage)
    {
        Test_PublishCriticalClass(Test_TransitionHookClass);
        Test_TransitionHookStage = 0U;
    }
}

static void Test_FaultCasHook(uint32 attempt, uint32 observed)
{
    (void)observed;
    Test_FaultCasHookCount++;
    if ((attempt < 6U) && (Test_FaultCasArrivalMask[attempt] != 0U))
    {
        StateManager_NotifyCriticalFault(
            Test_FaultCasArrivalMask[attempt]);
    }
    if ((attempt < 6U) &&
        (Test_FaultCasCorruptEveryAttempt == TRUE))
    {
        (void)__atomic_fetch_or(&StateManager_State,
                                (uint32)1U << (8U + attempt),
                                __ATOMIC_RELEASE);
    }
}

static void Test_FatalHook(void)
{
    Test_FatalObserved = TRUE;
    if (Test_FatalJumpArmed == TRUE)
    {
        longjmp(Test_FatalJump, 1);
    }
}

#define TEST_CHECK(condition)                                             \
    do                                                                    \
    {                                                                     \
        if (!(condition))                                                 \
        {                                                                 \
            (void)printf("FAIL line %d: %s\n", __LINE__, #condition);     \
            return 1;                                                     \
        }                                                                 \
    } while (0)

static void Test_ResetFixture(void)
{
    Test_AdcStartReturn = E_OK;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_PENDING;
    (void)memset(&Test_AdcResult, 0, sizeof(Test_AdcResult));
    Test_Tick = 0U;
    Test_OsRunning = TRUE;
    Test_GenerationId = 0U;
    Test_GenerationLookupReturn = E_OK;
    Test_FastSafetyPublishReturn = E_OK;
    Test_FastSafetyPublishCount = 0U;
    Test_LastFastSafetySnapshot =
        (MotorControlCdd_FastSafetySnapshotType){ 0 };
    (void)memset(&Test_MotorEvents, 0, sizeof(Test_MotorEvents));
    Test_ClockEvents = MCU_CLOCK_FAULT_EVENT_NONE;
    Test_EmergencyOffCount = 0U;
    Test_WdgTriggerCount = 0U;
    Test_ReadWhileWriting = FALSE;
    Test_PublishWhileReading = FALSE;
    Test_ConcurrentReadResult = E_OK;
    Test_SerialRequestAvailable = FALSE;
    (void)memset(&Test_SerialRequest, 0, sizeof(Test_SerialRequest));
    (void)memset(&Test_SerialResponse, 0, sizeof(Test_SerialResponse));
    (void)memset(&Test_SerialDiagnostics, 0,
                 sizeof(Test_SerialDiagnostics));
    Test_SerialMainCount = 0U;
    Test_SerialReleaseCount = 0U;
    Test_SerialResponseCount = 0U;
    Test_CalibrationActive = FALSE;
    Test_CalibrationComplete = FALSE;
    Test_CalibrationCleanupPending = FALSE;
    Test_CalibrationStartReturn = E_OK;
    Test_CalibrationStepReturn = E_OK;
    Test_LastAbortMode =
        MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP;
    Test_CalibrationStartCount = 0U;
    Test_CalibrationStepCount = 0U;
    Test_CalibrationCleanupCount = 0U;
    Test_CalibrationCleanupSubstate = 0U;
    Test_LastCleanupSubstate = UINT_MAX;
    Test_CalibrationActiveSubstate = 0U;
    Test_AbortObservedSubstate = UINT_MAX;
    (void)memset(&Test_LastEnvironment, 0, sizeof(Test_LastEnvironment));
    Test_SafetyConfigApproved = FALSE;
    Test_MotorReadinessApproved = FALSE;
    Test_MotorOffsetsInstalled = FALSE;
    Test_CompleteCalibrationOnStep = FALSE;
    Test_MotorIdentityMismatch = FALSE;
    Test_MotorReadinessStructuralInvalid = FALSE;
    Test_MotorReadinessCallFails = FALSE;
    Test_ResultCallFails = FALSE;
    Test_ResultInvalid = FALSE;
    Test_ResultApprovalMismatch = FALSE;
    Test_ResultOffsetsMismatch = FALSE;
    Test_ResultIdentityMismatch = FALSE;
    Test_ClockReadCount = 0U;
    Test_ClockEventOnRead = 0U;
    Test_CalibrationHookStage = 0U;
    Test_CalibrationHookClass = STATE_MANAGER_CRITICAL_CLASS_MOTOR;
    Test_FaultReadInjectionStage = 0U;
    Test_FaultReadInjectionOccurrence = 1U;
    Test_TransitionHookStage = 0U;
    Test_TransitionHookClass = STATE_MANAGER_CRITICAL_CLASS_MOTOR;
    (void)memset(Test_FaultCasArrivalMask, 0,
                 sizeof(Test_FaultCasArrivalMask));
    Test_FaultCasCorruptEveryAttempt = FALSE;
    Test_FaultCasHookCount = 0U;
    Test_FatalJumpArmed = FALSE;
    Test_FatalObserved = FALSE;

    SafetySupervisor_State = SAFETY_SUPERVISOR_UNINITIALIZED;
    (void)memset(SafetySupervisor_Mailbox, 0,
                 sizeof(SafetySupervisor_Mailbox));
    SafetySupervisor_FaultEvent = 0U;
    SafetySupervisor_AliveCounter = 0U;
    SafetySupervisor_DropCount = 0U;
    SafetySupervisor_PendingStartTick = 0U;
    SafetySupervisor_LastAcquisitionSequence = 0U;
    SafetySupervisor_SequenceEstablished = FALSE;
    SafetySupervisor_ActiveConfig = (SafetySupervisor_ConfigType){ 0 };
    SafetySupervisor_ConfigValid = FALSE;

    StateManager_State = (uint32)STATE_MANAGER_RESET;
    StateManager_FaultMask = 0U;
    StateManager_ClassifiedPendingMask = 0U;
    StateManager_AliveCounter = 0U;
    StateManager_LastMotorBreakSequence = 0U;
    StateManager_LastMotorUpdateSequence = 0U;
    StateManager_LastMotorAdcSequence = 0U;
    StateManager_LastMotorCalibrationSequence = 0U;
    StateManager_LastSafetySequence = 0U;
    StateManager_LastWdgMSequence = 0U;
    StateManager_CalibrationEntryTick = 0U;
    StateManager_CalibrationActivation = 0U;
    StateManager_CalibrationDeadlineArmed = FALSE;
    StateManager_CalibrationStarted = FALSE;
    StateManager_LatestSafetySnapshot =
        (SafetySupervisor_SnapshotType){ 0 };
    StateManager_LatestSafetySnapshotValid = FALSE;

    WdgM_State = WDGM_UNINITIALIZED;
    WdgM_PreparedToken = 0U;
    WdgM_LastTick = 0U;
    WdgM_LastSafetyAlive = 0U;
    WdgM_LastStateAlive = 0U;
    WdgM_FaultEvent = 0U;
}

static int Test_InitializeSupervision(void)
{
    TEST_CHECK(StateManager_InitBootSafe() == E_OK);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_BOOT_SAFE);
    TEST_CHECK(SafetySupervisor_Init() == E_OK);
    TEST_CHECK(StateManager_EnterSelfTest() == E_OK);
    return 0;
}

Std_ReturnType Adc_StartSafetyGroup(void)
{
    if (Test_AdcStartReturn == E_OK)
    {
        Test_AdcPollStatus = ADC_SAFETY_GROUP_PENDING;
    }
    return Test_AdcStartReturn;
}

Adc_SafetyGroupStatusType Adc_PollSafetyGroup(
    Adc_SafetyGroupResultType *Result)
{
    if ((Test_AdcPollStatus == ADC_SAFETY_GROUP_COMPLETED) &&
        (Result != NULL_PTR))
    {
        *Result = Test_AdcResult;
    }
    return Test_AdcPollStatus;
}

Std_ReturnType MotorControlCdd_GetCurrentGenerationId(uint32 *GenerationId)
{
    if ((GenerationId == NULL_PTR) ||
        (Test_GenerationLookupReturn != E_OK))
    {
        return E_NOT_OK;
    }
    *GenerationId = Test_GenerationId;
    return E_OK;
}

Std_ReturnType MotorControlCdd_PublishSafetySnapshot(
    const MotorControlCdd_FastSafetySnapshotType *Snapshot)
{
    if ((Snapshot == NULL_PTR) || (Test_FastSafetyPublishReturn != E_OK))
    {
        return E_NOT_OK;
    }
    Test_LastFastSafetySnapshot = *Snapshot;
    Test_FastSafetyPublishCount++;
    return E_OK;
}

Std_ReturnType MotorControlCdd_GetFaultEvents(
    MotorControlCdd_FaultEventsType *Events)
{
    if (Events == NULL_PTR)
    {
        return E_NOT_OK;
    }
    Events->breakEvent = Test_MotorEvents.breakEvent;
    Test_FaultReadHook(TEST_FAULT_READ_MOTOR_BREAK);
    Events->updateEvent = Test_MotorEvents.updateEvent;
    Test_FaultReadHook(TEST_FAULT_READ_MOTOR_UPDATE);
    Events->adcEvent = Test_MotorEvents.adcEvent;
    Test_FaultReadHook(TEST_FAULT_READ_MOTOR_ADC);
    Events->calibrationEvent = Test_MotorEvents.calibrationEvent;
    Test_FaultReadHook(TEST_FAULT_READ_MOTOR_CALIBRATION);
    Events->controlEvent = Test_MotorEvents.controlEvent;
    return E_OK;
}

Std_ReturnType MotorControlCdd_StartCalibration(
    const MotorControlCdd_CalibrationEnvironmentType *Environment)
{
    if ((Environment == NULL_PTR) ||
        (Test_CalibrationStartReturn != E_OK))
    {
        return E_NOT_OK;
    }
    Test_LastEnvironment = *Environment;
    Test_CalibrationStartCount++;
    Test_CalibrationActive = TRUE;
    return E_OK;
}

Std_ReturnType MotorControlCdd_CalibrationStep(
    const MotorControlCdd_CalibrationEnvironmentType *Environment)
{
    if ((Environment == NULL_PTR) ||
        (Test_CalibrationStepReturn != E_OK) ||
        (Test_CalibrationActive != TRUE))
    {
        return E_NOT_OK;
    }
    Test_LastEnvironment = *Environment;
    Test_CalibrationStepCount++;
    if (Test_CompleteCalibrationOnStep == TRUE)
    {
        Test_CalibrationActive = FALSE;
        Test_CalibrationComplete = TRUE;
    }
    return E_OK;
}

Std_ReturnType MotorControlCdd_RequestCalibrationAbort(
    MotorControlCdd_CalibrationAbortModeType Mode)
{
    Test_LastAbortMode = Mode;
    Test_AbortObservedSubstate = Test_CalibrationActiveSubstate;
    Test_CalibrationActive = FALSE;
    Test_CalibrationCleanupPending =
        (Mode == MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP) ?
        TRUE : FALSE;
    return E_OK;
}

Std_ReturnType MotorControlCdd_CalibrationCleanupStep(void)
{
    if (Test_CalibrationCleanupPending != TRUE)
    {
        return E_NOT_OK;
    }
    Test_LastCleanupSubstate = Test_CalibrationCleanupSubstate;
    Test_CalibrationCleanupCount++;
    Test_CalibrationCleanupSubstate++;
    if (Test_CalibrationCleanupSubstate >= 4U)
    {
        Test_CalibrationCleanupPending = FALSE;
    }
    return E_OK;
}

Std_ReturnType MotorControlCdd_GetCalibrationResult(
    MotorControlCdd_CalibrationResultType *Result)
{
    if ((Result == NULL_PTR) || (Test_CalibrationComplete != TRUE) ||
        (Test_ResultCallFails == TRUE))
    {
        return E_NOT_OK;
    }
    *Result = (MotorControlCdd_CalibrationResultType){ 0 };
    Result->identity = Test_LastEnvironment.identity;
    Result->lastSafetySequence =
        Test_LastEnvironment.safetyAcquisitionSequence;
    Result->lastSafetyTick = Test_LastEnvironment.safetyTick;
    Result->measurementApproved = Test_MotorReadinessApproved;
    Result->offsetsInstalled = Test_MotorOffsetsInstalled;
    Result->valid = (Test_ResultInvalid == TRUE) ? FALSE : TRUE;
    if (Test_ResultApprovalMismatch == TRUE)
    {
        Result->measurementApproved = FALSE;
    }
    if (Test_ResultOffsetsMismatch == TRUE)
    {
        Result->offsetsInstalled = FALSE;
    }
    if (Test_ResultIdentityMismatch == TRUE)
    {
        Result->identity.evidenceSha256[1] = 1U;
    }
    return E_OK;
}

Std_ReturnType MotorControlCdd_GetCalibrationReadiness(
    MotorControlCdd_CalibrationReadinessType *Readiness)
{
    if ((Readiness == NULL_PTR) ||
        (Test_MotorReadinessCallFails == TRUE))
    {
        return E_NOT_OK;
    }
    *Readiness = (MotorControlCdd_CalibrationReadinessType){ 0 };
    Readiness->identity.schemaVersion = SC1_MEASUREMENT_SCHEMA_VERSION;
    Readiness->identity.boardProfileCode =
        SC1_MEASUREMENT_BOARD_PROFILE_CODE;
    if (Test_MotorReadinessApproved == TRUE)
    {
        Readiness->identity.evidenceSha256[0] = 1U;
    }
    if (Test_MotorIdentityMismatch == TRUE)
    {
        Readiness->identity.evidenceSha256[1] = 1U;
    }
    Readiness->structuralValid =
        (Test_MotorReadinessStructuralInvalid == TRUE) ? FALSE : TRUE;
    Readiness->measurementApproved = Test_MotorReadinessApproved;
    Readiness->calibrationComplete = Test_CalibrationComplete;
    Readiness->offsetsInstalled = Test_MotorOffsetsInstalled;
    return E_OK;
}

boolean MotorControlCdd_IsCalibrationActive(void)
{
    return Test_CalibrationActive;
}

boolean MotorControlCdd_IsCalibrationCleanupPending(void)
{
    return Test_CalibrationCleanupPending;
}

MotorControlCdd_CalibrationFailureType
MotorControlCdd_GetCalibrationFailureCause(void)
{
    return MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE;
}

void MotorControlCdd_EmergencyOff(void)
{
    Test_EmergencyOffCount++;
}

uint32 Mcu_GetClockFaultEvents(void)
{
    uint32 events;

    Test_ClockReadCount++;
    if ((Test_ClockEventOnRead != 0U) &&
        (Test_ClockReadCount >= Test_ClockEventOnRead))
    {
        Test_ClockEvents = MCU_CLOCK_FAULT_EVENT_CSS;
    }
    events = Test_ClockEvents;
    Test_FaultReadHook(TEST_FAULT_READ_CLOCK);
    return events;
}

Os_TickType Os_GetCounterValue(void)
{
    return Test_Tick;
}

boolean Os_IsRunning(void)
{
    return Test_OsRunning;
}

void Wdg_Trigger(void)
{
    Test_WdgTriggerCount++;
}

void SerialCommunicationCdd_MainFunction(uint32 CurrentTick)
{
    if (CurrentTick != Test_Tick)
    {
        Test_SerialMainCount = UINT_MAX;
        return;
    }
    Test_SerialMainCount++;
}

Std_ReturnType SerialCommunicationCdd_RequestAcquire(
    const SerialCommunicationCdd_RequestType **Request)
{
    if ((Request == NULL_PTR) || (Test_SerialRequestAvailable != TRUE))
    {
        return E_NOT_OK;
    }

    *Request = &Test_SerialRequest;
    return E_OK;
}

Std_ReturnType SerialCommunicationCdd_RequestRelease(
    const SerialCommunicationCdd_RequestType *Request)
{
    if ((Request != &Test_SerialRequest) ||
        (Test_SerialRequestAvailable != TRUE))
    {
        return E_NOT_OK;
    }

    Test_SerialRequestAvailable = FALSE;
    Test_SerialReleaseCount++;
    return E_OK;
}

Std_ReturnType SerialCommunicationCdd_ResponseQueue(
    const SerialCommunicationCdd_ResponseType *Response)
{
    if (Response == NULL_PTR)
    {
        return E_NOT_OK;
    }

    Test_SerialResponse = *Response;
    Test_SerialResponseCount++;
    return E_OK;
}

Std_ReturnType SerialCommunicationCdd_GetDiagnostics(
    SerialCommunicationCdd_DiagnosticsType *Diagnostics)
{
    if (Diagnostics == NULL_PTR)
    {
        return E_NOT_OK;
    }

    *Diagnostics = Test_SerialDiagnostics;
    return E_OK;
}

static void Test_MailboxHook(uint32 state, uint32 slotIndex)
{
    (void)slotIndex;
    if ((state == SAFETY_SUPERVISOR_MAILBOX_WRITING) &&
        (Test_ReadWhileWriting == TRUE))
    {
        SafetySupervisor_SnapshotType concurrentSnapshot;

        Test_ReadWhileWriting = FALSE;
        Test_ConcurrentReadResult =
            SafetySupervisor_GetLatestSnapshot(&concurrentSnapshot);
    }
    else if ((state == SAFETY_SUPERVISOR_MAILBOX_READING) &&
             (Test_PublishWhileReading == TRUE))
    {
        SafetySupervisor_SnapshotType concurrentSnapshot;

        Test_PublishWhileReading = FALSE;
        (void)memset(&concurrentSnapshot, 0, sizeof(concurrentSnapshot));
        concurrentSnapshot.acquisitionSequence = 2U;
        concurrentSnapshot.acquisitionValid = TRUE;
        SafetySupervisor_PublishSnapshot(&concurrentSnapshot);
    }
}

static int Test_SafeLifecycleStopsAtCalibrating(void)
{
    SafetySupervisor_SnapshotType snapshot;

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    TEST_CHECK(StateManager_InitBootSafe() == E_NOT_OK);
    TEST_CHECK(SafetySupervisor_Init() == E_NOT_OK);
    TEST_CHECK(StateManager_EnterSelfTest() == E_NOT_OK);

    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);

    Test_AdcResult.vbusRaw = 2100U;
    Test_AdcResult.temperatureRaw = 900U;
    Test_AdcResult.acquisitionSequence = 1U;
    Test_GenerationId = 37U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(&snapshot) == E_OK);
    TEST_CHECK(snapshot.vbusRaw == 2100U);
    TEST_CHECK(snapshot.temperatureRaw == 900U);
    TEST_CHECK(snapshot.acquisitionSequence == 1U);
    TEST_CHECK(snapshot.osTick == 2U);
    TEST_CHECK(snapshot.pwmGenerationId == 37U);
    TEST_CHECK(snapshot.acquisitionValid == TRUE);
    TEST_CHECK(snapshot.limitsValid == FALSE);
    TEST_CHECK(Test_FastSafetyPublishCount == 1U);
    TEST_CHECK(Test_LastFastSafetySnapshot.vbusRaw == snapshot.vbusRaw);
    TEST_CHECK(Test_LastFastSafetySnapshot.temperatureRaw ==
               snapshot.temperatureRaw);
    TEST_CHECK(Test_LastFastSafetySnapshot.acquisitionSequence ==
               snapshot.acquisitionSequence);
    TEST_CHECK(Test_LastFastSafetySnapshot.osTick == snapshot.osTick);
    TEST_CHECK(Test_LastFastSafetySnapshot.pwmGenerationId ==
               snapshot.pwmGenerationId);
    TEST_CHECK(Test_LastFastSafetySnapshot.acquisitionValid == TRUE);
    TEST_CHECK(Test_LastFastSafetySnapshot.limitsValid == FALSE);
    TEST_CHECK(Test_LastFastSafetySnapshot.structuralValid == TRUE);
    TEST_CHECK(Test_LastFastSafetySnapshot.measurementApproved == FALSE);
    TEST_CHECK(Test_LastFastSafetySnapshot.identity.schemaVersion ==
               SC1_MEASUREMENT_SCHEMA_VERSION);

    Test_AdcResult.acquisitionSequence = 2U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 3U;
    SafetySupervisor_MainFunction();
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    TEST_CHECK(StateManager_GetFaultMask() == STATE_MANAGER_FAULT_NONE);
    TEST_CHECK(Test_EmergencyOffCount == 0U);
    return 0;
}

static void Test_PublishSafetyResult(uint32 tick, uint32 sequence)
{
    Test_Tick = tick;
    Test_AdcResult.vbusRaw = 2100U;
    Test_AdcResult.temperatureRaw = 900U;
    Test_AdcResult.acquisitionSequence = sequence;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    SafetySupervisor_MainFunction();
}

static int Test_Sc1SnapshotAgeBoundaries(void)
{
    SafetySupervisor_SnapshotType snapshot;

    Test_ResetFixture();
    Test_SafetyConfigApproved = TRUE;
    TEST_CHECK(SafetySupervisor_Init() == E_OK);
    (void)memset(&snapshot, 0, sizeof(snapshot));
    snapshot.acquisitionValid = TRUE;
    snapshot.limitsValid = TRUE;
    snapshot.osTick = 100U;
    TEST_CHECK(SafetySupervisor_IsSnapshotReady(&snapshot, 100U) == TRUE);
    TEST_CHECK(SafetySupervisor_IsSnapshotReady(&snapshot, 101U) == TRUE);
    TEST_CHECK(SafetySupervisor_IsSnapshotReady(&snapshot, 102U) == FALSE);
    snapshot.limitsValid = FALSE;
    TEST_CHECK(SafetySupervisor_IsSnapshotReady(&snapshot, 100U) == FALSE);
    snapshot.limitsValid = TRUE;
    snapshot.acquisitionValid = FALSE;
    TEST_CHECK(SafetySupervisor_IsSnapshotReady(&snapshot, 100U) == FALSE);
    TEST_CHECK(SafetySupervisor_IsSnapshotReady(NULL_PTR, 100U) == FALSE);
    return 0;
}

static int Test_FastSafetyCopyFailurePublishesInternalFault(void)
{
    Test_ResetFixture();
    TEST_CHECK(SafetySupervisor_Init() == E_OK);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.vbusRaw = 2100U;
    Test_AdcResult.temperatureRaw = 900U;
    Test_AdcResult.acquisitionSequence = 1U;
    Test_GenerationId = 7U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_FastSafetyPublishReturn = E_NOT_OK;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_INTERNAL);
    TEST_CHECK(Test_EmergencyOffCount == 1U);
    TEST_CHECK(Test_FastSafetyPublishCount == 0U);
    return 0;
}

static int Test_RunSc1RawLimitCase(
    uint16 vbusRaw,
    uint16 temperatureRaw,
    boolean expectedValid)
{
    SafetySupervisor_SnapshotType snapshot;

    Test_ResetFixture();
    Test_SafetyConfigApproved = TRUE;
    TEST_CHECK(SafetySupervisor_Init() == E_OK);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.vbusRaw = vbusRaw;
    Test_AdcResult.temperatureRaw = temperatureRaw;
    Test_AdcResult.acquisitionSequence = 1U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(&snapshot) == E_OK);
    TEST_CHECK(snapshot.limitsValid == expectedValid);
    TEST_CHECK(SafetySupervisor_GetFaultEvent() == 0U);
    return 0;
}

static int Test_Sc1RawLimitInclusiveAndAdjacentCases(void)
{
    TEST_CHECK(Test_RunSc1RawLimitCase(100U, 100U, TRUE) == 0);
    TEST_CHECK(Test_RunSc1RawLimitCase(4000U, 4000U, TRUE) == 0);
    TEST_CHECK(Test_RunSc1RawLimitCase(99U, 100U, FALSE) == 0);
    TEST_CHECK(Test_RunSc1RawLimitCase(4001U, 100U, FALSE) == 0);
    TEST_CHECK(Test_RunSc1RawLimitCase(100U, 99U, FALSE) == 0);
    TEST_CHECK(Test_RunSc1RawLimitCase(100U, 4001U, FALSE) == 0);
    return 0;
}

static int Test_Sc1aCompletionDisarmsDeadlineAndCannotReady(void)
{
    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);

    Test_PublishSafetyResult(2U, 1U);
    StateManager_MainFunction();
    TEST_CHECK(Test_CalibrationStartCount == 1U);
    TEST_CHECK(Test_CalibrationActive == TRUE);

    Test_CompleteCalibrationOnStep = TRUE;
    Test_PublishSafetyResult(3U, 2U);
    StateManager_MainFunction();
    TEST_CHECK(Test_CalibrationStepCount == 1U);
    TEST_CHECK(Test_CalibrationComplete == TRUE);
    TEST_CHECK(StateManager_CalibrationDeadlineArmed == FALSE);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    TEST_CHECK(StateManager_GetFaultMask() == STATE_MANAGER_FAULT_NONE);

    StateManager_CalibrationActivation = 254U;
    Test_Tick = 4U;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_CalibrationActivation == 255U);
    Test_Tick = 5U;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_CalibrationActivation == 256U);
    Test_Tick = 6U;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_CalibrationActivation == 257U);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    TEST_CHECK(StateManager_GetFaultMask() == STATE_MANAGER_FAULT_NONE);
    return 0;
}

static int Test_Sc1StalledDeadlineAndClockCleanupOrdering(void)
{
    uint32 activation;

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    for (activation = 1U; activation <= 255U; activation++)
    {
        Test_Tick++;
        StateManager_MainFunction();
    }
    TEST_CHECK(StateManager_CalibrationActivation == 255U);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    Test_Tick++;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_CalibrationActivation == 256U);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);
    TEST_CHECK(Test_CalibrationCleanupPending == TRUE);

    Test_ClockEventOnRead = Test_ClockReadCount + 2U;
    Test_Tick++;
    StateManager_MainFunction();
    TEST_CHECK(Test_CalibrationCleanupCount == 0U);
    TEST_CHECK(Test_LastAbortMode ==
               MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC);
    TEST_CHECK(Test_SerialMainCount == 256U);

    Test_ResetFixture();
    StateManager_State = (uint32)STATE_MANAGER_FAULT_LATCHED;
    Test_CalibrationCleanupPending = TRUE;
    Test_ClockEventOnRead = Test_ClockReadCount + 3U;
    StateManager_MainFunction();
    TEST_CHECK(Test_CalibrationCleanupCount == 1U);
    TEST_CHECK(Test_LastAbortMode ==
               MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC);
    TEST_CHECK(Test_CalibrationCleanupPending == FALSE);
    return 0;
}

static int Test_Sc1ClockLossAtEveryCleanupSubstate(void)
{
    uint32 substate;

    for (substate = 0U; substate < 4U; substate++)
    {
        Test_ResetFixture();
        StateManager_State = (uint32)STATE_MANAGER_FAULT_LATCHED;
        Test_CalibrationCleanupPending = TRUE;
        Test_CalibrationCleanupSubstate = substate;
        Test_ClockEventOnRead = Test_ClockReadCount + 2U;
        StateManager_MainFunction();
        TEST_CHECK(Test_CalibrationCleanupCount == 0U);
        TEST_CHECK(Test_LastCleanupSubstate == UINT_MAX);
        TEST_CHECK(Test_LastAbortMode ==
                   MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC);
        TEST_CHECK(Test_CalibrationCleanupPending == FALSE);
        TEST_CHECK(Test_SerialMainCount == 0U);

        Test_ResetFixture();
        StateManager_State = (uint32)STATE_MANAGER_FAULT_LATCHED;
        Test_CalibrationCleanupPending = TRUE;
        Test_CalibrationCleanupSubstate = substate;
        Test_ClockEventOnRead = Test_ClockReadCount + 3U;
        StateManager_MainFunction();
        TEST_CHECK(Test_CalibrationCleanupCount == 1U);
        TEST_CHECK(Test_LastCleanupSubstate == substate);
        TEST_CHECK(Test_LastAbortMode ==
                   MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC);
        TEST_CHECK(Test_CalibrationCleanupPending == FALSE);
        TEST_CHECK(Test_SerialMainCount == 0U);

        StateManager_MainFunction();
        TEST_CHECK(Test_CalibrationCleanupCount == 1U);
        TEST_CHECK(Test_SerialMainCount == 0U);
    }
    return 0;
}

static int Test_Sc1DeadlineAtEveryActiveSubstate(void)
{
    static const uint32 activeSubstates[] =
    {
        TEST_CALIBRATION_SUBSTATE_ACQUIRE,
        TEST_CALIBRATION_SUBSTATE_RESTORE_REQUEST,
        TEST_CALIBRATION_SUBSTATE_RESTORE_VERIFY
    };
    uint32 index;

    for (index = 0U;
         index < (uint32)(sizeof(activeSubstates) /
                         sizeof(activeSubstates[0]));
         index++)
    {
        Test_ResetFixture();
        StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
        StateManager_CalibrationActivation = 255U;
        StateManager_CalibrationDeadlineArmed = TRUE;
        StateManager_CalibrationStarted = TRUE;
        Test_CalibrationActive = TRUE;
        Test_CalibrationActiveSubstate = activeSubstates[index];

        StateManager_MainFunction();
        TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK((StateManager_GetFaultMask() &
                    STATE_MANAGER_FAULT_CALIBRATION) != 0U);
        TEST_CHECK(Test_LastAbortMode ==
                   MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP);
        TEST_CHECK(Test_AbortObservedSubstate == activeSubstates[index]);
        TEST_CHECK(Test_CalibrationStepCount == 0U);
        TEST_CHECK(Test_CalibrationCleanupPending == TRUE);
        TEST_CHECK(Test_SerialMainCount == 0U);
    }
    return 0;
}

static int Test_Sc1CleanupFaultBoundarySuppressesCommands(void)
{
    uint32 boundary;

    Test_ResetFixture();
    StateManager_State = (uint32)STATE_MANAGER_FAULT_LATCHED;
    Test_CalibrationCleanupPending = TRUE;
    for (boundary = 0U; boundary < 4U; boundary++)
    {
        uint32 serialCount = Test_SerialMainCount;

        Test_MotorEvents.breakEvent =
            ((boundary + 1U) << 16U) |
            MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK;
        StateManager_MainFunction();
        TEST_CHECK(Test_CalibrationCleanupCount == boundary);
        TEST_CHECK(Test_SerialMainCount == serialCount);
        TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);

        StateManager_MainFunction();
        TEST_CHECK(Test_CalibrationCleanupCount == (boundary + 1U));
        TEST_CHECK(Test_SerialMainCount == serialCount);
        TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    }
    TEST_CHECK(Test_CalibrationCleanupPending == FALSE);
    return 0;
}

static int Test_Sc1ReadyTransactionAndIdentityRejection(void)
{
    uint32 serialCount;

    Test_ResetFixture();
    Test_SafetyConfigApproved = TRUE;
    Test_MotorReadinessApproved = TRUE;
    Test_MotorOffsetsInstalled = TRUE;
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    StateManager_MainFunction();
    Test_PublishSafetyResult(2U, 1U);
    StateManager_MainFunction();
    Test_CompleteCalibrationOnStep = TRUE;
    Test_PublishSafetyResult(3U, 2U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    Test_PublishSafetyResult(4U, 3U);
    serialCount = Test_SerialMainCount;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_READY);
    TEST_CHECK(StateManager_GetFaultMask() == STATE_MANAGER_FAULT_NONE);
    TEST_CHECK(Test_SerialMainCount == serialCount);
    TEST_CHECK(Test_EmergencyOffCount == 0U);

    Test_ResetFixture();
    Test_SafetyConfigApproved = TRUE;
    Test_MotorReadinessApproved = TRUE;
    Test_MotorOffsetsInstalled = TRUE;
    Test_MotorIdentityMismatch = TRUE;
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    StateManager_MainFunction();
    Test_PublishSafetyResult(2U, 1U);
    StateManager_MainFunction();
    Test_CompleteCalibrationOnStep = TRUE;
    Test_PublishSafetyResult(3U, 2U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);
    TEST_CHECK(Test_CalibrationCleanupPending == TRUE);
    return 0;
}

static int Test_PrepareApprovedSc1Completion(void)
{
    Test_ResetFixture();
    Test_SafetyConfigApproved = TRUE;
    Test_MotorReadinessApproved = TRUE;
    Test_MotorOffsetsInstalled = TRUE;
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    StateManager_MainFunction();
    Test_PublishSafetyResult(2U, 1U);
    StateManager_MainFunction();
    Test_CompleteCalibrationOnStep = TRUE;
    Test_PublishSafetyResult(3U, 2U);
    StateManager_MainFunction();
    TEST_CHECK(Test_CalibrationComplete == TRUE);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    return 0;
}

static int Test_Sc1ReadyAdjacentNegativeGuards(void)
{
    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_MotorOffsetsInstalled = FALSE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    TEST_CHECK(StateManager_GetFaultMask() == STATE_MANAGER_FAULT_NONE);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_MotorReadinessCallFails = TRUE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_MotorReadinessStructuralInvalid = TRUE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_ResultCallFails = TRUE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_ResultInvalid = TRUE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_ResultApprovalMismatch = TRUE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_ResultOffsetsMismatch = TRUE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_ResultIdentityMismatch = TRUE;
    Test_PublishSafetyResult(4U, 3U);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    StateManager_LatestSafetySnapshot.osTick = 4U;
    Test_Tick = 4U;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    StateManager_LatestSafetySnapshot.acquisitionSequence = 3U;
    StateManager_LatestSafetySnapshot.osTick = 5U;
    Test_Tick = 5U;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    StateManager_LatestSafetySnapshot.acquisitionSequence = 4U;
    StateManager_LatestSafetySnapshot.osTick = 4U;
    Test_Tick = 4U;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_CALIBRATION) != 0U);
    return 0;
}

static int Test_Sc1ReadyEventGuardInjection(void)
{
    uint32 serialCount;

    Test_ResetFixture();
    Test_SafetyConfigApproved = TRUE;
    Test_MotorReadinessApproved = TRUE;
    Test_MotorOffsetsInstalled = TRUE;
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    StateManager_MainFunction();
    Test_CalibrationHookStage =
        STATE_MANAGER_TEST_CALIBRATION_AFTER_CDD;
    Test_PublishSafetyResult(2U, 1U);
    serialCount = Test_SerialMainCount;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_BREAK) != 0U);
    TEST_CHECK(Test_SerialMainCount == serialCount);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_CalibrationHookStage = STATE_MANAGER_TEST_CALIBRATION_PRE_READY;
    Test_PublishSafetyResult(4U, 3U);
    serialCount = Test_SerialMainCount;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_BREAK) != 0U);
    TEST_CHECK(Test_SerialMainCount == serialCount);

    TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
    Test_CalibrationHookStage = STATE_MANAGER_TEST_CALIBRATION_POST_READY;
    Test_PublishSafetyResult(4U, 3U);
    serialCount = Test_SerialMainCount;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_BREAK) != 0U);
    TEST_CHECK(Test_SerialMainCount == serialCount);
    return 0;
}

static int Test_SafetyFailuresAreLatched(void)
{
    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_AdcStartReturn = E_NOT_OK;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_START_FAILED);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_SAFETY_ADC) != 0U);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_Tick = 2U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_PENDING;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_TIMEOUT);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_Tick = 2U;
    Test_AdcResult.acquisitionSequence = 0U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_SEQUENCE);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_Tick = 2U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_FAILED;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_ADC_FAILED);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 1U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 3U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 3U;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_SEQUENCE);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 1U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 1U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 3U;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_SEQUENCE);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 2U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 1U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 3U;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_SEQUENCE);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 1U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_AdcStartReturn = E_NOT_OK;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_START_FAILED);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    Test_AdcResult.acquisitionSequence = 1U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_GenerationLookupReturn = E_NOT_OK;
    Test_Tick = 2U;
    SafetySupervisor_MainFunction();
    TEST_CHECK((SafetySupervisor_GetFaultEvent() & 0xFFFFU) ==
               SAFETY_SUPERVISOR_EVENT_CAUSE_INTERNAL);

    Test_ResetFixture();
    TEST_CHECK(SafetySupervisor_Init() == E_OK);
    SafetySupervisor_State = SAFETY_SUPERVISOR_PENDING;
    SafetySupervisor_SequenceEstablished = TRUE;
    SafetySupervisor_LastAcquisitionSequence = UINT_MAX;
    Test_AdcResult.acquisitionSequence = 0U;
    Test_AdcPollStatus = ADC_SAFETY_GROUP_COMPLETED;
    Test_Tick = 1U;
    SafetySupervisor_MainFunction();
    TEST_CHECK(SafetySupervisor_GetFaultEvent() == 0U);
    return 0;
}

static int Test_MailboxDropAndSequenceWrap(void)
{
    SafetySupervisor_SnapshotType snapshot;
    SafetySupervisor_SnapshotType output;
    uint32 dropCount;

    Test_ResetFixture();
    TEST_CHECK(SafetySupervisor_Init() == E_OK);
    (void)memset(&snapshot, 0, sizeof(snapshot));
    snapshot.acquisitionValid = TRUE;
    snapshot.acquisitionSequence = UINT_MAX - 1U;
    SafetySupervisor_PublishSnapshot(&snapshot);
    snapshot.acquisitionSequence = UINT_MAX;
    SafetySupervisor_PublishSnapshot(&snapshot);
    snapshot.acquisitionSequence = 0U;
    SafetySupervisor_PublishSnapshot(&snapshot);
    snapshot.acquisitionSequence = 1U;
    SafetySupervisor_PublishSnapshot(&snapshot);

    TEST_CHECK(SafetySupervisor_GetDropCount(&dropCount) == E_OK);
    TEST_CHECK(dropCount == 1U);
    TEST_CHECK(SafetySupervisor_GetDropCount(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(&output) == E_OK);
    TEST_CHECK(output.acquisitionSequence == 0U);
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(&output) == E_NOT_OK);

    snapshot.acquisitionSequence = 1U;
    Test_ReadWhileWriting = TRUE;
    SafetySupervisor_PublishSnapshot(&snapshot);
    TEST_CHECK(Test_ConcurrentReadResult == E_NOT_OK);
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(&output) == E_OK);
    TEST_CHECK(output.acquisitionSequence == 1U);

    SafetySupervisor_PublishSnapshot(&snapshot);
    Test_PublishWhileReading = TRUE;
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(&output) == E_OK);
    TEST_CHECK(output.acquisitionSequence == 2U);
    TEST_CHECK(SafetySupervisor_GetLatestSnapshot(&output) == E_NOT_OK);
    return 0;
}

static int Test_RunWdgFailureScenario(
    uint16 expectedCause,
    uint32 tickDelta,
    uint32 safetyDelta,
    uint32 stateDelta,
    StateManager_StateType state)
{
    WdgM_HandoffTokenType token;

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    SafetySupervisor_AliveCounter = 10U;
    StateManager_AliveCounter = 10U;
    Test_Tick = 10U;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_OK);
    TEST_CHECK(WdgM_AcceptHandoff(token) == E_OK);
    SafetySupervisor_AliveCounter += safetyDelta;
    StateManager_AliveCounter += stateDelta;
    StateManager_State = (uint32)state;
    Test_Tick += tickDelta;
    WdgM_MainFunction();
    TEST_CHECK((WdgM_GetFaultEvent() & 0xFFFFU) == expectedCause);
    TEST_CHECK(WdgM_HasRefreshAuthority() == FALSE);
    TEST_CHECK(Test_WdgTriggerCount == 0U);
    TEST_CHECK(Test_EmergencyOffCount == 1U);
    return 0;
}

static int Test_StateAggregatesIndependentFaultProducers(void)
{
    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_MotorEvents.breakEvent =
        (1UL << 16U) | MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK;
    Test_MotorEvents.adcEvent =
        (1UL << 16U) | MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_BREAK) != 0U);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_ADC) != 0U);
    TEST_CHECK(Test_EmergencyOffCount == 1U);

    StateManager_MainFunction();
    TEST_CHECK(Test_EmergencyOffCount == 1U);

    Test_MotorEvents.breakEvent =
        (2UL << 16U) | MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK;
    StateManager_MainFunction();
    TEST_CHECK(Test_EmergencyOffCount == 2U);

    StateManager_LastMotorBreakSequence = UINT16_MAX;
    Test_MotorEvents.breakEvent = MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_LastMotorBreakSequence == 0U);
    TEST_CHECK(Test_EmergencyOffCount == 3U);

    Test_ClockEvents = MCU_CLOCK_FAULT_EVENT_CSS;
    StateManager_MainFunction();
    TEST_CHECK((StateManager_GetFaultMask() & STATE_MANAGER_FAULT_CLOCK) !=
               0U);
    return 0;
}

static int Test_StateOwnedSerialResponsesRemainNonActuating(void)
{
    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_SerialRequest.command =
        SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ;
    Test_SerialRequest.sessionId = 7U;
    Test_SerialRequest.sequence = 1U;
    Test_SerialRequest.diagnosticSelector = 0U;
    Test_SerialRequestAvailable = TRUE;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    TEST_CHECK(Test_SerialMainCount == 1U);
    TEST_CHECK(Test_SerialReleaseCount == 1U);
    TEST_CHECK(Test_SerialResponseCount == 1U);
    TEST_CHECK(Test_SerialResponse.status ==
               SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED);
    TEST_CHECK(Test_SerialResponse.detail ==
               SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_STATE);
    TEST_CHECK(Test_SerialResponse.data0 ==
               (uint32)STATE_MANAGER_CALIBRATING);
    TEST_CHECK(Test_SerialResponse.data1 == STATE_MANAGER_FAULT_NONE);

    Test_SerialRequest.command = SERIAL_COMMUNICATION_CDD_COMMAND_STOP;
    Test_SerialRequest.sequence++;
    Test_SerialRequestAvailable = TRUE;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);
    TEST_CHECK(Test_SerialResponse.status ==
               SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED);
    TEST_CHECK(Test_EmergencyOffCount == 0U);

    Test_SerialRequest.command = SERIAL_COMMUNICATION_CDD_COMMAND_START;
    Test_SerialRequest.sequence++;
    Test_SerialRequest.leaseMs = 100U;
    Test_SerialRequest.receiveTick = Test_Tick;
    Test_SerialRequestAvailable = TRUE;
    StateManager_MainFunction();
    TEST_CHECK(Test_SerialResponse.status ==
               SERIAL_COMMUNICATION_CDD_STATUS_NOT_READY);
    TEST_CHECK(Test_SerialResponse.detail ==
               SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_STATE);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_CALIBRATING);

    Test_Tick = 100U;
    Test_SerialRequest.command =
        SERIAL_COMMUNICATION_CDD_COMMAND_SET_CURRENT;
    Test_SerialRequest.sequence++;
    Test_SerialRequest.receiveTick = 0U;
    Test_SerialRequest.leaseMs = 100U;
    Test_SerialRequestAvailable = TRUE;
    StateManager_MainFunction();
    TEST_CHECK(Test_SerialResponse.status ==
               SERIAL_COMMUNICATION_CDD_STATUS_NOT_READY);
    TEST_CHECK(Test_SerialResponse.detail ==
               SERIAL_COMMUNICATION_CDD_DETAIL_EXPIRED_LEASE);

    Test_SerialDiagnostics.frameOrCrcRejects = 1U;
    Test_SerialDiagnostics.identityOrSequenceRejects = 2U;
    Test_SerialDiagnostics.usartErrorOrRingDrops = 3U;
    Test_SerialDiagnostics.mailboxBusyOrResponseDrops = 4U;
    Test_SerialRequest.command =
        SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ;
    Test_SerialRequest.sequence++;
    Test_SerialRequest.diagnosticSelector = 1U;
    Test_SerialRequest.leaseMs = 0U;
    Test_SerialRequestAvailable = TRUE;
    StateManager_MainFunction();
    TEST_CHECK(Test_SerialResponse.status ==
               SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED);
    TEST_CHECK(Test_SerialResponse.detail ==
               SERIAL_COMMUNICATION_CDD_DETAIL_LINK_DIAGNOSTICS);
    TEST_CHECK(Test_SerialResponse.data0 == 0x00020001U);
    TEST_CHECK(Test_SerialResponse.data1 == 0x00040003U);

    {
        uint32 responseCount = Test_SerialResponseCount;
        uint32 releaseCount = Test_SerialReleaseCount;

        Test_MotorEvents.breakEvent = (1UL << 16U) |
                                      MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK;
        Test_SerialRequest.sequence++;
        Test_SerialRequest.diagnosticSelector = 0U;
        Test_SerialRequestAvailable = TRUE;
        StateManager_MainFunction();
        TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK((StateManager_GetFaultMask() &
                    STATE_MANAGER_FAULT_MOTOR_BREAK) != 0U);
        TEST_CHECK(Test_SerialResponseCount == responseCount);
        TEST_CHECK(Test_SerialReleaseCount == releaseCount);
        TEST_CHECK(Test_SerialRequestAvailable == TRUE);
        TEST_CHECK(Test_EmergencyOffCount == 1U);
    }
    return 0;
}

static int Test_WdgMHandoffAndAliveProfiles(void)
{
    WdgM_HandoffTokenType token;

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    SafetySupervisor_AliveCounter = 10U;
    StateManager_AliveCounter = 10U;
    Test_Tick = 10U;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, NULL_PTR) == E_NOT_OK);
    Test_OsRunning = FALSE;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_NOT_OK);
    Test_OsRunning = TRUE;
    StateManager_State = (uint32)STATE_MANAGER_BOOT_SAFE;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_NOT_OK);
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    SafetySupervisor_AliveCounter = 9U;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_NOT_OK);
    SafetySupervisor_AliveCounter = 11U;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_NOT_OK);
    SafetySupervisor_AliveCounter = 10U;
    StateManager_AliveCounter = 9U;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_NOT_OK);
    StateManager_AliveCounter = 11U;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_NOT_OK);
    StateManager_AliveCounter = 10U;
    TEST_CHECK(WdgM_PrepareHandoff(Test_Tick, &token) == E_OK);
    TEST_CHECK(WdgM_AcceptHandoff(token + 1U) == E_NOT_OK);
    TEST_CHECK(WdgM_AcceptHandoff(token) == E_OK);
    TEST_CHECK(WdgM_AcceptHandoff(token) == E_NOT_OK);
    TEST_CHECK(WdgM_HasRefreshAuthority() == TRUE);

    SafetySupervisor_AliveCounter = 20U;
    StateManager_AliveCounter = 20U;
    Test_Tick = 20U;
    WdgM_MainFunction();
    TEST_CHECK(Test_WdgTriggerCount == 1U);
    TEST_CHECK(WdgM_GetFaultEvent() == 0U);

    WdgM_LastTick = UINT_MAX - 4U;
    WdgM_LastSafetyAlive = UINT_MAX - 4U;
    WdgM_LastStateAlive = UINT_MAX - 4U;
    Test_Tick = 5U;
    SafetySupervisor_AliveCounter = 5U;
    StateManager_AliveCounter = 5U;
    StateManager_State = (uint32)STATE_MANAGER_FAULT_LATCHED;
    WdgM_MainFunction();
    TEST_CHECK(Test_WdgTriggerCount == 2U);
    TEST_CHECK(WdgM_GetFaultEvent() == 0U);

    StateManager_AliveCounter = 15U;
    Test_Tick = 15U;
    WdgM_MainFunction();
    TEST_CHECK(Test_WdgTriggerCount == 2U);
    TEST_CHECK(WdgM_HasRefreshAuthority() == FALSE);
    TEST_CHECK((WdgM_GetFaultEvent() & 0xFFFFU) ==
               WDGM_EVENT_CAUSE_SAFETY_ALIVE);
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() & STATE_MANAGER_FAULT_WDGM) !=
               0U);
    {
        uint32 emergencyOffCount = Test_EmergencyOffCount;

        StateManager_MainFunction();
        TEST_CHECK(Test_EmergencyOffCount == emergencyOffCount);
    }

    TEST_CHECK(Test_RunWdgFailureScenario(
                   WDGM_EVENT_CAUSE_OS_TIME,
                   11U, 10U, 10U,
                   STATE_MANAGER_CALIBRATING) == 0);
    TEST_CHECK(Test_RunWdgFailureScenario(
                   WDGM_EVENT_CAUSE_STATE_ALIVE,
                   10U, 10U, 9U,
                   STATE_MANAGER_CALIBRATING) == 0);
    TEST_CHECK(Test_RunWdgFailureScenario(
                   WDGM_EVENT_CAUSE_FOC_PROFILE,
                   10U, 10U, 10U,
                   STATE_MANAGER_RUN) == 0);
    return 0;
}

static uint32 Test_ExpectedFaultForReadStage(uint32 stage)
{
    switch (stage)
    {
        case TEST_FAULT_READ_MOTOR_BREAK:
            return STATE_MANAGER_FAULT_MOTOR_BREAK;
        case TEST_FAULT_READ_MOTOR_UPDATE:
            return STATE_MANAGER_FAULT_MOTOR_UPDATE;
        case TEST_FAULT_READ_MOTOR_ADC:
            return STATE_MANAGER_FAULT_MOTOR_ADC;
        case TEST_FAULT_READ_MOTOR_CALIBRATION:
            return STATE_MANAGER_FAULT_CALIBRATION;
        case TEST_FAULT_READ_CLOCK:
            return STATE_MANAGER_FAULT_CLOCK;
        case TEST_FAULT_READ_SAFETY:
            return STATE_MANAGER_FAULT_SAFETY_ADC;
        case TEST_FAULT_READ_WDGM:
            return STATE_MANAGER_FAULT_WDGM;
        default:
            return STATE_MANAGER_FAULT_INVARIANT;
    }
}

static uint32 Test_CriticalClassForReadStage(uint32 stage)
{
    if (stage <= TEST_FAULT_READ_MOTOR_CALIBRATION)
    {
        return STATE_MANAGER_CRITICAL_CLASS_MOTOR;
    }
    if (stage == TEST_FAULT_READ_CLOCK)
    {
        return STATE_MANAGER_CRITICAL_CLASS_CLOCK;
    }
    if (stage == TEST_FAULT_READ_SAFETY)
    {
        return STATE_MANAGER_CRITICAL_CLASS_SAFETY;
    }
    return STATE_MANAGER_CRITICAL_CLASS_WDGM;
}

static int Test_ReadyInterlockAfterEveryDetailedLoad(void)
{
    uint32 stage;

    for (stage = TEST_FAULT_READ_MOTOR_BREAK;
         stage <= TEST_FAULT_READ_WDGM;
         stage++)
    {
        uint32 expectedFault = Test_ExpectedFaultForReadStage(stage);
        uint32 expectedClass = Test_CriticalClassForReadStage(stage);
        uint32 serialCount;

        TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
        Test_FaultReadInjectionStage = stage;
        Test_FaultReadInjectionOccurrence = 3U;
        Test_PublishSafetyResult(4U, 3U);
        serialCount = Test_SerialMainCount;
        StateManager_MainFunction();
        TEST_CHECK(StateManager_GetState() ==
                   STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK((StateManager_GetFaultMask() & expectedFault) != 0U);
        TEST_CHECK((StateManager_GetFaultMask() &
                    STATE_MANAGER_FAULT_UNCLASSIFIED_CRITICAL) == 0U);
        TEST_CHECK((StateManager_ClassifiedPendingMask & expectedClass) ==
                   expectedClass);
        TEST_CHECK(Test_SerialMainCount == serialCount);
        TEST_CHECK(Test_FaultReadInjectionStage == 0U);
    }
    return 0;
}

static int Test_ReadyCasAndPostReadyEveryProducer(void)
{
    static const uint32 classes[4] = {
        STATE_MANAGER_CRITICAL_CLASS_CLOCK,
        STATE_MANAGER_CRITICAL_CLASS_MOTOR,
        STATE_MANAGER_CRITICAL_CLASS_SAFETY,
        STATE_MANAGER_CRITICAL_CLASS_WDGM
    };
    static const uint32 faults[4] = {
        STATE_MANAGER_FAULT_CLOCK,
        STATE_MANAGER_FAULT_MOTOR_BREAK,
        STATE_MANAGER_FAULT_SAFETY_ADC,
        STATE_MANAGER_FAULT_WDGM
    };
    uint32 index;

    for (index = 0U; index < 4U; index++)
    {
        uint32 serialCount;

        TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
        Test_CalibrationHookClass = classes[index];
        Test_CalibrationHookStage =
            STATE_MANAGER_TEST_CALIBRATION_PRE_READY;
        Test_PublishSafetyResult(4U, 3U);
        serialCount = Test_SerialMainCount;
        StateManager_MainFunction();
        TEST_CHECK(StateManager_GetState() ==
                   STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK((StateManager_GetFaultMask() & faults[index]) != 0U);
        TEST_CHECK(Test_SerialMainCount == serialCount);

        TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
        Test_CalibrationHookClass = classes[index];
        Test_CalibrationHookStage =
            STATE_MANAGER_TEST_CALIBRATION_POST_READY;
        Test_PublishSafetyResult(4U, 3U);
        serialCount = Test_SerialMainCount;
        StateManager_MainFunction();
        TEST_CHECK(StateManager_GetState() ==
                   STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK((StateManager_GetFaultMask() & faults[index]) != 0U);
        TEST_CHECK(Test_SerialMainCount == serialCount);

        TEST_CHECK(Test_PrepareApprovedSc1Completion() == 0);
        Test_PublishSafetyResult(4U, 3U);
        StateManager_MainFunction();
        TEST_CHECK(StateManager_GetState() == STATE_MANAGER_READY);
        serialCount = Test_SerialMainCount;
        Test_PublishCriticalClass(classes[index]);
        TEST_CHECK(StateManager_GetState() ==
                   STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK((StateManager_GetFaultMask() &
                    ((classes[index] ==
                      STATE_MANAGER_CRITICAL_CLASS_MOTOR) ?
                     STATE_MANAGER_FAULT_UNCLASSIFIED_CRITICAL :
                     faults[index])) != 0U);
        TEST_CHECK(Test_SerialMainCount == serialCount);
        StateManager_MainFunction();
        TEST_CHECK((StateManager_GetFaultMask() & faults[index]) != 0U);
        TEST_CHECK((StateManager_ClassifiedPendingMask & classes[index]) !=
                   0U);
        TEST_CHECK(Test_SerialMainCount == serialCount);
    }
    return 0;
}

static int Test_NotifyValidationAndFaultMaskSnapshots(void)
{
    uint32 firstControl;
    uint32 subset;
    static const uint32 classes[] =
    {
        STATE_MANAGER_CRITICAL_CLASS_CLOCK,
        STATE_MANAGER_CRITICAL_CLASS_MOTOR,
        STATE_MANAGER_CRITICAL_CLASS_SAFETY,
        STATE_MANAGER_CRITICAL_CLASS_WDGM,
        STATE_MANAGER_CRITICAL_CLASS_INVARIANT
    };
    static const uint32 provisionalFaults[] =
    {
        STATE_MANAGER_FAULT_CLOCK,
        STATE_MANAGER_FAULT_UNCLASSIFIED_CRITICAL,
        STATE_MANAGER_FAULT_SAFETY_ADC,
        STATE_MANAGER_FAULT_WDGM,
        STATE_MANAGER_FAULT_INVARIANT
    };
    static const uint32 exactFaults[] =
    {
        STATE_MANAGER_FAULT_CLOCK,
        STATE_MANAGER_FAULT_MOTOR_BREAK,
        STATE_MANAGER_FAULT_SAFETY_ADC,
        STATE_MANAGER_FAULT_WDGM,
        STATE_MANAGER_FAULT_INVARIANT
    };

    Test_ResetFixture();
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    StateManager_NotifyCriticalFault(0U);
    TEST_CHECK((StateManager_State & STATE_MANAGER_CRITICAL_CLASS_ALL) ==
               STATE_MANAGER_CRITICAL_CLASS_INVARIANT);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_INVARIANT) != 0U);

    Test_ResetFixture();
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    StateManager_NotifyCriticalFault(0x80000000U);
    TEST_CHECK((StateManager_State & STATE_MANAGER_CRITICAL_CLASS_ALL) ==
               STATE_MANAGER_CRITICAL_CLASS_INVARIANT);
    TEST_CHECK((StateManager_State & 0x80000000U) == 0U);

    Test_ResetFixture();
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    StateManager_NotifyCriticalFault(
        STATE_MANAGER_CRITICAL_CLASS_MOTOR | 0x80000000U);
    TEST_CHECK((StateManager_State & STATE_MANAGER_CRITICAL_CLASS_ALL) ==
               (STATE_MANAGER_CRITICAL_CLASS_MOTOR |
                STATE_MANAGER_CRITICAL_CLASS_INVARIANT));
    firstControl = StateManager_State;
    StateManager_NotifyCriticalFault(
        STATE_MANAGER_CRITICAL_CLASS_MOTOR | 0x80000000U);
    TEST_CHECK(StateManager_State == firstControl);

    for (subset = 1U; subset < 32U; subset++)
    {
        uint32 classMask = 0U;
        uint32 expectedProvisional = 0U;
        uint32 expectedExact = 0U;
        uint32 classIndex;

        for (classIndex = 0U; classIndex < 5U; classIndex++)
        {
            if ((subset & (1UL << classIndex)) != 0U)
            {
                classMask |= classes[classIndex];
                expectedProvisional |= provisionalFaults[classIndex];
                expectedExact |= exactFaults[classIndex];
            }
        }

        Test_ResetFixture();
        StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
        Test_PublishCriticalClass(classMask);
        TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK(StateManager_GetFaultMask() == expectedProvisional);
        StateManager_MainFunction();
        TEST_CHECK(StateManager_GetFaultMask() == expectedExact);
        TEST_CHECK(StateManager_ClassifiedPendingMask == classMask);
    }

    Test_MotorEvents.adcEvent = Test_NextDetailedEvent(
        Test_MotorEvents.adcEvent,
        MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC);
    StateManager_NotifyCriticalFault(STATE_MANAGER_CRITICAL_CLASS_MOTOR);
    StateManager_MainFunction();
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_ADC) != 0U);
    TEST_CHECK(StateManager_ClassifiedPendingMask ==
               STATE_MANAGER_CRITICAL_CLASS_ALL);
    return 0;
}

static int Test_NonFaultTransitionInterlocks(void)
{
    Test_ResetFixture();
    Test_TransitionHookStage = STATE_MANAGER_TEST_TRANSITION_BOOT_SAFE;
    Test_TransitionHookClass = STATE_MANAGER_CRITICAL_CLASS_CLOCK;
    TEST_CHECK(StateManager_InitBootSafe() == E_NOT_OK);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_State & STATE_MANAGER_CRITICAL_CLASS_CLOCK) !=
               0U);

    Test_ResetFixture();
    TEST_CHECK(StateManager_InitBootSafe() == E_OK);
    Test_PublishCriticalClass(STATE_MANAGER_CRITICAL_CLASS_MOTOR);
    TEST_CHECK(StateManager_EnterSelfTest() == E_NOT_OK);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_BREAK) != 0U);

    Test_ResetFixture();
    TEST_CHECK(StateManager_InitBootSafe() == E_OK);
    Test_TransitionHookStage = STATE_MANAGER_TEST_TRANSITION_SELF_TEST;
    Test_TransitionHookClass = STATE_MANAGER_CRITICAL_CLASS_MOTOR;
    TEST_CHECK(StateManager_EnterSelfTest() == E_NOT_OK);
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_MOTOR_BREAK) != 0U);

    Test_ResetFixture();
    TEST_CHECK(Test_InitializeSupervision() == 0);
    Test_TransitionHookStage =
        STATE_MANAGER_TEST_TRANSITION_CALIBRATING;
    Test_TransitionHookClass = STATE_MANAGER_CRITICAL_CLASS_SAFETY;
    StateManager_MainFunction();
    TEST_CHECK(StateManager_GetState() == STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK((StateManager_GetFaultMask() &
                STATE_MANAGER_FAULT_SAFETY_ADC) != 0U);
    TEST_CHECK(Test_SerialMainCount == 0U);
    return 0;
}

static int Test_StrongFaultCasAndExhaustion(void)
{
    static const uint32 classes[5] = {
        STATE_MANAGER_CRITICAL_CLASS_CLOCK,
        STATE_MANAGER_CRITICAL_CLASS_MOTOR,
        STATE_MANAGER_CRITICAL_CLASS_SAFETY,
        STATE_MANAGER_CRITICAL_CLASS_WDGM,
        STATE_MANAGER_CRITICAL_CLASS_INVARIANT
    };
    uint32 index;

    for (index = 0U; index < 5U; index++)
    {
        Test_ResetFixture();
        StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
        Test_FaultCasArrivalMask[0] = classes[index];
        StateManager_LatchFault(STATE_MANAGER_FAULT_CALIBRATION);
        TEST_CHECK(StateManager_ControlState(StateManager_State) ==
                   STATE_MANAGER_FAULT_LATCHED);
        TEST_CHECK((StateManager_State & classes[index]) != 0U);
        TEST_CHECK(Test_FaultCasHookCount == 2U);
        TEST_CHECK(Test_FatalObserved == FALSE);
    }

    Test_ResetFixture();
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    for (index = 0U; index < 5U; index++)
    {
        Test_FaultCasArrivalMask[index] = classes[index];
    }
    StateManager_LatchFault(STATE_MANAGER_FAULT_CALIBRATION);
    TEST_CHECK((StateManager_State & STATE_MANAGER_CRITICAL_CLASS_ALL) ==
               STATE_MANAGER_CRITICAL_CLASS_ALL);
    TEST_CHECK(StateManager_ControlState(StateManager_State) ==
               STATE_MANAGER_FAULT_LATCHED);
    TEST_CHECK(Test_FaultCasHookCount == 6U);

    Test_ResetFixture();
    StateManager_State = (uint32)STATE_MANAGER_CALIBRATING;
    Test_FaultCasCorruptEveryAttempt = TRUE;
    Test_FatalJumpArmed = TRUE;
    if (setjmp(Test_FatalJump) == 0)
    {
        StateManager_LatchFault(STATE_MANAGER_FAULT_INVARIANT);
        TEST_CHECK(FALSE);
    }
    Test_FatalJumpArmed = FALSE;
    TEST_CHECK(Test_FatalObserved == TRUE);
    TEST_CHECK(Test_FaultCasHookCount == 6U);
    TEST_CHECK(Test_EmergencyOffCount == 2U);
    TEST_CHECK(Test_SerialMainCount == 0U);
    TEST_CHECK(Test_WdgTriggerCount == 0U);
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_SafeLifecycleStopsAtCalibrating();
    failures += Test_Sc1SnapshotAgeBoundaries();
    failures += Test_FastSafetyCopyFailurePublishesInternalFault();
    failures += Test_Sc1RawLimitInclusiveAndAdjacentCases();
    failures += Test_Sc1aCompletionDisarmsDeadlineAndCannotReady();
    failures += Test_Sc1StalledDeadlineAndClockCleanupOrdering();
    failures += Test_Sc1ClockLossAtEveryCleanupSubstate();
    failures += Test_Sc1DeadlineAtEveryActiveSubstate();
    failures += Test_Sc1CleanupFaultBoundarySuppressesCommands();
    failures += Test_Sc1ReadyTransactionAndIdentityRejection();
    failures += Test_Sc1ReadyAdjacentNegativeGuards();
    failures += Test_Sc1ReadyEventGuardInjection();
    failures += Test_SafetyFailuresAreLatched();
    failures += Test_MailboxDropAndSequenceWrap();
    failures += Test_StateAggregatesIndependentFaultProducers();
    failures += Test_StateOwnedSerialResponsesRemainNonActuating();
    failures += Test_WdgMHandoffAndAliveProfiles();
    failures += Test_ReadyInterlockAfterEveryDetailedLoad();
    failures += Test_ReadyCasAndPostReadyEveryProducer();
    failures += Test_NotifyValidationAndFaultMaskSnapshots();
    failures += Test_NonFaultTransitionInterlocks();
    failures += Test_StrongFaultCasAndExhaustion();

    if (failures == 0)
    {
        (void)printf("PASS: Step-7A/7B supervision exact-source host matrix\n");
        return 0;
    }

    (void)printf("FAIL: %d supervision host group(s)\n", failures);
    return 1;
}
