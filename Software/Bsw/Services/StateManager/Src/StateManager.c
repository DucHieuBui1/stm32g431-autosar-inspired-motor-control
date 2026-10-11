#include "StateManager.h"

#include "Mcu.h"
#include "MotorControlCdd.h"
#include "Os.h"
#include "Rte_CommandPolicy.h"
#include "SafetySupervisor.h"
#include "SerialCommunicationCdd.h"
#include "WdgM.h"

#define STATE_MANAGER_TEST_CALIBRATION_AFTER_CDD  1U
#define STATE_MANAGER_TEST_CALIBRATION_PRE_READY  2U
#define STATE_MANAGER_TEST_CALIBRATION_POST_READY 3U
#define STATE_MANAGER_TEST_TRANSITION_BOOT_SAFE   1U
#define STATE_MANAGER_TEST_TRANSITION_SELF_TEST   2U
#define STATE_MANAGER_TEST_TRANSITION_CALIBRATING 3U

#define STATE_MANAGER_CONTROL_STATE_MASK          0x000000FFU
#define STATE_MANAGER_CONTROL_RESERVED_MASK       0xFFE0FF00U
#define STATE_MANAGER_FAULT_LATCH_ATTEMPTS        6U
#define STATE_MANAGER_MOTOR_DETAIL_FAULT_MASK     \
    (STATE_MANAGER_FAULT_MOTOR_BREAK |            \
     STATE_MANAGER_FAULT_MOTOR_UPDATE |           \
     STATE_MANAGER_FAULT_MOTOR_ADC |              \
     STATE_MANAGER_FAULT_CALIBRATION)

#ifndef STATE_MANAGER_TEST_CALIBRATION_HOOK
#define STATE_MANAGER_TEST_CALIBRATION_HOOK(stage) ((void)(stage))
#endif

#ifndef STATE_MANAGER_TEST_FAULT_CAS_HOOK
#define STATE_MANAGER_TEST_FAULT_CAS_HOOK(attempt, observed) \
    do                                                        \
    {                                                         \
        (void)(attempt);                                      \
        (void)(observed);                                     \
    } while (0)
#endif

#ifndef STATE_MANAGER_TEST_FATAL_HOOK
#define STATE_MANAGER_TEST_FATAL_HOOK() ((void)0)
#endif

#ifndef STATE_MANAGER_TEST_TRANSITION_HOOK
#define STATE_MANAGER_TEST_TRANSITION_HOOK(stage) ((void)(stage))
#endif

_Alignas(4) STATIC uint32 StateManager_State =
    (uint32)STATE_MANAGER_RESET;
_Alignas(4) STATIC uint32 StateManager_FaultMask;
_Alignas(4) STATIC uint32 StateManager_ClassifiedPendingMask;
_Alignas(4) STATIC uint32 StateManager_AliveCounter;
STATIC uint16 StateManager_LastMotorBreakSequence;
STATIC uint16 StateManager_LastMotorUpdateSequence;
STATIC uint16 StateManager_LastMotorAdcSequence;
STATIC uint16 StateManager_LastMotorCalibrationSequence;
STATIC uint16 StateManager_LastSafetySequence;
STATIC uint16 StateManager_LastWdgMSequence;
STATIC uint32 StateManager_CalibrationEntryTick;
STATIC uint32 StateManager_CalibrationActivation;
STATIC boolean StateManager_CalibrationDeadlineArmed;
STATIC boolean StateManager_CalibrationStarted;
STATIC SafetySupervisor_SnapshotType StateManager_LatestSafetySnapshot;
STATIC boolean StateManager_LatestSafetySnapshotValid;

_Static_assert(__atomic_always_lock_free(sizeof(uint32), 0),
               "32-bit StateManager atomics must always be lock-free");
_Static_assert((STATE_MANAGER_CRITICAL_CLASS_ALL &
                STATE_MANAGER_CONTROL_STATE_MASK) == 0U,
               "critical classes must not overlap operating-state bits");
_Static_assert((STATE_MANAGER_CRITICAL_CLASS_ALL &
                STATE_MANAGER_CONTROL_RESERVED_MASK) == 0U,
               "critical classes must not overlap reserved control bits");

typedef struct
{
    uint32 clockEvents;
    MotorControlCdd_FaultEventsType motorEvents;
    uint32 safetyEvent;
    uint32 wdgMEvent;
} StateManager_GlobalEventsType;

STATIC Std_ReturnType StateManager_SnapshotGlobalEvents(
    StateManager_GlobalEventsType *events)
{
    if ((events == NULL_PTR) ||
        (MotorControlCdd_GetFaultEvents(&events->motorEvents) != E_OK))
    {
        return E_NOT_OK;
    }
    events->clockEvents = Mcu_GetClockFaultEvents();
    events->safetyEvent = SafetySupervisor_GetFaultEvent();
    events->wdgMEvent = WdgM_GetFaultEvent();
    return E_OK;
}

STATIC boolean StateManager_GlobalEventsAreEqual(
    const StateManager_GlobalEventsType *left,
    const StateManager_GlobalEventsType *right)
{
    if ((left->clockEvents == right->clockEvents) &&
        (left->motorEvents.breakEvent == right->motorEvents.breakEvent) &&
        (left->motorEvents.updateEvent == right->motorEvents.updateEvent) &&
        (left->motorEvents.adcEvent == right->motorEvents.adcEvent) &&
        (left->motorEvents.calibrationEvent ==
         right->motorEvents.calibrationEvent) &&
        (left->safetyEvent == right->safetyEvent) &&
        (left->wdgMEvent == right->wdgMEvent))
    {
        return TRUE;
    }
    return FALSE;
}

STATIC boolean StateManager_IdentityIsEqual(
    const Sc1_MeasurementIdentityType *left,
    const Sc1_MeasurementIdentityType *right)
{
    uint32 index;

    if ((left->schemaVersion != right->schemaVersion) ||
        (left->boardProfileCode != right->boardProfileCode))
    {
        return FALSE;
    }
    for (index = 0U;
         index < SC1_MEASUREMENT_IDENTITY_DIGEST_WORD_COUNT;
         index++)
    {
        if (left->evidenceSha256[index] != right->evidenceSha256[index])
        {
            return FALSE;
        }
    }
    return TRUE;
}

STATIC boolean StateManager_TakeNewEvent(
    uint32 event,
    uint16 *lastSeenSequence)
{
    uint16 cause = (uint16)(event & 0xFFFFU);
    uint16 sequence = (uint16)(event >> 16U);

    if ((cause == 0U) ||
        ((uint16)(sequence - *lastSeenSequence) == 0U))
    {
        return FALSE;
    }

    *lastSeenSequence = sequence;
    return TRUE;
}

STATIC uint32 StateManager_ConsumeNewFaults(
    const StateManager_GlobalEventsType *events)
{
    uint32 faultMask = STATE_MANAGER_FAULT_NONE;

    if (events->clockEvents != MCU_CLOCK_FAULT_EVENT_NONE)
    {
        faultMask |= STATE_MANAGER_FAULT_CLOCK;
    }
    if (StateManager_TakeNewEvent(
            events->motorEvents.breakEvent,
            &StateManager_LastMotorBreakSequence) == TRUE)
    {
        faultMask |= STATE_MANAGER_FAULT_MOTOR_BREAK;
    }
    if (StateManager_TakeNewEvent(
            events->motorEvents.updateEvent,
            &StateManager_LastMotorUpdateSequence) == TRUE)
    {
        faultMask |= STATE_MANAGER_FAULT_MOTOR_UPDATE;
    }
    if (StateManager_TakeNewEvent(
            events->motorEvents.adcEvent,
            &StateManager_LastMotorAdcSequence) == TRUE)
    {
        faultMask |= STATE_MANAGER_FAULT_MOTOR_ADC;
    }
    if (StateManager_TakeNewEvent(
            events->motorEvents.calibrationEvent,
            &StateManager_LastMotorCalibrationSequence) == TRUE)
    {
        faultMask |= STATE_MANAGER_FAULT_CALIBRATION;
    }
    if (StateManager_TakeNewEvent(
            events->safetyEvent,
            &StateManager_LastSafetySequence) == TRUE)
    {
        faultMask |= STATE_MANAGER_FAULT_SAFETY_ADC;
    }
    if (StateManager_TakeNewEvent(
            events->wdgMEvent,
            &StateManager_LastWdgMSequence) == TRUE)
    {
        faultMask |= STATE_MANAGER_FAULT_WDGM;
    }
    return faultMask;
}

STATIC void StateManager_RequestCalibrationAbortForFault(uint32 faultMask)
{
    MotorControlCdd_CalibrationAbortModeType mode =
        MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP;

    if ((faultMask & STATE_MANAGER_FAULT_CLOCK) != 0U)
    {
        mode = MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC;
    }
    (void)MotorControlCdd_RequestCalibrationAbort(mode);
    StateManager_CalibrationDeadlineArmed = FALSE;
}

LOCAL_INLINE StateManager_StateType StateManager_ControlState(uint32 control)
{
    return (StateManager_StateType)(control &
                                    STATE_MANAGER_CONTROL_STATE_MASK);
}

LOCAL_INLINE uint32 StateManager_ControlPending(uint32 control)
{
    return control & STATE_MANAGER_CRITICAL_CLASS_ALL;
}

STATIC void StateManager_EnterControlWordFatal(void)
{
    MotorControlCdd_EmergencyOff();
    STATE_MANAGER_TEST_FATAL_HOOK();
    for (;;)
    {
        /* Deliberately starve watchdog refresh until the IWDG resets. */
    }
}

STATIC void StateManager_ForceFaultState(void)
{
    uint32 observed = __atomic_load_n(&StateManager_State,
                                      __ATOMIC_ACQUIRE);
    uint32 attempt;

    for (attempt = 0U;
         attempt < STATE_MANAGER_FAULT_LATCH_ATTEMPTS;
         attempt++)
    {
        uint32 desired =
            (observed & ~STATE_MANAGER_CONTROL_STATE_MASK) |
            (uint32)STATE_MANAGER_FAULT_LATCHED;

        STATE_MANAGER_TEST_FAULT_CAS_HOOK(attempt, observed);
        if (__atomic_compare_exchange_n(
                &StateManager_State, &observed, desired, FALSE,
                __ATOMIC_RELEASE, __ATOMIC_ACQUIRE) == TRUE)
        {
            return;
        }
    }

    StateManager_EnterControlWordFatal();
}

STATIC void StateManager_LatchFault(uint32 faultMask)
{
    (void)__atomic_fetch_or(&StateManager_FaultMask, faultMask,
                            __ATOMIC_RELEASE);
    MotorControlCdd_EmergencyOff();
    StateManager_ForceFaultState();
}

STATIC boolean StateManager_ProcessObservedPending(uint32 observedControl)
{
    StateManager_GlobalEventsType events;
    StateManager_StateType rawState =
        StateManager_ControlState(observedControl);
    uint32 pending = StateManager_ControlPending(observedControl);
    uint32 classified = __atomic_load_n(
        &StateManager_ClassifiedPendingMask, __ATOMIC_ACQUIRE);
    uint32 unclassified = pending & ~classified;
    uint32 previousExact = __atomic_load_n(&StateManager_FaultMask,
                                           __ATOMIC_ACQUIRE);
    uint32 exactFaults;

    if (StateManager_SnapshotGlobalEvents(&events) != E_OK)
    {
        exactFaults = STATE_MANAGER_FAULT_INVARIANT;
    }
    else
    {
        exactFaults = StateManager_ConsumeNewFaults(&events);
    }

    if (((unclassified & STATE_MANAGER_CRITICAL_CLASS_CLOCK) != 0U) &&
        ((exactFaults & STATE_MANAGER_FAULT_CLOCK) == 0U))
    {
        exactFaults |= STATE_MANAGER_FAULT_INVARIANT;
    }
    if (((unclassified & STATE_MANAGER_CRITICAL_CLASS_MOTOR) != 0U) &&
        ((exactFaults & STATE_MANAGER_MOTOR_DETAIL_FAULT_MASK) == 0U))
    {
        exactFaults |= STATE_MANAGER_FAULT_INVARIANT;
    }
    if (((unclassified & STATE_MANAGER_CRITICAL_CLASS_SAFETY) != 0U) &&
        ((exactFaults & STATE_MANAGER_FAULT_SAFETY_ADC) == 0U))
    {
        exactFaults |= STATE_MANAGER_FAULT_INVARIANT;
    }
    if (((unclassified & STATE_MANAGER_CRITICAL_CLASS_WDGM) != 0U) &&
        ((exactFaults & STATE_MANAGER_FAULT_WDGM) == 0U))
    {
        exactFaults |= STATE_MANAGER_FAULT_INVARIANT;
    }
    if ((unclassified & STATE_MANAGER_CRITICAL_CLASS_INVARIANT) != 0U)
    {
        exactFaults |= STATE_MANAGER_FAULT_INVARIANT;
    }

    if (rawState == STATE_MANAGER_CALIBRATING)
    {
        StateManager_RequestCalibrationAbortForFault(exactFaults);
    }
    (void)__atomic_fetch_or(&StateManager_FaultMask, exactFaults,
                            __ATOMIC_RELEASE);
    (void)__atomic_fetch_or(&StateManager_ClassifiedPendingMask, pending,
                            __ATOMIC_RELEASE);

    if ((unclassified != 0U) ||
        (rawState != STATE_MANAGER_FAULT_LATCHED) ||
        ((exactFaults & ~previousExact) != 0U))
    {
        StateManager_LatchFault(STATE_MANAGER_FAULT_NONE);
        return TRUE;
    }

    return FALSE;
}

STATIC void StateManager_ProcessSerialRequest(uint32 currentTick)
{
    const SerialCommunicationCdd_RequestType *request;
    SerialCommunicationCdd_ResponseType response;

    if (SerialCommunicationCdd_RequestAcquire(&request) != E_OK)
    {
        return;
    }

    response.command = request->command;
    response.sessionId = request->sessionId;
    response.sequence = request->sequence;
    response.status = SERIAL_COMMUNICATION_CDD_STATUS_INTERNAL;
    response.detail = SERIAL_COMMUNICATION_CDD_DETAIL_NONE;
    response.data0 = 0U;
    response.data1 = 0U;

    if (request->command == SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ)
    {
        if (request->diagnosticSelector == 0U)
        {
            response.status = SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED;
            response.detail = SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_STATE;
            response.data0 = (uint32)StateManager_GetState();
            response.data1 = StateManager_GetFaultMask();
        }
        else if (request->diagnosticSelector == 1U)
        {
            SerialCommunicationCdd_DiagnosticsType diagnostics;

            if (SerialCommunicationCdd_GetDiagnostics(&diagnostics) == E_OK)
            {
                response.status = SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED;
                response.detail =
                    SERIAL_COMMUNICATION_CDD_DETAIL_LINK_DIAGNOSTICS;
                response.data0 =
                    ((uint32)diagnostics.identityOrSequenceRejects << 16U) |
                    (uint32)diagnostics.frameOrCrcRejects;
                response.data1 =
                    ((uint32)diagnostics.mailboxBusyOrResponseDrops << 16U) |
                    (uint32)diagnostics.usartErrorOrRingDrops;
            }
        }
        else
        {
            response.status = SERIAL_COMMUNICATION_CDD_STATUS_INVALID_STATE;
        }
    }
    else if (request->command == SERIAL_COMMUNICATION_CDD_COMMAND_STOP)
    {
        response.status = SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED;
        response.detail = SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_STATE;
        response.data0 = (uint32)StateManager_GetState();
    }
    else if ((request->command == SERIAL_COMMUNICATION_CDD_COMMAND_START) ||
             (request->command ==
              SERIAL_COMMUNICATION_CDD_COMMAND_SET_CURRENT))
    {
        if ((uint32)(currentTick - request->receiveTick) >=
            (uint32)request->leaseMs)
        {
            response.status = SERIAL_COMMUNICATION_CDD_STATUS_NOT_READY;
            response.detail = SERIAL_COMMUNICATION_CDD_DETAIL_EXPIRED_LEASE;
        }
        else
        {
            Rte_CommandPolicyRequestType policyRequest;
            Rte_CommandPolicyDecisionType policyDecision;

            policyRequest.command = (uint8)request->command;
            policyRequest.leaseMs = request->leaseMs;
            policyRequest.currentMa = request->currentMa;
            policyRequest.stateSnapshot = (uint32)StateManager_GetState();
            if ((Rte_CommandPolicyEvaluate(&policyRequest, &policyDecision) !=
                 E_OK) ||
                (policyDecision != RTE_COMMAND_POLICY_NOT_READY))
            {
                response.status = SERIAL_COMMUNICATION_CDD_STATUS_INTERNAL;
            }
            else
            {
                /* The approved zero envelope has no START/FOC authority. */
                response.status = SERIAL_COMMUNICATION_CDD_STATUS_NOT_READY;
                response.detail =
                    SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_STATE;
                response.data0 = (uint32)StateManager_GetState();
            }
        }
    }
    else
    {
        response.status = SERIAL_COMMUNICATION_CDD_STATUS_INVALID_STATE;
    }

    (void)SerialCommunicationCdd_ResponseQueue(&response);
    (void)SerialCommunicationCdd_RequestRelease(request);
}

void StateManager_NotifyCriticalFault(uint32 CriticalClassMask)
{
    uint32 publishMask =
        CriticalClassMask & STATE_MANAGER_CRITICAL_CLASS_ALL;

    if ((CriticalClassMask == 0U) ||
        ((CriticalClassMask & ~STATE_MANAGER_CRITICAL_CLASS_ALL) != 0U))
    {
        publishMask |= STATE_MANAGER_CRITICAL_CLASS_INVARIANT;
    }

    (void)__atomic_fetch_or(&StateManager_State, publishMask,
                            __ATOMIC_RELEASE);
}

Std_ReturnType StateManager_InitBootSafe(void)
{
    uint32 expectedState = (uint32)STATE_MANAGER_RESET;

    if (__atomic_load_n(&StateManager_State, __ATOMIC_RELAXED) !=
        (uint32)STATE_MANAGER_RESET)
    {
        return E_NOT_OK;
    }

    __atomic_store_n(&StateManager_FaultMask, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&StateManager_ClassifiedPendingMask, 0U,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&StateManager_AliveCounter, 0U, __ATOMIC_RELAXED);
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
    STATE_MANAGER_TEST_TRANSITION_HOOK(
        STATE_MANAGER_TEST_TRANSITION_BOOT_SAFE);
    if (__atomic_compare_exchange_n(
            &StateManager_State, &expectedState,
            (uint32)STATE_MANAGER_BOOT_SAFE, FALSE,
            __ATOMIC_RELEASE, __ATOMIC_ACQUIRE) != TRUE)
    {
        return E_NOT_OK;
    }
    return E_OK;
}

Std_ReturnType StateManager_EnterSelfTest(void)
{
    uint32 expectedState = (uint32)STATE_MANAGER_BOOT_SAFE;
    uint32 observedState = __atomic_load_n(&StateManager_State,
                                           __ATOMIC_ACQUIRE);

    if (observedState != expectedState)
    {
        if (StateManager_ControlPending(observedState) != 0U)
        {
            (void)StateManager_ProcessObservedPending(observedState);
        }
        return E_NOT_OK;
    }

    STATE_MANAGER_TEST_TRANSITION_HOOK(
        STATE_MANAGER_TEST_TRANSITION_SELF_TEST);
    if (__atomic_compare_exchange_n(
            &StateManager_State, &expectedState,
            (uint32)STATE_MANAGER_SELF_TEST, FALSE,
            __ATOMIC_RELEASE, __ATOMIC_ACQUIRE) != TRUE)
    {
        if (StateManager_ControlPending(expectedState) != 0U)
        {
            (void)StateManager_ProcessObservedPending(expectedState);
        }
        else
        {
            StateManager_LatchFault(STATE_MANAGER_FAULT_INVARIANT);
        }
        return E_NOT_OK;
    }
    return E_OK;
}

void StateManager_MainFunction(void)
{
    StateManager_GlobalEventsType entryEvents;
    uint32 faultMask = STATE_MANAGER_FAULT_NONE;
    uint32 control;
    StateManager_StateType state;
    uint32 currentTick;

    (void)__atomic_fetch_add(&StateManager_AliveCounter, 1U,
                             __ATOMIC_RELEASE);
    control = __atomic_load_n(&StateManager_State, __ATOMIC_ACQUIRE);
    currentTick = Os_GetCounterValue();

    if (StateManager_ControlPending(control) != 0U)
    {
        if (StateManager_ProcessObservedPending(control) == TRUE)
        {
            return;
        }
        control = __atomic_load_n(&StateManager_State, __ATOMIC_ACQUIRE);
    }
    else
    {
        if (StateManager_SnapshotGlobalEvents(&entryEvents) != E_OK)
        {
            StateManager_LatchFault(STATE_MANAGER_FAULT_INVARIANT);
            return;
        }
        faultMask = StateManager_ConsumeNewFaults(&entryEvents);
    }

    state = StateManager_ControlState(control);

    if (state == STATE_MANAGER_FAULT_LATCHED)
    {
        if (faultMask != STATE_MANAGER_FAULT_NONE)
        {
            if (((faultMask & STATE_MANAGER_FAULT_CLOCK) != 0U) &&
                (MotorControlCdd_IsCalibrationCleanupPending() == TRUE))
            {
                (void)MotorControlCdd_RequestCalibrationAbort(
                    MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC);
            }
            StateManager_LatchFault(faultMask);
            return;
        }
        if (MotorControlCdd_IsCalibrationCleanupPending() == TRUE)
        {
            if (Mcu_GetClockFaultEvents() != MCU_CLOCK_FAULT_EVENT_NONE)
            {
                (void)MotorControlCdd_RequestCalibrationAbort(
                    MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC);
                return;
            }
            (void)MotorControlCdd_CalibrationCleanupStep();
            if (Mcu_GetClockFaultEvents() != MCU_CLOCK_FAULT_EVENT_NONE)
            {
                (void)MotorControlCdd_RequestCalibrationAbort(
                    MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC);
            }
            return;
        }
        return;
    }
    else if (faultMask != STATE_MANAGER_FAULT_NONE)
    {
        if (state == STATE_MANAGER_CALIBRATING)
        {
            StateManager_RequestCalibrationAbortForFault(faultMask);
        }
        StateManager_LatchFault(faultMask);
        return;
    }
    else if (state == STATE_MANAGER_SELF_TEST)
    {
        uint32 expectedState = (uint32)STATE_MANAGER_SELF_TEST;

        StateManager_CalibrationEntryTick = currentTick;
        StateManager_CalibrationActivation = 0U;
        StateManager_CalibrationDeadlineArmed = TRUE;
        StateManager_CalibrationStarted = FALSE;
        StateManager_LatestSafetySnapshotValid = FALSE;
        STATE_MANAGER_TEST_TRANSITION_HOOK(
            STATE_MANAGER_TEST_TRANSITION_CALIBRATING);
        if (__atomic_compare_exchange_n(
                &StateManager_State, &expectedState,
                (uint32)STATE_MANAGER_CALIBRATING, FALSE,
                __ATOMIC_RELEASE, __ATOMIC_ACQUIRE) != TRUE)
        {
            if (StateManager_ControlPending(expectedState) != 0U)
            {
                (void)StateManager_ProcessObservedPending(expectedState);
            }
            else
            {
                StateManager_LatchFault(STATE_MANAGER_FAULT_INVARIANT);
            }
            return;
        }
    }
    else if (state == STATE_MANAGER_CALIBRATING)
    {
        SafetySupervisor_SnapshotType snapshot;
        SafetySupervisor_ReadinessType safetyReadiness;
        MotorControlCdd_CalibrationReadinessType motorReadiness;
        StateManager_GlobalEventsType eventsAfterStep;
        boolean calibrationAdvancedFromActive = FALSE;

        StateManager_CalibrationActivation++;
        if ((StateManager_CalibrationDeadlineArmed == TRUE) &&
            (StateManager_CalibrationActivation >= 256U))
        {
            MotorControlCdd_EmergencyOff();
            (void)MotorControlCdd_RequestCalibrationAbort(
                MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP);
            StateManager_CalibrationDeadlineArmed = FALSE;
            StateManager_LatchFault(STATE_MANAGER_FAULT_CALIBRATION);
            return;
        }

        if (SafetySupervisor_GetLatestSnapshot(&snapshot) == E_OK)
        {
            MotorControlCdd_CalibrationEnvironmentType environment;
            Std_ReturnType calibrationResult;

            if ((snapshot.acquisitionValid != TRUE) ||
                ((uint32)(currentTick - snapshot.osTick) > 1U) ||
                (SafetySupervisor_GetReadiness(&safetyReadiness) != E_OK) ||
                (safetyReadiness.structuralValid != TRUE))
            {
                StateManager_RequestCalibrationAbortForFault(
                    STATE_MANAGER_FAULT_CALIBRATION);
                StateManager_LatchFault(STATE_MANAGER_FAULT_CALIBRATION);
                return;
            }

            StateManager_LatestSafetySnapshot = snapshot;
            StateManager_LatestSafetySnapshotValid = TRUE;
            environment.vbusRaw = snapshot.vbusRaw;
            environment.temperatureRaw = snapshot.temperatureRaw;
            environment.safetyAcquisitionSequence =
                snapshot.acquisitionSequence;
            environment.safetyTick = snapshot.osTick;
            environment.identity = safetyReadiness.identity;

            if (StateManager_CalibrationStarted != TRUE)
            {
                calibrationResult =
                    MotorControlCdd_StartCalibration(&environment);
                if (calibrationResult == E_OK)
                {
                    StateManager_CalibrationStarted = TRUE;
                }
            }
            else if (MotorControlCdd_IsCalibrationActive() == TRUE)
            {
                calibrationAdvancedFromActive = TRUE;
                calibrationResult =
                    MotorControlCdd_CalibrationStep(&environment);
            }
            else
            {
                calibrationResult = E_OK;
            }

            if (calibrationResult != E_OK)
            {
                if (StateManager_SnapshotGlobalEvents(&eventsAfterStep) != E_OK)
                {
                    faultMask = STATE_MANAGER_FAULT_INVARIANT;
                }
                else
                {
                    faultMask =
                        StateManager_ConsumeNewFaults(&eventsAfterStep);
                    if (faultMask == STATE_MANAGER_FAULT_NONE)
                    {
                        faultMask = STATE_MANAGER_FAULT_CALIBRATION;
                    }
                }
                StateManager_RequestCalibrationAbortForFault(faultMask);
                StateManager_LatchFault(faultMask);
                return;
            }
        }

        STATE_MANAGER_TEST_CALIBRATION_HOOK(
            STATE_MANAGER_TEST_CALIBRATION_AFTER_CDD);
        if (StateManager_SnapshotGlobalEvents(&eventsAfterStep) != E_OK)
        {
            StateManager_RequestCalibrationAbortForFault(
                STATE_MANAGER_FAULT_INVARIANT);
            StateManager_LatchFault(STATE_MANAGER_FAULT_INVARIANT);
            return;
        }
        faultMask = StateManager_ConsumeNewFaults(&eventsAfterStep);
        if (faultMask != STATE_MANAGER_FAULT_NONE)
        {
            StateManager_RequestCalibrationAbortForFault(faultMask);
            StateManager_LatchFault(faultMask);
            return;
        }

        if ((MotorControlCdd_GetCalibrationReadiness(&motorReadiness) != E_OK) ||
            (SafetySupervisor_GetReadiness(&safetyReadiness) != E_OK) ||
            (motorReadiness.structuralValid != TRUE) ||
            (safetyReadiness.structuralValid != TRUE) ||
            (StateManager_IdentityIsEqual(
                 &motorReadiness.identity,
                 &safetyReadiness.identity) != TRUE))
        {
            StateManager_RequestCalibrationAbortForFault(
                STATE_MANAGER_FAULT_CALIBRATION);
            StateManager_LatchFault(STATE_MANAGER_FAULT_CALIBRATION);
            return;
        }

        if (motorReadiness.calibrationComplete == TRUE)
        {
            StateManager_CalibrationDeadlineArmed = FALSE;
        }

        if ((motorReadiness.calibrationComplete == TRUE) &&
            (calibrationAdvancedFromActive != TRUE) &&
            (motorReadiness.measurementApproved == TRUE) &&
            (safetyReadiness.measurementApproved == TRUE) &&
            (motorReadiness.offsetsInstalled == TRUE) &&
            (StateManager_LatestSafetySnapshotValid == TRUE) &&
            (SafetySupervisor_IsSnapshotReady(
                 &StateManager_LatestSafetySnapshot, currentTick) == TRUE))
        {
            MotorControlCdd_CalibrationResultType result;
            StateManager_GlobalEventsType preStoreEvents;
            StateManager_GlobalEventsType postStoreEvents;
            uint32 expectedState = (uint32)STATE_MANAGER_CALIBRATING;
            uint32 finalControl;

            if ((MotorControlCdd_GetCalibrationResult(&result) != E_OK) ||
                (result.valid != TRUE) ||
                (result.offsetsInstalled != TRUE) ||
                (result.measurementApproved != TRUE) ||
                (((uint32)(StateManager_LatestSafetySnapshot.acquisitionSequence -
                           result.lastSafetySequence) == 0U) &&
                 (StateManager_LatestSafetySnapshot.osTick !=
                  result.lastSafetyTick)) ||
                (((uint32)(StateManager_LatestSafetySnapshot.acquisitionSequence -
                           result.lastSafetySequence) == 1U) &&
                 ((uint32)(StateManager_LatestSafetySnapshot.osTick -
                           result.lastSafetyTick) != 1U)) ||
                ((uint32)(StateManager_LatestSafetySnapshot.acquisitionSequence -
                          result.lastSafetySequence) > 1U) ||
                (StateManager_IdentityIsEqual(
                     &result.identity, &motorReadiness.identity) != TRUE))
            {
                StateManager_RequestCalibrationAbortForFault(
                    STATE_MANAGER_FAULT_CALIBRATION);
                StateManager_LatchFault(STATE_MANAGER_FAULT_CALIBRATION);
                return;
            }

            if (StateManager_SnapshotGlobalEvents(&preStoreEvents) != E_OK)
            {
                StateManager_RequestCalibrationAbortForFault(
                    STATE_MANAGER_FAULT_INVARIANT);
                StateManager_LatchFault(STATE_MANAGER_FAULT_INVARIANT);
                return;
            }
            if (StateManager_GlobalEventsAreEqual(
                    &entryEvents, &preStoreEvents) != TRUE)
            {
                faultMask = StateManager_ConsumeNewFaults(&preStoreEvents);
                if (faultMask == STATE_MANAGER_FAULT_NONE)
                {
                    faultMask = STATE_MANAGER_FAULT_INVARIANT;
                }
                StateManager_RequestCalibrationAbortForFault(faultMask);
                StateManager_LatchFault(faultMask);
                return;
            }

            STATE_MANAGER_TEST_CALIBRATION_HOOK(
                STATE_MANAGER_TEST_CALIBRATION_PRE_READY);
            if (__atomic_compare_exchange_n(
                    &StateManager_State, &expectedState,
                    (uint32)STATE_MANAGER_READY, FALSE,
                    __ATOMIC_RELEASE, __ATOMIC_ACQUIRE) != TRUE)
            {
                if (StateManager_ControlPending(expectedState) != 0U)
                {
                    (void)StateManager_ProcessObservedPending(expectedState);
                }
                else
                {
                    StateManager_RequestCalibrationAbortForFault(
                        STATE_MANAGER_FAULT_INVARIANT);
                    StateManager_LatchFault(
                        STATE_MANAGER_FAULT_INVARIANT);
                }
                return;
            }
            STATE_MANAGER_TEST_CALIBRATION_HOOK(
                STATE_MANAGER_TEST_CALIBRATION_POST_READY);
            finalControl = __atomic_load_n(&StateManager_State,
                                           __ATOMIC_ACQUIRE);
            if (StateManager_ControlPending(finalControl) != 0U)
            {
                (void)StateManager_ProcessObservedPending(finalControl);
                return;
            }
            if (StateManager_SnapshotGlobalEvents(&postStoreEvents) != E_OK)
            {
                faultMask = STATE_MANAGER_FAULT_INVARIANT;
                StateManager_LatchFault(faultMask);
            }
            else if (StateManager_GlobalEventsAreEqual(
                         &preStoreEvents, &postStoreEvents) != TRUE)
            {
                faultMask = StateManager_ConsumeNewFaults(&postStoreEvents);
                if (faultMask == STATE_MANAGER_FAULT_NONE)
                {
                    faultMask = STATE_MANAGER_FAULT_INVARIANT;
                }
                StateManager_LatchFault(faultMask);
            }
            return;
        }
    }
    else if ((state != STATE_MANAGER_BOOT_SAFE) &&
             (state != STATE_MANAGER_READY) &&
             (state != STATE_MANAGER_FAULT_LATCHED))
    {
        StateManager_LatchFault(STATE_MANAGER_FAULT_INVARIANT);
        return;
    }

    (void)StateManager_CalibrationEntryTick;
    SerialCommunicationCdd_MainFunction(currentTick);
    StateManager_ProcessSerialRequest(currentTick);
}

StateManager_StateType StateManager_GetState(void)
{
    uint32 control = __atomic_load_n(&StateManager_State,
                                     __ATOMIC_ACQUIRE);

    if (StateManager_ControlPending(control) != 0U)
    {
        return STATE_MANAGER_FAULT_LATCHED;
    }
    return StateManager_ControlState(control);
}

uint32 StateManager_GetFaultMask(void)
{
    uint32 classified = __atomic_load_n(
        &StateManager_ClassifiedPendingMask, __ATOMIC_ACQUIRE);
    uint32 faultMask = __atomic_load_n(&StateManager_FaultMask,
                                       __ATOMIC_ACQUIRE);
    uint32 control = __atomic_load_n(&StateManager_State,
                                     __ATOMIC_ACQUIRE);
    uint32 provisional =
        StateManager_ControlPending(control) & ~classified;

    if ((provisional & STATE_MANAGER_CRITICAL_CLASS_CLOCK) != 0U)
    {
        faultMask |= STATE_MANAGER_FAULT_CLOCK;
    }
    if ((provisional & STATE_MANAGER_CRITICAL_CLASS_MOTOR) != 0U)
    {
        faultMask |= STATE_MANAGER_FAULT_UNCLASSIFIED_CRITICAL;
    }
    if ((provisional & STATE_MANAGER_CRITICAL_CLASS_SAFETY) != 0U)
    {
        faultMask |= STATE_MANAGER_FAULT_SAFETY_ADC;
    }
    if ((provisional & STATE_MANAGER_CRITICAL_CLASS_WDGM) != 0U)
    {
        faultMask |= STATE_MANAGER_FAULT_WDGM;
    }
    if ((provisional & STATE_MANAGER_CRITICAL_CLASS_INVARIANT) != 0U)
    {
        faultMask |= STATE_MANAGER_FAULT_INVARIANT;
    }
    return faultMask;
}

uint32 StateManager_GetAliveCounter(void)
{
    return __atomic_load_n(&StateManager_AliveCounter, __ATOMIC_ACQUIRE);
}
