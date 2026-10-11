#include "MotorControlStartup.h"

STATIC boolean MotorControlStartup_IsQ15(sint32 value)
{
    return ((value >= MOTOR_CONTROL_STARTUP_Q15_MIN) &&
            (value <= MOTOR_CONTROL_STARTUP_Q15_MAX)) ? TRUE : FALSE;
}

STATIC boolean MotorControlStartup_IsBoolean(boolean value)
{
    return ((value == FALSE) || (value == TRUE)) ? TRUE : FALSE;
}

STATIC uint32 MotorControlStartup_Abs(sint32 value)
{
    sint64 promoted = (sint64)value;

    return (uint32)((promoted < 0) ? -promoted : promoted);
}

STATIC uint64 MotorControlStartup_CoreDuration(
    const MotorControlStartup_ConfigType *config)
{
    return (uint64)config->alignCycles +
           (uint64)config->openLoopRampCycles +
           (uint64)config->qualificationCycles +
           (uint64)config->handoverCycles +
           (uint64)config->outputConfirmCycles;
}

STATIC boolean MotorControlStartup_ConfigValid(
    const MotorControlStartup_ConfigType *config)
{
    uint64 duration;

    if (config == NULL_PTR)
    {
        return FALSE;
    }

    duration = MotorControlStartup_CoreDuration(config);
    return ((config->alignCycles > 0U) &&
            (config->openLoopRampCycles > 0U) &&
            (config->qualificationCycles > 0U) &&
            (config->handoverCycles > 0U) &&
            (config->outputConfirmCycles > 0U) &&
            (duration < (uint64)MOTOR_CONTROL_STARTUP_MAX_CYCLES) &&
            (MotorControlStartup_IsQ15(config->alignDRefQ15) == TRUE) &&
            (MotorControlStartup_IsQ15(config->alignQRefQ15) == TRUE) &&
            (MotorControlStartup_IsQ15(config->finalDRefQ15) == TRUE) &&
            (MotorControlStartup_IsQ15(config->finalQRefQ15) == TRUE) &&
            (config->absoluteCurrentCapQ15 > 0) &&
            (config->absoluteCurrentCapQ15 <=
             MOTOR_CONTROL_STARTUP_Q15_MAX) &&
            (MotorControlStartup_Abs(config->alignDRefQ15) <=
             (uint32)config->absoluteCurrentCapQ15) &&
            (MotorControlStartup_Abs(config->alignQRefQ15) <=
             (uint32)config->absoluteCurrentCapQ15) &&
            (MotorControlStartup_Abs(config->finalDRefQ15) <=
             (uint32)config->absoluteCurrentCapQ15) &&
            (MotorControlStartup_Abs(config->finalQRefQ15) <=
             (uint32)config->absoluteCurrentCapQ15) &&
            (MotorControlStartup_IsQ15(
                 config->finalOpenLoopSpeedAngleCounts) == TRUE) &&
            (config->finalOpenLoopSpeedAngleCounts != 0) &&
            (config->qualificationRequiredCycles > 0U) &&
            (config->qualificationRequiredCycles <=
             config->qualificationCycles) &&
            (config->minimumObservableSpeedAngleCounts > 0U) &&
            (config->minimumObservableSpeedAngleCounts <=
             MOTOR_CONTROL_STARTUP_ANGLE_HALF_TURN) &&
            (config->qualificationMaxAngleErrorQ16 <=
             MOTOR_CONTROL_STARTUP_ANGLE_HALF_TURN) &&
            (config->qualificationMaxSpeedErrorAngleCounts <=
             MOTOR_CONTROL_STARTUP_ANGLE_HALF_TURN) &&
            (config->handoverMaxAngleCorrectionQ16 > 0U) &&
            (config->handoverMaxAngleCorrectionQ16 <=
             MOTOR_CONTROL_STARTUP_ANGLE_HALF_TURN) &&
            (config->handoverMaxSpeedSlewAngleCounts > 0U) &&
            (config->handoverMaxSpeedSlewAngleCounts <=
             MOTOR_CONTROL_STARTUP_ANGLE_HALF_TURN) &&
            (config->finalMaxAngleErrorQ16 <=
             config->handoverMaxAngleCorrectionQ16) &&
            (config->finalMaxSpeedErrorAngleCounts <=
             MOTOR_CONTROL_STARTUP_ANGLE_HALF_TURN)) ? TRUE : FALSE;
}

STATIC void MotorControlStartup_ClearObject(
    MotorControlStartup_Type *object)
{
    uint32 byteIndex;
    uint8 *bytes = (uint8 *)object;

    for (byteIndex = 0U;
         byteIndex < (uint32)sizeof(MotorControlStartup_Type);
         byteIndex++)
    {
        bytes[byteIndex] = 0U;
    }
}

STATIC void MotorControlStartup_ClearResult(
    MotorControlStartup_ResultType *result)
{
    if (result != NULL_PTR)
    {
        result->phase = MOTOR_CONTROL_STARTUP_PHASE_IDLE;
        result->selectedAngleQ16 = 0U;
        result->selectedSpeedAngleCounts = 0;
        result->dRefQ15 = 0;
        result->qRefQ15 = 0;
        result->observerQualified = FALSE;
        result->handoverComplete = FALSE;
        result->stopRequired = FALSE;
        result->readyForRun = FALSE;
        result->failure = MOTOR_CONTROL_STARTUP_FAILURE_NONE;
    }
}

STATIC void MotorControlStartup_PrepareRamp(
    MotorControlStartup_RampType *ramp,
    sint32 start,
    sint32 target,
    uint32 duration)
{
    sint64 difference = (sint64)target - (sint64)start;
    uint32 magnitude;

    ramp->current = start;
    ramp->target = target;
    ramp->direction = (difference > 0) ? 1 :
                      ((difference < 0) ? -1 : 0);
    magnitude = (uint32)((difference < 0) ? -difference : difference);
    ramp->baseIncrement = magnitude / duration;
    ramp->remainder = magnitude % duration;
    ramp->error = 0U;
    ramp->duration = duration;
    ramp->elapsed = 0U;
}

STATIC void MotorControlStartup_ResetRamp(MotorControlStartup_RampType *ramp)
{
    ramp->current = ramp->target -
        (ramp->direction * (sint32)
         ((ramp->baseIncrement * ramp->duration) + ramp->remainder));
    ramp->error = 0U;
    ramp->elapsed = 0U;
}

STATIC sint32 MotorControlStartup_AdvanceRamp(
    MotorControlStartup_RampType *ramp)
{
    sint64 next;

    next = (sint64)ramp->current +
           ((sint64)ramp->direction * (sint64)ramp->baseIncrement);
    ramp->error += ramp->remainder;
    if (ramp->error >= ramp->duration)
    {
        next += ramp->direction;
        ramp->error -= ramp->duration;
    }
    ramp->elapsed++;
    ramp->current = (sint32)next;
    if (ramp->elapsed == ramp->duration)
    {
        ramp->current = ramp->target;
    }
    return ramp->current;
}

STATIC sint32 MotorControlStartup_CircularDelta(
    uint16 target,
    uint16 current)
{
    uint16 raw = (uint16)(target - current);

    return (raw <= 32767U) ? (sint32)raw :
                             ((sint32)raw - (sint32)65536);
}

STATIC boolean MotorControlStartup_ObserverFlagsValid(
    const MotorControlStartup_ObserverSampleType *observer)
{
    return ((observer->numericValid == TRUE) &&
            (observer->angleValid == TRUE) &&
            (observer->speedValid == TRUE) &&
            (observer->varianceValid == TRUE) &&
            (observer->bemfValid == TRUE) &&
            (observer->currentValid == TRUE) &&
            (observer->directionValid == TRUE)) ? TRUE : FALSE;
}

STATIC boolean MotorControlStartup_ObserverBaseValid(
    const MotorControlStartup_Type *object,
    const MotorControlStartup_ObserverSampleType *observer)
{
    boolean directionMatches;

    directionMatches =
        (((object->config.finalOpenLoopSpeedAngleCounts > 0) &&
          (observer->speedAngleCounts > 0)) ||
         ((object->config.finalOpenLoopSpeedAngleCounts < 0) &&
          (observer->speedAngleCounts < 0))) ? TRUE : FALSE;
    return ((MotorControlStartup_ObserverFlagsValid(observer) == TRUE) &&
            (MotorControlStartup_Abs(observer->speedAngleCounts) >=
             object->config.minimumObservableSpeedAngleCounts) &&
            (directionMatches == TRUE)) ? TRUE : FALSE;
}

STATIC boolean MotorControlStartup_QualificationValid(
    const MotorControlStartup_Type *object,
    const MotorControlStartup_ObserverSampleType *observer)
{
    sint32 angleError = MotorControlStartup_CircularDelta(
        observer->angleQ16, object->selectedAngleQ16);
    sint64 speedError = (sint64)observer->speedAngleCounts -
                        (sint64)object->selectedSpeedAngleCounts;
    uint64 speedMagnitude = (uint64)((speedError < 0) ?
        -speedError : speedError);

    return ((MotorControlStartup_ObserverBaseValid(object, observer) == TRUE) &&
            (MotorControlStartup_Abs(angleError) <=
             object->config.qualificationMaxAngleErrorQ16) &&
            (speedMagnitude <=
             (uint64)object->config.qualificationMaxSpeedErrorAngleCounts)) ?
           TRUE : FALSE;
}

STATIC boolean MotorControlStartup_FinalObserverErrorValid(
    const MotorControlStartup_Type *object,
    const MotorControlStartup_ObserverSampleType *observer)
{
    sint32 angleError = MotorControlStartup_CircularDelta(
        observer->angleQ16, object->selectedAngleQ16);
    sint64 speedError = (sint64)observer->speedAngleCounts -
                        (sint64)object->selectedSpeedAngleCounts;
    uint64 speedMagnitude = (uint64)((speedError < 0) ?
        -speedError : speedError);

    return ((MotorControlStartup_Abs(angleError) <=
             object->config.finalMaxAngleErrorQ16) &&
            (speedMagnitude <=
             (uint64)object->config.finalMaxSpeedErrorAngleCounts)) ?
           TRUE : FALSE;
}

STATIC void MotorControlStartup_FillResult(
    const MotorControlStartup_Type *object,
    MotorControlStartup_PhaseType resultPhase,
    boolean stopRequired,
    MotorControlStartup_ResultType *result)
{
    result->phase = resultPhase;
    result->selectedAngleQ16 = object->selectedAngleQ16;
    result->selectedSpeedAngleCounts = object->selectedSpeedAngleCounts;
    result->dRefQ15 = object->dRefQ15;
    result->qRefQ15 = object->qRefQ15;
    result->observerQualified = object->observerQualified;
    result->handoverComplete = object->handoverComplete;
    result->stopRequired = stopRequired;
    result->readyForRun = object->readyForRun;
    result->failure = object->failure;
}

STATIC Std_ReturnType MotorControlStartup_LatchFailure(
    MotorControlStartup_Type *object,
    MotorControlStartup_FailureType failure,
    MotorControlStartup_ResultType *result)
{
    object->phase = MOTOR_CONTROL_STARTUP_PHASE_FAILED;
    object->failure = failure;
    object->dRefQ15 = 0;
    object->qRefQ15 = 0;
    object->observerQualified = FALSE;
    object->handoverComplete = FALSE;
    object->readyForRun = FALSE;
    MotorControlStartup_FillResult(
        object, MOTOR_CONTROL_STARTUP_PHASE_FAILED, FALSE, result);
    return E_NOT_OK;
}

STATIC boolean MotorControlStartup_InputRangesValid(
    const MotorControlStartup_StepInputType *input)
{
    const MotorControlStartup_ObserverSampleType *observer = &input->observer;

    return ((MotorControlStartup_IsQ15(observer->speedAngleCounts) == TRUE) &&
            (MotorControlStartup_IsBoolean(observer->numericValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(observer->angleValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(observer->speedValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(observer->varianceValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(observer->bemfValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(observer->currentValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(observer->directionValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(input->generationValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(input->safetyValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(
                 input->commandContinueValid) == TRUE) &&
            (MotorControlStartup_IsBoolean(input->stopRequested) == TRUE)) ?
           TRUE : FALSE;
}

STATIC boolean MotorControlStartup_RuntimeValid(
    const MotorControlStartup_Type *object)
{
    const MotorControlStartup_ConfigType *config;
    sint64 dDifference;
    sint64 qDifference;
    sint64 speedDifference;
    uint64 dMagnitude;
    uint64 qMagnitude;
    uint64 speedMagnitude;
    uint32 terminalPhaseLimit;
    boolean phaseCountersValid;

    if ((object->initialized != TRUE) ||
        (MotorControlStartup_ConfigValid(&object->config) != TRUE))
    {
        return FALSE;
    }

    config = &object->config;
    dDifference = (sint64)config->finalDRefQ15 -
                  (sint64)config->alignDRefQ15;
    qDifference = (sint64)config->finalQRefQ15 -
                  (sint64)config->alignQRefQ15;
    speedDifference = (sint64)config->finalOpenLoopSpeedAngleCounts;
    dMagnitude = (uint64)((dDifference < 0) ? -dDifference : dDifference);
    qMagnitude = (uint64)((qDifference < 0) ? -qDifference : qDifference);
    speedMagnitude = (uint64)((speedDifference < 0) ?
        -speedDifference : speedDifference);

    terminalPhaseLimit = config->alignCycles;
    if (config->openLoopRampCycles > terminalPhaseLimit)
    {
        terminalPhaseLimit = config->openLoopRampCycles;
    }
    if (config->qualificationCycles > terminalPhaseLimit)
    {
        terminalPhaseLimit = config->qualificationCycles;
    }
    if (config->handoverCycles > terminalPhaseLimit)
    {
        terminalPhaseLimit = config->handoverCycles;
    }
    if (config->outputConfirmCycles > terminalPhaseLimit)
    {
        terminalPhaseLimit = config->outputConfirmCycles;
    }

    if (object->phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE)
    {
        phaseCountersValid =
            ((object->phaseCycle == 0U) &&
             (object->dRefRamp.elapsed == 0U) &&
             (object->qRefRamp.elapsed == 0U) &&
             (object->speedRamp.elapsed == 0U)) ? TRUE : FALSE;
    }
    else if (object->phase == MOTOR_CONTROL_STARTUP_PHASE_ALIGN)
    {
        phaseCountersValid =
            ((object->phaseCycle < config->alignCycles) &&
             (object->dRefRamp.elapsed == 0U) &&
             (object->qRefRamp.elapsed == 0U) &&
             (object->speedRamp.elapsed == 0U)) ? TRUE : FALSE;
    }
    else if (object->phase == MOTOR_CONTROL_STARTUP_PHASE_OPEN_LOOP_RAMP)
    {
        phaseCountersValid =
            ((object->phaseCycle < config->openLoopRampCycles) &&
             (object->dRefRamp.elapsed == object->phaseCycle) &&
             (object->qRefRamp.elapsed == object->phaseCycle) &&
             (object->speedRamp.elapsed == object->phaseCycle)) ? TRUE :
                                                                  FALSE;
    }
    else if ((object->phase == MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY) ||
             (object->phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER) ||
             (object->phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM))
    {
        uint32 phaseLimit =
            (object->phase == MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY) ?
                config->qualificationCycles :
            ((object->phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER) ?
                config->handoverCycles : config->outputConfirmCycles);

        phaseCountersValid =
            ((object->phaseCycle < phaseLimit) &&
             (object->dRefRamp.elapsed == config->openLoopRampCycles) &&
             (object->qRefRamp.elapsed == config->openLoopRampCycles) &&
             (object->speedRamp.elapsed == config->openLoopRampCycles)) ?
            TRUE : FALSE;
    }
    else if (object->phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE)
    {
        phaseCountersValid =
            ((object->phaseCycle == 0U) &&
             (object->dRefRamp.elapsed == config->openLoopRampCycles) &&
             (object->qRefRamp.elapsed == config->openLoopRampCycles) &&
             (object->speedRamp.elapsed == config->openLoopRampCycles)) ?
            TRUE : FALSE;
    }
    else
    {
        phaseCountersValid =
            (object->phaseCycle <= terminalPhaseLimit) ? TRUE : FALSE;
    }

    return ((object->phase >= MOTOR_CONTROL_STARTUP_PHASE_IDLE) &&
            (object->phase <= MOTOR_CONTROL_STARTUP_PHASE_FAILED) &&
            (object->failure >= MOTOR_CONTROL_STARTUP_FAILURE_NONE) &&
            (object->failure <= MOTOR_CONTROL_STARTUP_FAILURE_START_TIMEOUT) &&
            (MotorControlStartup_IsQ15(
                 object->selectedSpeedAngleCounts) == TRUE) &&
            (MotorControlStartup_IsQ15(object->dRefQ15) == TRUE) &&
            (MotorControlStartup_IsQ15(object->qRefQ15) == TRUE) &&
            (MotorControlStartup_Abs(object->dRefQ15) <=
             (uint32)config->absoluteCurrentCapQ15) &&
            (MotorControlStartup_Abs(object->qRefQ15) <=
             (uint32)config->absoluteCurrentCapQ15) &&
            (object->totalCycle <= MOTOR_CONTROL_STARTUP_MAX_CYCLES) &&
            (object->qualificationConsecutiveCycles <=
             config->qualificationRequiredCycles) &&
            (MotorControlStartup_IsBoolean(object->observerQualified) == TRUE) &&
            (MotorControlStartup_IsBoolean(object->handoverComplete) == TRUE) &&
            (MotorControlStartup_IsBoolean(object->readyForRun) == TRUE) &&
            (MotorControlStartup_IsQ15(object->dRefRamp.current) == TRUE) &&
            (((dDifference >= 0) &&
              (object->dRefRamp.current >= config->alignDRefQ15) &&
              (object->dRefRamp.current <= config->finalDRefQ15)) ||
             ((dDifference < 0) &&
              (object->dRefRamp.current <= config->alignDRefQ15) &&
              (object->dRefRamp.current >= config->finalDRefQ15))) &&
            (object->dRefRamp.target == config->finalDRefQ15) &&
            (object->dRefRamp.duration == config->openLoopRampCycles) &&
            (object->dRefRamp.direction ==
                 ((dDifference > 0) ? 1 : ((dDifference < 0) ? -1 : 0))) &&
            (object->dRefRamp.remainder < object->dRefRamp.duration) &&
            (object->dRefRamp.error < object->dRefRamp.duration) &&
            (object->dRefRamp.elapsed <= object->dRefRamp.duration) &&
            ((((uint64)object->dRefRamp.baseIncrement *
               (uint64)object->dRefRamp.duration) +
              (uint64)object->dRefRamp.remainder) == dMagnitude) &&
            (MotorControlStartup_IsQ15(object->qRefRamp.current) == TRUE) &&
            (((qDifference >= 0) &&
              (object->qRefRamp.current >= config->alignQRefQ15) &&
              (object->qRefRamp.current <= config->finalQRefQ15)) ||
             ((qDifference < 0) &&
              (object->qRefRamp.current <= config->alignQRefQ15) &&
              (object->qRefRamp.current >= config->finalQRefQ15))) &&
            (object->qRefRamp.target == config->finalQRefQ15) &&
            (object->qRefRamp.duration == config->openLoopRampCycles) &&
            (object->qRefRamp.direction ==
                 ((qDifference > 0) ? 1 : ((qDifference < 0) ? -1 : 0))) &&
            (object->qRefRamp.remainder < object->qRefRamp.duration) &&
            (object->qRefRamp.error < object->qRefRamp.duration) &&
            (object->qRefRamp.elapsed <= object->qRefRamp.duration) &&
            ((((uint64)object->qRefRamp.baseIncrement *
               (uint64)object->qRefRamp.duration) +
              (uint64)object->qRefRamp.remainder) == qMagnitude) &&
            (MotorControlStartup_IsQ15(object->speedRamp.current) == TRUE) &&
            (((speedDifference > 0) &&
              (object->speedRamp.current >= 0) &&
              (object->speedRamp.current <=
               config->finalOpenLoopSpeedAngleCounts)) ||
             ((speedDifference < 0) &&
              (object->speedRamp.current <= 0) &&
              (object->speedRamp.current >=
               config->finalOpenLoopSpeedAngleCounts))) &&
            (object->speedRamp.target ==
             config->finalOpenLoopSpeedAngleCounts) &&
            (object->speedRamp.duration == config->openLoopRampCycles) &&
            (object->speedRamp.direction ==
                 ((speedDifference > 0) ? 1 : -1)) &&
            (object->speedRamp.remainder < object->speedRamp.duration) &&
            (object->speedRamp.error < object->speedRamp.duration) &&
            (object->speedRamp.elapsed <= object->speedRamp.duration) &&
            ((((uint64)object->speedRamp.baseIncrement *
               (uint64)object->speedRamp.duration) +
              (uint64)object->speedRamp.remainder) == speedMagnitude) &&
            (phaseCountersValid == TRUE)) ?
           TRUE : FALSE;
}

STATIC void MotorControlStartup_AdvanceOpenLoopAngle(
    MotorControlStartup_Type *object)
{
    object->selectedAngleQ16 =
        (uint16)(object->selectedAngleQ16 +
                 (uint16)object->selectedSpeedAngleCounts);
}

STATIC void MotorControlStartup_TrackObserver(
    MotorControlStartup_Type *object,
    const MotorControlStartup_ObserverSampleType *observer)
{
    sint64 speedDelta = (sint64)observer->speedAngleCounts -
                        (sint64)object->selectedSpeedAngleCounts;
    sint32 angleCorrection;
    uint16 predictedAngle;

    if (speedDelta >
        (sint64)object->config.handoverMaxSpeedSlewAngleCounts)
    {
        speedDelta = object->config.handoverMaxSpeedSlewAngleCounts;
    }
    else if (speedDelta <
             -(sint64)object->config.handoverMaxSpeedSlewAngleCounts)
    {
        speedDelta =
            -(sint64)object->config.handoverMaxSpeedSlewAngleCounts;
    }
    object->selectedSpeedAngleCounts += (sint32)speedDelta;
    predictedAngle =
        (uint16)(object->selectedAngleQ16 +
                 (uint16)object->selectedSpeedAngleCounts);
    angleCorrection = MotorControlStartup_CircularDelta(
        observer->angleQ16, predictedAngle);
    if (angleCorrection >
        (sint32)object->config.handoverMaxAngleCorrectionQ16)
    {
        angleCorrection =
            (sint32)object->config.handoverMaxAngleCorrectionQ16;
    }
    else if (angleCorrection <
             -(sint32)object->config.handoverMaxAngleCorrectionQ16)
    {
        angleCorrection =
            -(sint32)object->config.handoverMaxAngleCorrectionQ16;
    }
    object->selectedAngleQ16 =
        (uint16)(predictedAngle + (uint16)angleCorrection);
}

STATIC void MotorControlStartup_IncrementCycle(
    MotorControlStartup_Type *object)
{
    object->phaseCycle++;
    object->totalCycle++;
}

Std_ReturnType MotorControlStartup_Init(
    MotorControlStartup_Type *Object,
    const MotorControlStartup_ConfigType *Config)
{
    if (Object == NULL_PTR)
    {
        return E_NOT_OK;
    }

    MotorControlStartup_ClearObject(Object);
    if (MotorControlStartup_ConfigValid(Config) != TRUE)
    {
        return E_NOT_OK;
    }

    Object->config = *Config;
    MotorControlStartup_PrepareRamp(
        &Object->dRefRamp,
        Config->alignDRefQ15,
        Config->finalDRefQ15,
        Config->openLoopRampCycles);
    MotorControlStartup_PrepareRamp(
        &Object->qRefRamp,
        Config->alignQRefQ15,
        Config->finalQRefQ15,
        Config->openLoopRampCycles);
    MotorControlStartup_PrepareRamp(
        &Object->speedRamp,
        0,
        Config->finalOpenLoopSpeedAngleCounts,
        Config->openLoopRampCycles);
    Object->phase = MOTOR_CONTROL_STARTUP_PHASE_IDLE;
    Object->initialized = TRUE;
    return E_OK;
}

Std_ReturnType MotorControlStartup_Begin(
    MotorControlStartup_Type *Object,
    uint32 AlreadyConsumedStartingCycles)
{
    uint64 total;

    if ((Object == NULL_PTR) ||
        (MotorControlStartup_RuntimeValid(Object) != TRUE) ||
        (Object->phase != MOTOR_CONTROL_STARTUP_PHASE_IDLE) ||
        (AlreadyConsumedStartingCycles == 0U) ||
        (AlreadyConsumedStartingCycles >=
         MOTOR_CONTROL_STARTUP_MAX_CYCLES))
    {
        return E_NOT_OK;
    }

    total = (uint64)AlreadyConsumedStartingCycles +
            MotorControlStartup_CoreDuration(&Object->config);
    if (total > (uint64)MOTOR_CONTROL_STARTUP_MAX_CYCLES)
    {
        return E_NOT_OK;
    }

    MotorControlStartup_ResetRamp(&Object->dRefRamp);
    MotorControlStartup_ResetRamp(&Object->qRefRamp);
    MotorControlStartup_ResetRamp(&Object->speedRamp);
    Object->selectedAngleQ16 = Object->config.startAngleQ16;
    Object->selectedSpeedAngleCounts = 0;
    Object->dRefQ15 = Object->config.alignDRefQ15;
    Object->qRefQ15 = Object->config.alignQRefQ15;
    Object->phaseCycle = 0U;
    Object->totalCycle = AlreadyConsumedStartingCycles;
    Object->qualificationConsecutiveCycles = 0U;
    Object->phase = MOTOR_CONTROL_STARTUP_PHASE_ALIGN;
    Object->failure = MOTOR_CONTROL_STARTUP_FAILURE_NONE;
    Object->observerQualified = FALSE;
    Object->handoverComplete = FALSE;
    Object->readyForRun = FALSE;
    return E_OK;
}

Std_ReturnType MotorControlStartup_ValidateCycle(
    const MotorControlStartup_Type *Object,
    const MotorControlStartup_StepInputType *Input,
    sint32 *CurrentQRefQ15)
{
    boolean observerBaseValid;

    if (CurrentQRefQ15 != NULL_PTR)
    {
        *CurrentQRefQ15 = 0;
    }
    if ((Object == NULL_PTR) || (Input == NULL_PTR) ||
        (CurrentQRefQ15 == NULL_PTR) ||
        (Object->initialized != TRUE) ||
        (Object->phase < MOTOR_CONTROL_STARTUP_PHASE_ALIGN) ||
        (Object->phase > MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) ||
        (MotorControlStartup_RuntimeValid(Object) != TRUE) ||
        (MotorControlStartup_InputRangesValid(Input) != TRUE) ||
        (Object->totalCycle >= MOTOR_CONTROL_STARTUP_MAX_CYCLES) ||
        (Input->generationValid != TRUE) ||
        (Input->safetyValid != TRUE))
    {
        return E_NOT_OK;
    }

    observerBaseValid = MotorControlStartup_ObserverBaseValid(
        Object, &Input->observer);
    if (((Object->phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER) ||
         (Object->phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM)) &&
        (observerBaseValid != TRUE))
    {
        return E_NOT_OK;
    }
    if ((Object->phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
        (MotorControlStartup_FinalObserverErrorValid(
             Object, &Input->observer) != TRUE))
    {
        return E_NOT_OK;
    }

    *CurrentQRefQ15 = Object->qRefQ15;
    return E_OK;
}

Std_ReturnType MotorControlStartup_Step(
    MotorControlStartup_Type *Object,
    const MotorControlStartup_StepInputType *Input,
    MotorControlStartup_ResultType *Result)
{
    MotorControlStartup_PhaseType executedPhase;
    boolean observerBaseValid;

    MotorControlStartup_ClearResult(Result);
    if ((Object == NULL_PTR) || (Input == NULL_PTR) || (Result == NULL_PTR))
    {
        return E_NOT_OK;
    }
    if ((Object->initialized != TRUE) ||
        (Object->phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE))
    {
        return E_NOT_OK;
    }
    if (MotorControlStartup_RuntimeValid(Object) != TRUE)
    {
        return MotorControlStartup_LatchFailure(
            Object, MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC, Result);
    }

    if (Object->phase == MOTOR_CONTROL_STARTUP_PHASE_ABORTED)
    {
        MotorControlStartup_FillResult(
            Object, MOTOR_CONTROL_STARTUP_PHASE_ABORTED, TRUE, Result);
        return E_OK;
    }
    if (Object->phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED)
    {
        MotorControlStartup_FillResult(
            Object, MOTOR_CONTROL_STARTUP_PHASE_FAILED, FALSE, Result);
        return E_NOT_OK;
    }
    if (Object->phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE)
    {
        MotorControlStartup_FillResult(
            Object, MOTOR_CONTROL_STARTUP_PHASE_COMPLETE, FALSE, Result);
        return E_OK;
    }
    if (MotorControlStartup_InputRangesValid(Input) != TRUE)
    {
        return MotorControlStartup_LatchFailure(
            Object, MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC, Result);
    }
    if (Object->totalCycle >= MOTOR_CONTROL_STARTUP_MAX_CYCLES)
    {
        return MotorControlStartup_LatchFailure(
            Object, MOTOR_CONTROL_STARTUP_FAILURE_START_TIMEOUT, Result);
    }
    if ((Input->generationValid != TRUE) || (Input->safetyValid != TRUE))
    {
        return MotorControlStartup_LatchFailure(
            Object, MOTOR_CONTROL_STARTUP_FAILURE_GUARD, Result);
    }

    observerBaseValid = MotorControlStartup_ObserverBaseValid(
        Object, &Input->observer);
    if (((Object->phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER) ||
         (Object->phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM)) &&
        (observerBaseValid != TRUE))
    {
        return MotorControlStartup_LatchFailure(
            Object, MOTOR_CONTROL_STARTUP_FAILURE_OBSERVER_LOSS, Result);
    }
    if ((Object->phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
        (MotorControlStartup_FinalObserverErrorValid(
             Object, &Input->observer) != TRUE))
    {
        return MotorControlStartup_LatchFailure(
            Object, MOTOR_CONTROL_STARTUP_FAILURE_OBSERVER_LOSS, Result);
    }
    if ((Input->stopRequested == TRUE) ||
        (Input->commandContinueValid != TRUE))
    {
        executedPhase = Object->phase;
        Object->phase = MOTOR_CONTROL_STARTUP_PHASE_ABORTED;
        Object->readyForRun = FALSE;
        MotorControlStartup_FillResult(Object, executedPhase, TRUE, Result);
        return E_OK;
    }

    executedPhase = Object->phase;
    if (executedPhase == MOTOR_CONTROL_STARTUP_PHASE_ALIGN)
    {
        MotorControlStartup_IncrementCycle(Object);
        if (Object->phaseCycle == Object->config.alignCycles)
        {
            Object->phase = MOTOR_CONTROL_STARTUP_PHASE_OPEN_LOOP_RAMP;
            Object->phaseCycle = 0U;
        }
    }
    else if (executedPhase == MOTOR_CONTROL_STARTUP_PHASE_OPEN_LOOP_RAMP)
    {
        Object->dRefQ15 = MotorControlStartup_AdvanceRamp(&Object->dRefRamp);
        Object->qRefQ15 = MotorControlStartup_AdvanceRamp(&Object->qRefRamp);
        Object->selectedSpeedAngleCounts =
            MotorControlStartup_AdvanceRamp(&Object->speedRamp);
        MotorControlStartup_AdvanceOpenLoopAngle(Object);
        MotorControlStartup_IncrementCycle(Object);
        if (Object->phaseCycle == Object->config.openLoopRampCycles)
        {
            Object->phase = MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY;
            Object->phaseCycle = 0U;
        }
    }
    else if (executedPhase == MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY)
    {
        MotorControlStartup_AdvanceOpenLoopAngle(Object);
        if (MotorControlStartup_QualificationValid(
                Object, &Input->observer) == TRUE)
        {
            Object->qualificationConsecutiveCycles++;
        }
        else
        {
            Object->qualificationConsecutiveCycles = 0U;
        }
        MotorControlStartup_IncrementCycle(Object);
        if (Object->qualificationConsecutiveCycles >=
            Object->config.qualificationRequiredCycles)
        {
            Object->observerQualified = TRUE;
            Object->phase = MOTOR_CONTROL_STARTUP_PHASE_HANDOVER;
            Object->phaseCycle = 0U;
        }
        else if (Object->phaseCycle == Object->config.qualificationCycles)
        {
            return MotorControlStartup_LatchFailure(
                Object,
                MOTOR_CONTROL_STARTUP_FAILURE_QUALIFICATION_TIMEOUT,
                Result);
        }
    }
    else if (executedPhase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER)
    {
        MotorControlStartup_TrackObserver(Object, &Input->observer);
        MotorControlStartup_IncrementCycle(Object);
        if (Object->phaseCycle == Object->config.handoverCycles)
        {
            if (MotorControlStartup_FinalObserverErrorValid(
                    Object, &Input->observer) != TRUE)
            {
                return MotorControlStartup_LatchFailure(
                    Object,
                    MOTOR_CONTROL_STARTUP_FAILURE_HANDOVER_TIMEOUT,
                    Result);
            }
            Object->handoverComplete = TRUE;
            Object->phase = MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM;
            Object->phaseCycle = 0U;
        }
    }
    else
    {
        MotorControlStartup_TrackObserver(Object, &Input->observer);
        if (MotorControlStartup_FinalObserverErrorValid(
                Object, &Input->observer) != TRUE)
        {
            return MotorControlStartup_LatchFailure(
                Object,
                MOTOR_CONTROL_STARTUP_FAILURE_OBSERVER_LOSS,
                Result);
        }
        MotorControlStartup_IncrementCycle(Object);
        if (Object->phaseCycle == Object->config.outputConfirmCycles)
        {
            Object->phase = MOTOR_CONTROL_STARTUP_PHASE_COMPLETE;
            Object->phaseCycle = 0U;
            Object->readyForRun = TRUE;
        }
    }

    MotorControlStartup_FillResult(Object, executedPhase, FALSE, Result);
    return E_OK;
}

Std_ReturnType MotorControlStartup_RearmFromReady(
    MotorControlStartup_Type *Object)
{
    if ((Object == NULL_PTR) ||
        (MotorControlStartup_RuntimeValid(Object) != TRUE) ||
        ((Object->phase != MOTOR_CONTROL_STARTUP_PHASE_ABORTED) &&
         (Object->phase != MOTOR_CONTROL_STARTUP_PHASE_COMPLETE)))
    {
        return E_NOT_OK;
    }

    MotorControlStartup_ResetRamp(&Object->dRefRamp);
    MotorControlStartup_ResetRamp(&Object->qRefRamp);
    MotorControlStartup_ResetRamp(&Object->speedRamp);
    Object->selectedAngleQ16 = 0U;
    Object->selectedSpeedAngleCounts = 0;
    Object->dRefQ15 = 0;
    Object->qRefQ15 = 0;
    Object->phaseCycle = 0U;
    Object->totalCycle = 0U;
    Object->qualificationConsecutiveCycles = 0U;
    Object->phase = MOTOR_CONTROL_STARTUP_PHASE_IDLE;
    Object->failure = MOTOR_CONTROL_STARTUP_FAILURE_NONE;
    Object->observerQualified = FALSE;
    Object->handoverComplete = FALSE;
    Object->readyForRun = FALSE;
    return E_OK;
}
