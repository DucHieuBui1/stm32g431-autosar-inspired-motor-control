#include "MotorControlFastLoop.h"

#define MOTOR_CONTROL_FAST_LOOP_Q15_MIN              ((sint32)-32768)
#define MOTOR_CONTROL_FAST_LOOP_Q15_MAX              ((sint32)32767)
#define MOTOR_CONTROL_FAST_LOOP_ANGLE_QUARTER_TURN   ((uint16)16384U)
#define MOTOR_CONTROL_FAST_LOOP_ANGLE_HALF_TURN      ((uint16)32768U)
#define MOTOR_CONTROL_FAST_LOOP_ANGLE_THREE_QUARTER  ((uint16)49152U)
#define MOTOR_CONTROL_FAST_LOOP_ANGLE_FULL_TURN      ((sint32)65536)
#define MOTOR_CONTROL_FAST_LOOP_CORDIC_GAIN_Q30      ((sint64)652012976)
#define MOTOR_CONTROL_FAST_LOOP_CORDIC_PASSES        ((uint32)15U)
#define MOTOR_CONTROL_FAST_LOOP_Q30_TO_Q15_HALF      ((sint64)16384)
#define MOTOR_CONTROL_FAST_LOOP_Q30_TO_Q15_SHIFT     ((uint32)15U)

#if defined(MOTOR_CONTROL_FAST_LOOP_SINCOS_CALL) && \
    !defined(MOTOR_CONTROL_FAST_LOOP_HOST_TEST)
#error "FastLoop sine-cosine substitution is restricted to host tests"
#endif
#ifndef MOTOR_CONTROL_FAST_LOOP_SINCOS_CALL
#define MOTOR_CONTROL_FAST_LOOP_SINCOS_CALL MotorControlFastLoop_SinCosQ15
#endif

STATIC const sint32 MotorControlFastLoop_AtanQ16[
    MOTOR_CONTROL_FAST_LOOP_CORDIC_PASSES] =
{
    8192, 4836, 2555, 1297, 651,
    326, 163, 81, 41, 20, 10, 5, 3, 1, 1
};

STATIC boolean MotorControlFastLoop_IsCanonical(boolean value)
{
    return ((value == FALSE) || (value == TRUE)) ? TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_LifecycleKnown(
    MotorControlFastLoop_LifecycleType lifecycle)
{
    switch (lifecycle)
    {
        case MOTOR_CONTROL_FAST_LOOP_UNINITIALIZED:
        case MOTOR_CONTROL_FAST_LOOP_IDLE:
        case MOTOR_CONTROL_FAST_LOOP_ACTIVE:
        case MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED:
        case MOTOR_CONTROL_FAST_LOOP_FAILED:
            return TRUE;
        default:
            return FALSE;
    }
}

STATIC boolean MotorControlFastLoop_IsQ15(sint32 value)
{
    return ((value >= MOTOR_CONTROL_FAST_LOOP_Q15_MIN) &&
            (value <= MOTOR_CONTROL_FAST_LOOP_Q15_MAX)) ? TRUE : FALSE;
}

STATIC uint32 MotorControlFastLoop_AbsS32(sint32 value)
{
    return (value >= 0) ? (uint32)value : (uint32)(-(sint64)value);
}

STATIC boolean MotorControlCdd_ApplyCommissioningIqCap(
    const MotorControlFastLoop_Type *object,
    sint32 qReferenceQ15)
{
    sint32 acceptedRequestQ15 = object->config.startup.finalQRefQ15;
    uint32 referenceMagnitude = MotorControlFastLoop_AbsS32(qReferenceQ15);
    uint32 requestMagnitude = MotorControlFastLoop_AbsS32(acceptedRequestQ15);
    uint32 capMagnitude =
        (uint32)object->config.startup.absoluteCurrentCapQ15;

    if ((acceptedRequestQ15 == 0) ||
        (referenceMagnitude > requestMagnitude) ||
        (referenceMagnitude > capMagnitude) ||
        ((qReferenceQ15 > 0) && (acceptedRequestQ15 < 0)) ||
        ((qReferenceQ15 < 0) && (acceptedRequestQ15 > 0)))
    {
        return FALSE;
    }
    return TRUE;
}

STATIC sint64 MotorControlFastLoop_ShiftTowardZero(
    sint64 value,
    uint32 shift)
{
    if (value >= 0)
    {
        return value >> shift;
    }

    return -((-value) >> shift);
}

STATIC sint32 MotorControlFastLoop_RoundQ30ToQ15(sint64 value)
{
    if (value >= 0)
    {
        return (sint32)((value + MOTOR_CONTROL_FAST_LOOP_Q30_TO_Q15_HALF) >>
                        MOTOR_CONTROL_FAST_LOOP_Q30_TO_Q15_SHIFT);
    }

    return (sint32)(-(((-value) +
                       MOTOR_CONTROL_FAST_LOOP_Q30_TO_Q15_HALF) >>
                      MOTOR_CONTROL_FAST_LOOP_Q30_TO_Q15_SHIFT));
}

STATIC void MotorControlFastLoop_ClearResult(
    MotorControlFastLoop_ResultType *result)
{
    if (result != NULL_PTR)
    {
        *result = (MotorControlFastLoop_ResultType){0};
    }
}

STATIC void MotorControlFastLoop_ClearObject(
    MotorControlFastLoop_Type *object)
{
    *object = (MotorControlFastLoop_Type){0};
}

STATIC boolean MotorControlFastLoop_PiConfigEqual(
    const MotorControlAlgorithm_PiConfigType *first,
    const MotorControlAlgorithm_PiConfigType *second)
{
    return ((first->kpQ15 == second->kpQ15) &&
            (first->kiQ15 == second->kiQ15) &&
            (first->lowerQ15 == second->lowerQ15) &&
            (first->upperQ15 == second->upperQ15)) ? TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_PiObjectValid(
    const MotorControlAlgorithm_PiType *object,
    const MotorControlAlgorithm_PiConfigType *config)
{
    sint64 lowerQ30;
    sint64 upperQ30;

    if ((object->initialized != TRUE) ||
        (MotorControlFastLoop_PiConfigEqual(&object->config, config) != TRUE))
    {
        return FALSE;
    }
    lowerQ30 = (sint64)config->lowerQ15 * (sint64)32768;
    upperQ30 = (sint64)config->upperQ15 * (sint64)32768;
    return ((object->integratorQ30 >= lowerQ30) &&
            (object->integratorQ30 <= upperQ30)) ? TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_StartupConfigEqual(
    const MotorControlStartup_ConfigType *first,
    const MotorControlStartup_ConfigType *second)
{
    return ((first->alignCycles == second->alignCycles) &&
            (first->openLoopRampCycles == second->openLoopRampCycles) &&
            (first->qualificationCycles == second->qualificationCycles) &&
            (first->handoverCycles == second->handoverCycles) &&
            (first->outputConfirmCycles == second->outputConfirmCycles) &&
            (first->startAngleQ16 == second->startAngleQ16) &&
            (first->alignDRefQ15 == second->alignDRefQ15) &&
            (first->alignQRefQ15 == second->alignQRefQ15) &&
            (first->finalDRefQ15 == second->finalDRefQ15) &&
            (first->finalQRefQ15 == second->finalQRefQ15) &&
            (first->absoluteCurrentCapQ15 == second->absoluteCurrentCapQ15) &&
            (first->finalOpenLoopSpeedAngleCounts ==
             second->finalOpenLoopSpeedAngleCounts) &&
            (first->qualificationRequiredCycles ==
             second->qualificationRequiredCycles) &&
            (first->minimumObservableSpeedAngleCounts ==
             second->minimumObservableSpeedAngleCounts) &&
            (first->qualificationMaxAngleErrorQ16 ==
             second->qualificationMaxAngleErrorQ16) &&
            (first->qualificationMaxSpeedErrorAngleCounts ==
             second->qualificationMaxSpeedErrorAngleCounts) &&
            (first->handoverMaxAngleCorrectionQ16 ==
             second->handoverMaxAngleCorrectionQ16) &&
            (first->handoverMaxSpeedSlewAngleCounts ==
             second->handoverMaxSpeedSlewAngleCounts) &&
            (first->finalMaxAngleErrorQ16 ==
             second->finalMaxAngleErrorQ16) &&
            (first->finalMaxSpeedErrorAngleCounts ==
             second->finalMaxSpeedErrorAngleCounts)) ? TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_ObserverConfigEqual(
    const MotorControlObserver_ConfigType *first,
    const MotorControlObserver_ConfigType *second)
{
    return ((first->currentRetentionQ15 == second->currentRetentionQ15) &&
            (first->voltageInjectionQ15 == second->voltageInjectionQ15) &&
            (first->bemfSubtractionQ15 == second->bemfSubtractionQ15) &&
            (first->currentErrorToCurrentQ15 ==
             second->currentErrorToCurrentQ15) &&
            (first->currentErrorToBemfQ15 ==
             second->currentErrorToBemfQ15) &&
            (first->phaseProportionalQ15 == second->phaseProportionalQ15) &&
            (first->phaseIntegralQ15 == second->phaseIntegralQ15) &&
            (first->maxAbsCurrentQ15 == second->maxAbsCurrentQ15) &&
            (first->maxAbsBemfQ15 == second->maxAbsBemfQ15) &&
            (first->maxAbsSpeedAngleCounts ==
             second->maxAbsSpeedAngleCounts) &&
            (first->maxSpeedDeltaAngleCounts ==
             second->maxSpeedDeltaAngleCounts) &&
            (first->minBemfMagnitudeSqQ30 ==
             second->minBemfMagnitudeSqQ30) &&
            (first->maxBemfMagnitudeSqQ30 ==
             second->maxBemfMagnitudeSqQ30) &&
            (first->maxCurrentResidualMagnitudeSqQ30 ==
             second->maxCurrentResidualMagnitudeSqQ30) &&
            (first->speedWindowLength == second->speedWindowLength) &&
            (first->maxSpeedVarianceNumerator ==
             second->maxSpeedVarianceNumerator) &&
            (first->reliabilityWarmupSamples ==
             second->reliabilityWarmupSamples) &&
            (first->maxConsecutiveUnreliableSamples ==
             second->maxConsecutiveUnreliableSamples)) ? TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_ConfigMatchesObjects(
    const MotorControlFastLoop_Type *object)
{
    return ((object->config.phaseSumToleranceQ15 <=
             MOTOR_CONTROL_FAST_LOOP_PHASE_SUM_MAX_Q15) &&
            (object->config.trigNormToleranceQ30 >=
             MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30) &&
            (object->config.trigNormToleranceQ30 <=
             MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MAX_Q30) &&
            (MotorControlFastLoop_ObserverConfigEqual(
                 &object->config.observer,
                 &object->observer.config) == TRUE) &&
            (MotorControlFastLoop_StartupConfigEqual(
                 &object->config.startup,
                 &object->startup.config) == TRUE) &&
            (MotorControlFastLoop_PiObjectValid(
                 &object->dPi, &object->config.dPi) == TRUE) &&
            (MotorControlFastLoop_PiObjectValid(
                 &object->qPi, &object->config.qPi) == TRUE)) ? TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_RuntimeValid(
    const MotorControlFastLoop_Type *object)
{
    if ((object == NULL_PTR) ||
        (object->failure != MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE) ||
        (MotorControlFastLoop_IsCanonical(
             object->generationEpochValid) != TRUE) ||
        (MotorControlFastLoop_ConfigMatchesObjects(object) != TRUE))
    {
        return FALSE;
    }

    if (object->lifecycle == MOTOR_CONTROL_FAST_LOOP_IDLE)
    {
        return ((object->observer.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE) &&
                (object->observer.failure ==
                 MOTOR_CONTROL_OBSERVER_FAILURE_NONE) &&
                (object->startup.phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE) &&
                (object->startup.failure ==
                 MOTOR_CONTROL_STARTUP_FAILURE_NONE) &&
                (object->generationEpochValid == FALSE) &&
                (object->expectedDirection == 0)) ? TRUE : FALSE;
    }

    if ((object->lifecycle != MOTOR_CONTROL_FAST_LOOP_ACTIVE) &&
        (object->lifecycle != MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED))
    {
        return FALSE;
    }
    if ((object->observer.lifecycle != MOTOR_CONTROL_OBSERVER_ACTIVE) ||
        (object->observer.failure != MOTOR_CONTROL_OBSERVER_FAILURE_NONE) ||
        ((object->expectedDirection != 1) &&
         (object->expectedDirection != -1)) ||
        (object->observer.expectedDirection != object->expectedDirection) ||
        ((object->mode != MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP) &&
         (object->mode != MOTOR_CONTROL_FAST_LOOP_MODE_RUN)))
    {
        return FALSE;
    }

    if (object->lifecycle == MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED)
    {
        return ((((object->mode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP) &&
                  (object->startup.phase ==
                   MOTOR_CONTROL_STARTUP_PHASE_ABORTED)) ||
                 ((object->mode == MOTOR_CONTROL_FAST_LOOP_MODE_RUN) &&
                  (object->startup.phase ==
                   MOTOR_CONTROL_STARTUP_PHASE_COMPLETE))) &&
                (object->startup.failure ==
                 MOTOR_CONTROL_STARTUP_FAILURE_NONE)) ? TRUE : FALSE;
    }

    if (object->mode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP)
    {
        return ((object->startup.phase >=
                 MOTOR_CONTROL_STARTUP_PHASE_ALIGN) &&
                (object->startup.phase <=
                 MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
                (object->startup.failure ==
                 MOTOR_CONTROL_STARTUP_FAILURE_NONE)) ? TRUE : FALSE;
    }

    return ((object->startup.phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE) &&
            (object->startup.failure == MOTOR_CONTROL_STARTUP_FAILURE_NONE) &&
            (object->startup.readyForRun == TRUE) &&
            (object->retainedStartupResult.phase ==
             MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
            (object->retainedStartupResult.readyForRun == TRUE) &&
            (object->retainedStartupResult.failure ==
             MOTOR_CONTROL_STARTUP_FAILURE_NONE) &&
            (object->runDRefQ15 == object->config.startup.finalDRefQ15) &&
            (object->runQRefQ15 == object->config.startup.finalQRefQ15)) ?
           TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_InputRepresentationValid(
    const MotorControlFastLoop_StepInputType *input)
{
    return ((input != NULL_PTR) &&
            (MotorControlFastLoop_IsQ15(input->phaseAQ15) == TRUE) &&
            (MotorControlFastLoop_IsQ15(input->phaseBQ15) == TRUE) &&
            (MotorControlFastLoop_IsQ15(input->phaseCQ15) == TRUE) &&
            (MotorControlFastLoop_IsQ15(
                 input->appliedVoltage.alphaQ15) == TRUE) &&
            (MotorControlFastLoop_IsQ15(
                 input->appliedVoltage.betaQ15) == TRUE) &&
            (MotorControlFastLoop_IsCanonical(
                 input->currentSampleValid) == TRUE) &&
            (MotorControlFastLoop_IsCanonical(
                 input->appliedVoltageValid) == TRUE) &&
            (MotorControlFastLoop_IsCanonical(
                 input->generationOwnerValid) == TRUE) &&
            (MotorControlFastLoop_IsCanonical(input->safetyValid) == TRUE) &&
            (MotorControlFastLoop_IsCanonical(
                 input->commandContinueValid) == TRUE) &&
            (MotorControlFastLoop_IsCanonical(
                 input->stopRequested) == TRUE)) ? TRUE : FALSE;
}

STATIC boolean MotorControlFastLoop_ObserverReliable(
    const MotorControlFastLoop_Type *object,
    const MotorControlObserver_ResultType *observer)
{
    return ((observer->numericValid == TRUE) &&
            (observer->angleValid == TRUE) &&
            (observer->speedValid == TRUE) &&
            (observer->varianceValid == TRUE) &&
            (observer->bemfValid == TRUE) &&
            (observer->currentValid == TRUE) &&
            (observer->directionValid == TRUE) &&
            (MotorControlFastLoop_AbsS32(observer->speedAngleCounts) >=
             object->config.startup.minimumObservableSpeedAngleCounts) &&
            (((object->expectedDirection == 1) &&
              (observer->speedAngleCounts > 0)) ||
             ((object->expectedDirection == -1) &&
              (observer->speedAngleCounts < 0)))) ? TRUE : FALSE;
}

STATIC Std_ReturnType MotorControlFastLoop_LatchFailure(
    MotorControlFastLoop_Type *object,
    MotorControlFastLoop_FailureType failure,
    MotorControlFastLoop_ResultType *result)
{
    object->failure = failure;
    object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
    MotorControlFastLoop_ClearResult(result);
    result->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
    result->failure = object->failure;
    return E_NOT_OK;
}

Std_ReturnType MotorControlFastLoop_SinCosQ15(
    uint16 AngleQ16,
    sint32 *SinQ15,
    sint32 *CosQ15)
{
    sint64 x;
    sint64 y;
    sint64 oldX;
    sint64 oldY;
    sint64 shiftedX;
    sint64 shiftedY;
    sint32 z;
    sint32 sign = 1;
    uint32 pass;

    if (SinQ15 != NULL_PTR)
    {
        *SinQ15 = 0;
    }
    if (CosQ15 != NULL_PTR)
    {
        *CosQ15 = 0;
    }
    if ((SinQ15 == NULL_PTR) || (CosQ15 == NULL_PTR))
    {
        return E_NOT_OK;
    }

    if (AngleQ16 == 0U)
    {
        *CosQ15 = MOTOR_CONTROL_FAST_LOOP_Q15_MAX;
        return E_OK;
    }
    if (AngleQ16 == MOTOR_CONTROL_FAST_LOOP_ANGLE_QUARTER_TURN)
    {
        *SinQ15 = MOTOR_CONTROL_FAST_LOOP_Q15_MAX;
        return E_OK;
    }
    if (AngleQ16 == MOTOR_CONTROL_FAST_LOOP_ANGLE_HALF_TURN)
    {
        *CosQ15 = -MOTOR_CONTROL_FAST_LOOP_Q15_MAX;
        return E_OK;
    }
    if (AngleQ16 == MOTOR_CONTROL_FAST_LOOP_ANGLE_THREE_QUARTER)
    {
        *SinQ15 = -MOTOR_CONTROL_FAST_LOOP_Q15_MAX;
        return E_OK;
    }

    z = (sint32)AngleQ16;
    if (z >= (sint32)MOTOR_CONTROL_FAST_LOOP_ANGLE_HALF_TURN)
    {
        z -= MOTOR_CONTROL_FAST_LOOP_ANGLE_FULL_TURN;
    }
    if (z > (sint32)MOTOR_CONTROL_FAST_LOOP_ANGLE_QUARTER_TURN)
    {
        z -= (sint32)MOTOR_CONTROL_FAST_LOOP_ANGLE_HALF_TURN;
        sign = -1;
    }
    else if (z < -(sint32)MOTOR_CONTROL_FAST_LOOP_ANGLE_QUARTER_TURN)
    {
        z += (sint32)MOTOR_CONTROL_FAST_LOOP_ANGLE_HALF_TURN;
        sign = -1;
    }

    x = MOTOR_CONTROL_FAST_LOOP_CORDIC_GAIN_Q30;
    y = 0;
    for (pass = 0U;
         pass < MOTOR_CONTROL_FAST_LOOP_CORDIC_PASSES;
         pass++)
    {
        oldX = x;
        oldY = y;
        shiftedX = MotorControlFastLoop_ShiftTowardZero(oldX, pass);
        shiftedY = MotorControlFastLoop_ShiftTowardZero(oldY, pass);
        if (z >= 0)
        {
            x = oldX - shiftedY;
            y = oldY + shiftedX;
            z -= MotorControlFastLoop_AtanQ16[pass];
        }
        else
        {
            x = oldX + shiftedY;
            y = oldY - shiftedX;
            z += MotorControlFastLoop_AtanQ16[pass];
        }
    }

    x *= (sint64)sign;
    y *= (sint64)sign;
    *SinQ15 = MotorControlFastLoop_RoundQ30ToQ15(y);
    *CosQ15 = MotorControlFastLoop_RoundQ30ToQ15(x);
    if ((MotorControlFastLoop_IsQ15(*SinQ15) != TRUE) ||
        (MotorControlFastLoop_IsQ15(*CosQ15) != TRUE))
    {
        *SinQ15 = 0;
        *CosQ15 = 0;
        return E_NOT_OK;
    }
    return E_OK;
}

Std_ReturnType MotorControlFastLoop_Init(
    MotorControlFastLoop_Type *Object,
    const MotorControlFastLoop_ConfigType *Config)
{
    MotorControlFastLoop_Type candidate = {0};

    if (Object == NULL_PTR)
    {
        return E_NOT_OK;
    }
    MotorControlFastLoop_ClearObject(Object);
    if ((Config == NULL_PTR) ||
        (Config->phaseSumToleranceQ15 >
         MOTOR_CONTROL_FAST_LOOP_PHASE_SUM_MAX_Q15) ||
        (Config->trigNormToleranceQ30 <
         MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30) ||
        (Config->trigNormToleranceQ30 >
         MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MAX_Q30) ||
        (Config->startup.minimumObservableSpeedAngleCounts >
         Config->observer.maxAbsSpeedAngleCounts) ||
        (MotorControlFastLoop_AbsS32(
             Config->startup.finalOpenLoopSpeedAngleCounts) >
         Config->observer.maxAbsSpeedAngleCounts) ||
        (MotorControlObserver_Init(
             &candidate.observer, &Config->observer) != E_OK) ||
        (MotorControlStartup_Init(
             &candidate.startup, &Config->startup) != E_OK) ||
        (MotorControlAlgorithm_PiInit(
             &candidate.dPi, &Config->dPi, 0) != E_OK) ||
        (MotorControlAlgorithm_PiInit(
             &candidate.qPi, &Config->qPi, 0) != E_OK))
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_CONFIGURATION;
        return E_NOT_OK;
    }

    candidate.config = *Config;
    candidate.lifecycle = MOTOR_CONTROL_FAST_LOOP_IDLE;
    candidate.mode = MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP;
    candidate.failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE;
    candidate.generationEpochValid = FALSE;
    *Object = candidate;
    return E_OK;
}

Std_ReturnType MotorControlFastLoop_Begin(
    MotorControlFastLoop_Type *Object,
    sint32 ExpectedDirection,
    uint32 AlreadyConsumedStartingCycles)
{
    MotorControlFastLoop_Type candidate;
    sint32 configuredDirection;
    uint64 startupCoreCycles;

    if (Object == NULL_PTR)
    {
        return E_NOT_OK;
    }
    if (MotorControlFastLoop_LifecycleKnown(Object->lifecycle) != TRUE)
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        return E_NOT_OK;
    }
    if (Object->lifecycle != MOTOR_CONTROL_FAST_LOOP_IDLE)
    {
        return E_NOT_OK;
    }
    if (MotorControlFastLoop_RuntimeValid(Object) != TRUE)
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        return E_NOT_OK;
    }
    configuredDirection =
        (Object->config.startup.finalOpenLoopSpeedAngleCounts > 0) ? 1 : -1;
    startupCoreCycles =
        (uint64)Object->config.startup.alignCycles +
        (uint64)Object->config.startup.openLoopRampCycles +
        (uint64)Object->config.startup.qualificationCycles +
        (uint64)Object->config.startup.handoverCycles +
        (uint64)Object->config.startup.outputConfirmCycles;
    if (((ExpectedDirection != 1) && (ExpectedDirection != -1)) ||
        (ExpectedDirection != configuredDirection) ||
        (AlreadyConsumedStartingCycles == 0U) ||
        (AlreadyConsumedStartingCycles >=
         MOTOR_CONTROL_STARTUP_MAX_CYCLES) ||
        (((uint64)AlreadyConsumedStartingCycles + startupCoreCycles) >
         (uint64)MOTOR_CONTROL_STARTUP_MAX_CYCLES))
    {
        return E_NOT_OK;
    }

    candidate = *Object;
    if ((MotorControlObserver_Begin(
             &candidate.observer,
             candidate.config.startup.startAngleQ16,
             ExpectedDirection) != E_OK) ||
        (MotorControlStartup_Begin(
             &candidate.startup,
             AlreadyConsumedStartingCycles) != E_OK) ||
        (MotorControlAlgorithm_PiReset(&candidate.dPi, 0) != E_OK) ||
        (MotorControlAlgorithm_PiReset(&candidate.qPi, 0) != E_OK))
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        return E_NOT_OK;
    }

    candidate.retainedStartupResult =
        (MotorControlStartup_ResultType){0};
    candidate.runDRefQ15 = 0;
    candidate.runQRefQ15 = 0;
    candidate.lastGenerationId = 0U;
    candidate.expectedDirection = ExpectedDirection;
    candidate.lifecycle = MOTOR_CONTROL_FAST_LOOP_ACTIVE;
    candidate.mode = MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP;
    candidate.failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE;
    candidate.generationEpochValid = FALSE;
    *Object = candidate;
    return E_OK;
}

Std_ReturnType MotorControlFastLoop_Step(
    MotorControlFastLoop_Type *Object,
    const MotorControlFastLoop_StepInputType *Input,
    MotorControlFastLoop_ResultType *Result)
{
    MotorControlFastLoop_ResultType nextResult = {0};
    MotorControlObserver_StepInputType observerInput;
    MotorControlStartup_StepInputType startupInput;
    MotorControlAlgorithm_DqType voltageDq;
    MotorControlAlgorithm_PiResultType dPiResult;
    MotorControlAlgorithm_PiResultType qPiResult;
    MotorControlFastLoop_ModeType executedMode;
    sint64 dError;
    sint64 qError;
    sint32 sinQ15;
    sint32 cosQ15;
    sint32 currentQRefQ15;

    MotorControlFastLoop_ClearResult(Result);
    if ((Object == NULL_PTR) || (Input == NULL_PTR) ||
        (Result == NULL_PTR))
    {
        return E_NOT_OK;
    }
    if (MotorControlFastLoop_LifecycleKnown(Object->lifecycle) != TRUE)
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        Result->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        Result->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        return E_NOT_OK;
    }
    if (Object->lifecycle != MOTOR_CONTROL_FAST_LOOP_ACTIVE)
    {
        Result->lifecycle = Object->lifecycle;
        Result->failure = Object->failure;
        return E_NOT_OK;
    }
    if (MotorControlFastLoop_RuntimeValid(Object) != TRUE)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL,
            Result);
    }
    if (MotorControlFastLoop_InputRepresentationValid(Input) != TRUE)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_INPUT_REPRESENTATION,
            Result);
    }
    if ((Input->sampleGenerationId !=
         Input->appliedVoltageGenerationId) ||
        (Input->currentSampleValid != TRUE) ||
        (Input->appliedVoltageValid != TRUE) ||
        (Input->generationOwnerValid != TRUE) ||
        ((Object->generationEpochValid == TRUE) &&
         (Input->sampleGenerationId !=
          (Object->lastGenerationId + 1U))))
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR,
            Result);
    }

    executedMode = Object->mode;
    nextResult.mode = executedMode;
    if (MotorControlAlgorithm_Clarke(
            Input->phaseAQ15,
            Input->phaseBQ15,
            Input->phaseCQ15,
            Object->config.phaseSumToleranceQ15,
            &nextResult.measuredCurrent) != E_OK)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM,
            Result);
    }

    observerInput.measuredCurrentAlphaQ15 =
        nextResult.measuredCurrent.alphaQ15;
    observerInput.measuredCurrentBetaQ15 =
        nextResult.measuredCurrent.betaQ15;
    observerInput.appliedVoltageAlphaQ15 = Input->appliedVoltage.alphaQ15;
    observerInput.appliedVoltageBetaQ15 = Input->appliedVoltage.betaQ15;
    observerInput.currentSampleValid = Input->currentSampleValid;
    observerInput.voltageSampleValid = Input->appliedVoltageValid;
    observerInput.generationValid = Input->generationOwnerValid;
    if (MotorControlObserver_Step(
            &Object->observer,
            &observerInput,
            &nextResult.observer) != E_OK)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            ((Object->observer.failure ==
              MOTOR_CONTROL_OBSERVER_FAILURE_INTERNAL) ||
             (Object->observer.failure ==
              MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE)) ?
                MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL :
                MOTOR_CONTROL_FAST_LOOP_FAILURE_OBSERVER,
            Result);
    }

    if (executedMode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP)
    {
        startupInput.observer.angleQ16 = nextResult.observer.angleQ16;
        startupInput.observer.speedAngleCounts =
            nextResult.observer.speedAngleCounts;
        startupInput.observer.numericValid =
            nextResult.observer.numericValid;
        startupInput.observer.angleValid = nextResult.observer.angleValid;
        startupInput.observer.speedValid = nextResult.observer.speedValid;
        startupInput.observer.varianceValid =
            nextResult.observer.varianceValid;
        startupInput.observer.bemfValid = nextResult.observer.bemfValid;
        startupInput.observer.currentValid = nextResult.observer.currentValid;
        startupInput.observer.directionValid =
            nextResult.observer.directionValid;
        startupInput.generationValid = TRUE;
        startupInput.safetyValid = Input->safetyValid;
        startupInput.commandContinueValid = Input->commandContinueValid;
        startupInput.stopRequested = Input->stopRequested;
        if (MotorControlStartup_ValidateCycle(
                &Object->startup,
                &startupInput,
                &currentQRefQ15) != E_OK)
        {
            (void)MotorControlStartup_Step(
                &Object->startup, &startupInput, &nextResult.startup);
            return MotorControlFastLoop_LatchFailure(
                Object,
                (Object->startup.failure ==
                 MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC) ?
                    MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL :
                    MOTOR_CONTROL_FAST_LOOP_FAILURE_STARTUP,
                Result);
        }
        if (MotorControlCdd_ApplyCommissioningIqCap(
                Object, currentQRefQ15) != TRUE)
        {
            return MotorControlFastLoop_LatchFailure(
                Object, MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD, Result);
        }
        if (MotorControlStartup_Step(
                &Object->startup,
                &startupInput,
                &nextResult.startup) != E_OK)
        {
            return MotorControlFastLoop_LatchFailure(
                Object,
                (Object->startup.failure ==
                 MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC) ?
                    MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL :
                    MOTOR_CONTROL_FAST_LOOP_FAILURE_STARTUP,
                Result);
        }
        if (nextResult.startup.stopRequired == TRUE)
        {
            Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED;
            nextResult.stopRequired = TRUE;
            nextResult.lifecycle = MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED;
            nextResult.failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE;
            *Result = nextResult;
            return E_OK;
        }
        nextResult.selectedAngleQ16 = nextResult.startup.selectedAngleQ16;
        nextResult.selectedSpeedAngleCounts =
            nextResult.startup.selectedSpeedAngleCounts;
        nextResult.dRefQ15 = nextResult.startup.dRefQ15;
        nextResult.qRefQ15 = nextResult.startup.qRefQ15;
        nextResult.readyForRun = nextResult.startup.readyForRun;
    }
    else
    {
        nextResult.startup = Object->retainedStartupResult;
        if (MotorControlFastLoop_ObserverReliable(
                Object, &nextResult.observer) != TRUE)
        {
            return MotorControlFastLoop_LatchFailure(
                Object,
                MOTOR_CONTROL_FAST_LOOP_FAILURE_OBSERVER,
                Result);
        }
        if (Input->safetyValid != TRUE)
        {
            return MotorControlFastLoop_LatchFailure(
                Object,
                MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD,
                Result);
        }
        if (MotorControlCdd_ApplyCommissioningIqCap(
                Object, Object->runQRefQ15) != TRUE)
        {
            return MotorControlFastLoop_LatchFailure(
                Object, MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD, Result);
        }
        if ((Input->stopRequested == TRUE) ||
            (Input->commandContinueValid != TRUE))
        {
            Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED;
            nextResult.stopRequired = TRUE;
            nextResult.lifecycle = MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED;
            nextResult.failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE;
            *Result = nextResult;
            return E_OK;
        }
        nextResult.selectedAngleQ16 = nextResult.observer.angleQ16;
        nextResult.selectedSpeedAngleCounts =
            nextResult.observer.speedAngleCounts;
        nextResult.dRefQ15 = Object->runDRefQ15;
        nextResult.qRefQ15 = Object->runQRefQ15;
        nextResult.readyForRun = TRUE;
    }

    if (MotorControlCdd_ApplyCommissioningIqCap(
            Object, nextResult.qRefQ15) != TRUE)
    {
        return MotorControlFastLoop_LatchFailure(
            Object, MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD, Result);
    }

    if (MOTOR_CONTROL_FAST_LOOP_SINCOS_CALL(
            nextResult.selectedAngleQ16,
            &sinQ15,
            &cosQ15) != E_OK)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_NUMERIC,
            Result);
    }
    if (MotorControlAlgorithm_Park(
            &nextResult.measuredCurrent,
            sinQ15,
            cosQ15,
            Object->config.trigNormToleranceQ30,
            &nextResult.measuredCurrentDq) != E_OK)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM,
            Result);
    }
    dError = (sint64)nextResult.dRefQ15 -
             (sint64)nextResult.measuredCurrentDq.dQ15;
    qError = (sint64)nextResult.qRefQ15 -
             (sint64)nextResult.measuredCurrentDq.qQ15;
    if ((dError < (sint64)MOTOR_CONTROL_FAST_LOOP_Q15_MIN) ||
        (dError > (sint64)MOTOR_CONTROL_FAST_LOOP_Q15_MAX) ||
        (qError < (sint64)MOTOR_CONTROL_FAST_LOOP_Q15_MIN) ||
        (qError > (sint64)MOTOR_CONTROL_FAST_LOOP_Q15_MAX))
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_NUMERIC,
            Result);
    }
    if (MotorControlAlgorithm_PiStep(
            &Object->dPi, (sint32)dError, &dPiResult) != E_OK)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM,
            Result);
    }
    if (MotorControlAlgorithm_PiStep(
            &Object->qPi, (sint32)qError, &qPiResult) != E_OK)
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM,
            Result);
    }
    voltageDq.dQ15 = dPiResult.outputQ15;
    voltageDq.qQ15 = qPiResult.outputQ15;
    if ((MotorControlAlgorithm_InversePark(
             &voltageDq,
             sinQ15,
             cosQ15,
             Object->config.trigNormToleranceQ30,
             &nextResult.requestedVoltage) != E_OK) ||
        (MotorControlAlgorithm_Svpwm(
             &nextResult.requestedVoltage,
             &nextResult.requestedSvpwm) != E_OK))
    {
        return MotorControlFastLoop_LatchFailure(
            Object,
            MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM,
            Result);
    }

    if ((executedMode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP) &&
        (Object->startup.phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE) &&
        (nextResult.startup.readyForRun == TRUE))
    {
        Object->retainedStartupResult = nextResult.startup;
        Object->runDRefQ15 = nextResult.startup.dRefQ15;
        Object->runQRefQ15 = nextResult.startup.qRefQ15;
        Object->mode = MOTOR_CONTROL_FAST_LOOP_MODE_RUN;
    }
    Object->lastGenerationId = Input->sampleGenerationId;
    Object->generationEpochValid = TRUE;
    nextResult.sourceGenerationId = Input->sampleGenerationId;
    nextResult.nextGenerationId = Input->sampleGenerationId + 1U;
    nextResult.controlValid = TRUE;
    nextResult.lifecycle = MOTOR_CONTROL_FAST_LOOP_ACTIVE;
    nextResult.failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE;
    *Result = nextResult;
    return E_OK;
}

Std_ReturnType MotorControlFastLoop_ResetFromReady(
    MotorControlFastLoop_Type *Object)
{
    MotorControlFastLoop_Type candidate;

    if (Object == NULL_PTR)
    {
        return E_NOT_OK;
    }
    if (Object->lifecycle == MOTOR_CONTROL_FAST_LOOP_IDLE)
    {
        if (MotorControlFastLoop_RuntimeValid(Object) != TRUE)
        {
            Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
            Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
            return E_NOT_OK;
        }
        return E_OK;
    }
    if (MotorControlFastLoop_LifecycleKnown(Object->lifecycle) != TRUE)
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        return E_NOT_OK;
    }
    if (Object->lifecycle != MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED)
    {
        return E_NOT_OK;
    }
    if (MotorControlFastLoop_RuntimeValid(Object) != TRUE)
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        return E_NOT_OK;
    }

    candidate = *Object;
    if ((MotorControlStartup_RearmFromReady(&candidate.startup) != E_OK) ||
        (MotorControlObserver_ResetFromReady(&candidate.observer) != E_OK) ||
        (MotorControlAlgorithm_PiReset(&candidate.dPi, 0) != E_OK) ||
        (MotorControlAlgorithm_PiReset(&candidate.qPi, 0) != E_OK))
    {
        Object->failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL;
        Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_FAILED;
        return E_NOT_OK;
    }

    candidate.retainedStartupResult =
        (MotorControlStartup_ResultType){0};
    candidate.runDRefQ15 = 0;
    candidate.runQRefQ15 = 0;
    candidate.lastGenerationId = 0U;
    candidate.expectedDirection = 0;
    candidate.lifecycle = MOTOR_CONTROL_FAST_LOOP_IDLE;
    candidate.mode = MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP;
    candidate.failure = MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE;
    candidate.generationEpochValid = FALSE;
    *Object = candidate;
    return E_OK;
}
