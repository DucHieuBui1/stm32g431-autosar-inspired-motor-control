#include <stdint.h>

#include "MotorControlObserver.h"

#define MOTOR_CONTROL_OBSERVER_Q15_SCALE              ((sint64)32768LL)
#define MOTOR_CONTROL_OBSERVER_Q15_HALF               ((uint64)16384ULL)
#define MOTOR_CONTROL_OBSERVER_ANGLE_MASK             ((uint32)65535UL)
#define MOTOR_CONTROL_OBSERVER_MAX_RESIDUAL_SQ         ((uint64)8589672450ULL)
#define MOTOR_CONTROL_OBSERVER_CORDIC_ITERATIONS       ((uint32)15UL)

STATIC const sint32 MotorControlObserver_AtanTable[
    MOTOR_CONTROL_OBSERVER_CORDIC_ITERATIONS] =
{
    8192, 4836, 2555, 1297, 651,
    326, 163, 81, 41, 20, 10, 5, 3, 1, 1
};

STATIC boolean MotorControlObserver_IsBoolean(boolean value)
{
    return ((value == FALSE) || (value == TRUE)) ? TRUE : FALSE;
}

STATIC boolean MotorControlObserver_IsQ15(sint32 value)
{
    return ((value >= MOTOR_CONTROL_OBSERVER_Q15_MIN) &&
            (value <= MOTOR_CONTROL_OBSERVER_Q15_MAX)) ? TRUE : FALSE;
}

STATIC boolean MotorControlObserver_LifecycleValid(
    MotorControlObserver_LifecycleType lifecycle)
{
    return ((lifecycle >= MOTOR_CONTROL_OBSERVER_UNINITIALIZED) &&
            (lifecycle <= MOTOR_CONTROL_OBSERVER_FAILED)) ? TRUE : FALSE;
}

STATIC boolean MotorControlObserver_FailureValid(
    MotorControlObserver_FailureType failure)
{
    return ((failure >= MOTOR_CONTROL_OBSERVER_FAILURE_NONE) &&
            (failure <= MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE)) ?
        TRUE : FALSE;
}

STATIC uint64 MotorControlObserver_Square(sint32 value)
{
    const sint64 wideValue = (sint64)value;

    return (uint64)(wideValue * wideValue);
}

STATIC sint64 MotorControlObserver_Abs64(sint64 value)
{
    return (value < 0) ? -value : value;
}

STATIC boolean MotorControlObserver_AddSint64(
    sint64 left,
    sint64 right,
    sint64 *result)
{
    if (result == NULL_PTR)
    {
        return FALSE;
    }

    if (((right > 0) && (left > (INT64_MAX - right))) ||
        ((right < 0) && (left < (INT64_MIN - right))))
    {
        return FALSE;
    }

    *result = left + right;
    return TRUE;
}

STATIC sint64 MotorControlObserver_RoundQ15(sint64 valueQ15)
{
    uint64 magnitude;
    uint64 rounded;

    if (valueQ15 >= 0)
    {
        magnitude = (uint64)valueQ15;
        rounded = (magnitude + MOTOR_CONTROL_OBSERVER_Q15_HALF) >> 15U;
        return (sint64)rounded;
    }

    magnitude = (uint64)(-valueQ15);
    rounded = (magnitude + MOTOR_CONTROL_OBSERVER_Q15_HALF) >> 15U;
    return -(sint64)rounded;
}

STATIC sint64 MotorControlObserver_ShiftTowardZero(
    sint64 value,
    uint32 shift)
{
    if (value >= 0)
    {
        return (sint64)((uint64)value >> shift);
    }

    return -(sint64)((uint64)(-value) >> shift);
}

STATIC uint16 MotorControlObserver_AddAngle(
    uint16 angleQ16,
    sint32 deltaAngleCounts)
{
    const uint32 wrapped =
        ((uint32)angleQ16 + (uint32)deltaAngleCounts) &
        MOTOR_CONTROL_OBSERVER_ANGLE_MASK;

    return (uint16)wrapped;
}

STATIC sint32 MotorControlObserver_CircularDelta(
    uint16 targetQ16,
    uint16 referenceQ16)
{
    const uint32 delta =
        ((uint32)targetQ16 - (uint32)referenceQ16) &
        MOTOR_CONTROL_OBSERVER_ANGLE_MASK;

    if (delta >= MOTOR_CONTROL_OBSERVER_ANGLE_HALF_TURN)
    {
        return (sint32)((sint64)delta -
                        (sint64)MOTOR_CONTROL_OBSERVER_ANGLE_FULL_TURN);
    }

    return (sint32)delta;
}

STATIC void MotorControlObserver_ClearConfig(
    MotorControlObserver_ConfigType *config)
{
    config->currentRetentionQ15 = 0;
    config->voltageInjectionQ15 = 0;
    config->bemfSubtractionQ15 = 0;
    config->currentErrorToCurrentQ15 = 0;
    config->currentErrorToBemfQ15 = 0;
    config->phaseProportionalQ15 = 0;
    config->phaseIntegralQ15 = 0;
    config->maxAbsCurrentQ15 = 0;
    config->maxAbsBemfQ15 = 0;
    config->maxAbsSpeedAngleCounts = 0U;
    config->maxSpeedDeltaAngleCounts = 0U;
    config->minBemfMagnitudeSqQ30 = 0U;
    config->maxBemfMagnitudeSqQ30 = 0U;
    config->maxCurrentResidualMagnitudeSqQ30 = 0U;
    config->speedWindowLength = 0U;
    config->maxSpeedVarianceNumerator = 0U;
    config->reliabilityWarmupSamples = 0U;
    config->maxConsecutiveUnreliableSamples = 0U;
}

STATIC void MotorControlObserver_CopyConfig(
    MotorControlObserver_ConfigType *destination,
    const MotorControlObserver_ConfigType *source)
{
    destination->currentRetentionQ15 = source->currentRetentionQ15;
    destination->voltageInjectionQ15 = source->voltageInjectionQ15;
    destination->bemfSubtractionQ15 = source->bemfSubtractionQ15;
    destination->currentErrorToCurrentQ15 =
        source->currentErrorToCurrentQ15;
    destination->currentErrorToBemfQ15 = source->currentErrorToBemfQ15;
    destination->phaseProportionalQ15 = source->phaseProportionalQ15;
    destination->phaseIntegralQ15 = source->phaseIntegralQ15;
    destination->maxAbsCurrentQ15 = source->maxAbsCurrentQ15;
    destination->maxAbsBemfQ15 = source->maxAbsBemfQ15;
    destination->maxAbsSpeedAngleCounts = source->maxAbsSpeedAngleCounts;
    destination->maxSpeedDeltaAngleCounts =
        source->maxSpeedDeltaAngleCounts;
    destination->minBemfMagnitudeSqQ30 = source->minBemfMagnitudeSqQ30;
    destination->maxBemfMagnitudeSqQ30 = source->maxBemfMagnitudeSqQ30;
    destination->maxCurrentResidualMagnitudeSqQ30 =
        source->maxCurrentResidualMagnitudeSqQ30;
    destination->speedWindowLength = source->speedWindowLength;
    destination->maxSpeedVarianceNumerator =
        source->maxSpeedVarianceNumerator;
    destination->reliabilityWarmupSamples =
        source->reliabilityWarmupSamples;
    destination->maxConsecutiveUnreliableSamples =
        source->maxConsecutiveUnreliableSamples;
}

STATIC void MotorControlObserver_ClearRuntime(
    MotorControlObserver_ObjectType *object)
{
    uint32 index;

    object->estimatedCurrentAlphaQ15 = 0;
    object->estimatedCurrentBetaQ15 = 0;
    object->estimatedBemfAlphaQ15 = 0;
    object->estimatedBemfBetaQ15 = 0;
    object->angleQ16 = 0U;
    object->speedIntegratorQ15 = 0;
    object->speedAngleCounts = 0;
    for (index = 0U;
         index < MOTOR_CONTROL_OBSERVER_MAX_SPEED_WINDOW;
         index++)
    {
        object->speedRing[index] = 0;
    }
    object->speedSum = 0;
    object->speedSumSquares = 0U;
    object->speedWindowFill = 0U;
    object->speedWindowIndex = 0U;
    object->warmupSamples = 0U;
    object->consecutiveUnreliableSamples = 0U;
    object->expectedDirection = 0;
    object->lifecycle = MOTOR_CONTROL_OBSERVER_IDLE;
    object->failure = MOTOR_CONTROL_OBSERVER_FAILURE_NONE;
}

STATIC void MotorControlObserver_ClearObject(
    MotorControlObserver_ObjectType *object)
{
    MotorControlObserver_ClearConfig(&object->config);
    MotorControlObserver_ClearRuntime(object);
    object->lifecycle = MOTOR_CONTROL_OBSERVER_UNINITIALIZED;
}

STATIC void MotorControlObserver_ClearResult(
    MotorControlObserver_ResultType *result)
{
    result->estimatedCurrentAlphaQ15 = 0;
    result->estimatedCurrentBetaQ15 = 0;
    result->estimatedBemfAlphaQ15 = 0;
    result->estimatedBemfBetaQ15 = 0;
    result->angleQ16 = 0U;
    result->speedAngleCounts = 0;
    result->currentResidualMagnitudeSqQ30 = 0U;
    result->bemfMagnitudeSqQ30 = 0U;
    result->speedVarianceNumerator = 0U;
    result->consecutiveUnreliableSamples = 0U;
    result->numericValid = FALSE;
    result->angleValid = FALSE;
    result->speedValid = FALSE;
    result->varianceValid = FALSE;
    result->bemfValid = FALSE;
    result->currentValid = FALSE;
    result->directionValid = FALSE;
    result->lifecycle = MOTOR_CONTROL_OBSERVER_UNINITIALIZED;
    result->failure = MOTOR_CONTROL_OBSERVER_FAILURE_NONE;
}

STATIC void MotorControlObserver_FillFailureResult(
    const MotorControlObserver_ObjectType *object,
    MotorControlObserver_ResultType *result)
{
    MotorControlObserver_ClearResult(result);
    if (object != NULL_PTR)
    {
        result->estimatedCurrentAlphaQ15 = object->estimatedCurrentAlphaQ15;
        result->estimatedCurrentBetaQ15 = object->estimatedCurrentBetaQ15;
        result->estimatedBemfAlphaQ15 = object->estimatedBemfAlphaQ15;
        result->estimatedBemfBetaQ15 = object->estimatedBemfBetaQ15;
        result->angleQ16 = object->angleQ16;
        result->speedAngleCounts = object->speedAngleCounts;
        result->consecutiveUnreliableSamples =
            object->consecutiveUnreliableSamples;
        result->lifecycle = object->lifecycle;
        result->failure = object->failure;
    }
}

STATIC boolean MotorControlObserver_ConfigValid(
    const MotorControlObserver_ConfigType *config)
{
    uint64 maxBemfMagnitude;
    uint64 maxVarianceNumerator;
    uint64 speedSquared;
    uint64 windowSquared;

    if (config == NULL_PTR)
    {
        return FALSE;
    }

    if ((MotorControlObserver_IsQ15(config->currentRetentionQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(config->voltageInjectionQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(config->bemfSubtractionQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(
            config->currentErrorToCurrentQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(
            config->currentErrorToBemfQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(config->phaseProportionalQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(config->phaseIntegralQ15) != TRUE))
    {
        return FALSE;
    }

    if ((config->currentRetentionQ15 < 0) ||
        (config->voltageInjectionQ15 < 0) ||
        (config->bemfSubtractionQ15 < 0) ||
        (config->currentErrorToCurrentQ15 < 0) ||
        (config->phaseProportionalQ15 < 0) ||
        (config->phaseIntegralQ15 < 0))
    {
        return FALSE;
    }

    if ((config->maxAbsCurrentQ15 <= 0) ||
        (config->maxAbsCurrentQ15 > MOTOR_CONTROL_OBSERVER_Q15_MAX) ||
        (config->maxAbsBemfQ15 <= 0) ||
        (config->maxAbsBemfQ15 > MOTOR_CONTROL_OBSERVER_Q15_MAX) ||
        (config->maxAbsSpeedAngleCounts == 0U) ||
        (config->maxAbsSpeedAngleCounts >
            (uint32)MOTOR_CONTROL_OBSERVER_Q15_MAX) ||
        (config->maxSpeedDeltaAngleCounts == 0U) ||
        ((uint64)config->maxSpeedDeltaAngleCounts >
            (2ULL * (uint64)config->maxAbsSpeedAngleCounts)))
    {
        return FALSE;
    }

    if ((config->speedWindowLength < 2U) ||
        (config->speedWindowLength >
            MOTOR_CONTROL_OBSERVER_MAX_SPEED_WINDOW) ||
        ((config->speedWindowLength &
          (config->speedWindowLength - 1U)) != 0U) ||
        (config->reliabilityWarmupSamples < config->speedWindowLength) ||
        (config->maxConsecutiveUnreliableSamples == 0U))
    {
        return FALSE;
    }

    maxBemfMagnitude =
        2ULL * (uint64)config->maxAbsBemfQ15 *
        (uint64)config->maxAbsBemfQ15;
    if ((config->minBemfMagnitudeSqQ30 == 0U) ||
        (config->minBemfMagnitudeSqQ30 >
            config->maxBemfMagnitudeSqQ30) ||
        (config->maxBemfMagnitudeSqQ30 > maxBemfMagnitude) ||
        (config->maxCurrentResidualMagnitudeSqQ30 == 0U) ||
        (config->maxCurrentResidualMagnitudeSqQ30 >
            MOTOR_CONTROL_OBSERVER_MAX_RESIDUAL_SQ))
    {
        return FALSE;
    }

    speedSquared = (uint64)config->maxAbsSpeedAngleCounts *
                   (uint64)config->maxAbsSpeedAngleCounts;
    windowSquared = (uint64)config->speedWindowLength *
                    (uint64)config->speedWindowLength;
    maxVarianceNumerator = windowSquared * speedSquared;
    if ((config->maxSpeedVarianceNumerator == 0U) ||
        (config->maxSpeedVarianceNumerator > maxVarianceNumerator))
    {
        return FALSE;
    }

    return TRUE;
}

STATIC boolean MotorControlObserver_RuntimeObjectValid(
    const MotorControlObserver_ObjectType *object)
{
    sint64 integratorLimit;
    sint64 recomputedSum = 0;
    uint64 recomputedSquares = 0U;
    uint32 index;

    if ((object->lifecycle != MOTOR_CONTROL_OBSERVER_ACTIVE) ||
        (object->failure != MOTOR_CONTROL_OBSERVER_FAILURE_NONE) ||
        (MotorControlObserver_ConfigValid(&object->config) != TRUE) ||
        ((object->expectedDirection != 1) &&
         (object->expectedDirection != -1)))
    {
        return FALSE;
    }

    if ((object->estimatedCurrentAlphaQ15 <
            -object->config.maxAbsCurrentQ15) ||
        (object->estimatedCurrentAlphaQ15 >
            object->config.maxAbsCurrentQ15) ||
        (object->estimatedCurrentBetaQ15 <
            -object->config.maxAbsCurrentQ15) ||
        (object->estimatedCurrentBetaQ15 >
            object->config.maxAbsCurrentQ15) ||
        (object->estimatedBemfAlphaQ15 <
            -object->config.maxAbsBemfQ15) ||
        (object->estimatedBemfAlphaQ15 >
            object->config.maxAbsBemfQ15) ||
        (object->estimatedBemfBetaQ15 <
            -object->config.maxAbsBemfQ15) ||
        (object->estimatedBemfBetaQ15 >
            object->config.maxAbsBemfQ15))
    {
        return FALSE;
    }

    integratorLimit =
        (sint64)object->config.maxAbsSpeedAngleCounts *
        MOTOR_CONTROL_OBSERVER_Q15_SCALE;
    if ((object->speedIntegratorQ15 < -integratorLimit) ||
        (object->speedIntegratorQ15 > integratorLimit) ||
        (object->speedAngleCounts <
            -(sint32)object->config.maxAbsSpeedAngleCounts) ||
        (object->speedAngleCounts >
            (sint32)object->config.maxAbsSpeedAngleCounts) ||
        ((sint32)MotorControlObserver_RoundQ15(
            object->speedIntegratorQ15) != object->speedAngleCounts))
    {
        return FALSE;
    }

    if ((object->speedWindowFill > object->config.speedWindowLength) ||
        (object->speedWindowIndex >= object->config.speedWindowLength) ||
        (object->warmupSamples > object->config.reliabilityWarmupSamples) ||
        (object->consecutiveUnreliableSamples >
            object->config.maxConsecutiveUnreliableSamples))
    {
        return FALSE;
    }

    if (((object->speedWindowFill < object->config.speedWindowLength) &&
         (object->speedWindowIndex != object->speedWindowFill)) ||
        ((object->warmupSamples < object->config.speedWindowLength) &&
         (object->speedWindowFill != object->warmupSamples)) ||
        ((object->warmupSamples >= object->config.speedWindowLength) &&
         (object->speedWindowFill != object->config.speedWindowLength)))
    {
        return FALSE;
    }

    for (index = 0U;
         index < MOTOR_CONTROL_OBSERVER_MAX_SPEED_WINDOW;
         index++)
    {
        const boolean entryActive =
            ((index < object->config.speedWindowLength) &&
             ((object->speedWindowFill ==
                object->config.speedWindowLength) ||
              (index < object->speedWindowFill))) ? TRUE : FALSE;

        if (entryActive == TRUE)
        {
            if ((object->speedRing[index] <
                    -(sint32)object->config.maxAbsSpeedAngleCounts) ||
                (object->speedRing[index] >
                    (sint32)object->config.maxAbsSpeedAngleCounts))
            {
                return FALSE;
            }
            recomputedSum += (sint64)object->speedRing[index];
            recomputedSquares +=
                MotorControlObserver_Square(object->speedRing[index]);
        }
        else if (object->speedRing[index] != 0)
        {
            return FALSE;
        }
        else
        {
            /* Empty entries remain zero. */
        }
    }

    if ((recomputedSum != object->speedSum) ||
        (recomputedSquares != object->speedSumSquares))
    {
        return FALSE;
    }

    return TRUE;
}

STATIC Std_ReturnType MotorControlObserver_CordicAngle(
    sint32 alphaQ15,
    sint32 betaQ15,
    uint16 *angleQ16)
{
    sint64 x = (sint64)alphaQ15;
    sint64 y = (sint64)betaQ15;
    sint64 z = 0;
    uint32 index;

    if (angleQ16 == NULL_PTR)
    {
        return E_NOT_OK;
    }

    if ((x == 0) && (y == 0))
    {
        *angleQ16 = 0U;
        return E_NOT_OK;
    }

    if (y == 0)
    {
        *angleQ16 = (x > 0) ? 0U : (uint16)0x8000U;
        return E_OK;
    }
    if (x == 0)
    {
        *angleQ16 = (y > 0) ? (uint16)0x4000U : (uint16)0xC000U;
        return E_OK;
    }

    for (index = 0U;
         index < MOTOR_CONTROL_OBSERVER_CORDIC_ITERATIONS;
         index++)
    {
        if ((MotorControlObserver_Abs64(x) < 16384) &&
            (MotorControlObserver_Abs64(y) < 16384))
        {
            x *= 2LL;
            y *= 2LL;
        }
    }

    if (x < 0)
    {
        x = -x;
        y = -y;
        z = (sint64)MOTOR_CONTROL_OBSERVER_ANGLE_HALF_TURN;
    }

    for (index = 0U;
         index < MOTOR_CONTROL_OBSERVER_CORDIC_ITERATIONS;
         index++)
    {
        const sint64 priorX = x;
        const sint64 priorY = y;

        if (priorY > 0)
        {
            x = priorX +
                MotorControlObserver_ShiftTowardZero(priorY, index);
            y = priorY -
                MotorControlObserver_ShiftTowardZero(priorX, index);
            z += (sint64)MotorControlObserver_AtanTable[index];
        }
        else if (priorY < 0)
        {
            x = priorX -
                MotorControlObserver_ShiftTowardZero(priorY, index);
            y = priorY +
                MotorControlObserver_ShiftTowardZero(priorX, index);
            z -= (sint64)MotorControlObserver_AtanTable[index];
        }
        else
        {
            /* An exact vectoring tie is a deterministic no-op. */
        }
    }

    *angleQ16 = (uint16)((uint64)z &
                          (uint64)MOTOR_CONTROL_OBSERVER_ANGLE_MASK);
    return E_OK;
}

STATIC MotorControlObserver_FailureType MotorControlObserver_ComputeAxis(
    sint32 measuredCurrentQ15,
    sint32 appliedVoltageQ15,
    sint32 estimatedCurrentQ15,
    sint32 estimatedBemfQ15,
    const MotorControlObserver_ConfigType *config,
    sint32 *nextEstimatedCurrentQ15,
    sint32 *nextEstimatedBemfQ15,
    sint32 *currentErrorQ15)
{
    sint64 accumulator;
    sint64 temporary;
    sint64 modelCurrent;
    sint64 error;
    sint64 nextCurrent;
    sint64 nextBemf;

    accumulator =
        (sint64)config->currentRetentionQ15 *
        (sint64)estimatedCurrentQ15;
    if (MotorControlObserver_AddSint64(
            accumulator,
            (sint64)config->voltageInjectionQ15 *
                (sint64)appliedVoltageQ15,
            &temporary) != TRUE)
    {
        return MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC;
    }
    accumulator = temporary;
    if (MotorControlObserver_AddSint64(
            accumulator,
            -((sint64)config->bemfSubtractionQ15 *
              (sint64)estimatedBemfQ15),
            &temporary) != TRUE)
    {
        return MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC;
    }

    modelCurrent = MotorControlObserver_RoundQ15(temporary);
    if ((modelCurrent < -(sint64)config->maxAbsCurrentQ15) ||
        (modelCurrent > (sint64)config->maxAbsCurrentQ15))
    {
        return MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE;
    }

    error = (sint64)measuredCurrentQ15 - modelCurrent;
    if ((error < -65535LL) || (error > 65535LL))
    {
        return MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC;
    }

    accumulator = modelCurrent * MOTOR_CONTROL_OBSERVER_Q15_SCALE;
    if (MotorControlObserver_AddSint64(
            accumulator,
            (sint64)config->currentErrorToCurrentQ15 * error,
            &temporary) != TRUE)
    {
        return MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC;
    }
    nextCurrent = MotorControlObserver_RoundQ15(temporary);

    accumulator =
        (sint64)estimatedBemfQ15 *
        MOTOR_CONTROL_OBSERVER_Q15_SCALE;
    if (MotorControlObserver_AddSint64(
            accumulator,
            (sint64)config->currentErrorToBemfQ15 * error,
            &temporary) != TRUE)
    {
        return MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC;
    }
    nextBemf = MotorControlObserver_RoundQ15(temporary);

    if ((nextCurrent < -(sint64)config->maxAbsCurrentQ15) ||
        (nextCurrent > (sint64)config->maxAbsCurrentQ15) ||
        (nextBemf < -(sint64)config->maxAbsBemfQ15) ||
        (nextBemf > (sint64)config->maxAbsBemfQ15))
    {
        return MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE;
    }

    *nextEstimatedCurrentQ15 = (sint32)nextCurrent;
    *nextEstimatedBemfQ15 = (sint32)nextBemf;
    *currentErrorQ15 = (sint32)error;
    return MOTOR_CONTROL_OBSERVER_FAILURE_NONE;
}

STATIC Std_ReturnType MotorControlObserver_LatchFailure(
    MotorControlObserver_ObjectType *object,
    MotorControlObserver_FailureType failure,
    MotorControlObserver_ResultType *result)
{
    if ((MotorControlObserver_FailureValid(object->failure) != TRUE) ||
        (object->failure == MOTOR_CONTROL_OBSERVER_FAILURE_NONE))
    {
        object->failure = failure;
    }
    object->lifecycle = MOTOR_CONTROL_OBSERVER_FAILED;
    MotorControlObserver_FillFailureResult(object, result);
    return E_NOT_OK;
}

Std_ReturnType MotorControlObserver_Init(
    MotorControlObserver_ObjectType *Object,
    const MotorControlObserver_ConfigType *Config)
{
    if ((Object == NULL_PTR) || (Config == NULL_PTR))
    {
        return E_NOT_OK;
    }

    MotorControlObserver_ClearObject(Object);
    if (MotorControlObserver_ConfigValid(Config) != TRUE)
    {
        Object->failure = MOTOR_CONTROL_OBSERVER_FAILURE_CONFIGURATION;
        return E_NOT_OK;
    }

    MotorControlObserver_CopyConfig(&Object->config, Config);
    Object->lifecycle = MOTOR_CONTROL_OBSERVER_IDLE;
    Object->failure = MOTOR_CONTROL_OBSERVER_FAILURE_NONE;
    return E_OK;
}

Std_ReturnType MotorControlObserver_Begin(
    MotorControlObserver_ObjectType *Object,
    uint16 InitialAngleQ16,
    sint32 ExpectedDirection)
{
    MotorControlObserver_ConfigType retainedConfig;

    if ((Object == NULL_PTR) ||
        (Object->lifecycle != MOTOR_CONTROL_OBSERVER_IDLE) ||
        (Object->failure != MOTOR_CONTROL_OBSERVER_FAILURE_NONE) ||
        ((ExpectedDirection != 1) && (ExpectedDirection != -1)) ||
        (MotorControlObserver_ConfigValid(&Object->config) != TRUE))
    {
        return E_NOT_OK;
    }

    MotorControlObserver_CopyConfig(&retainedConfig, &Object->config);
    MotorControlObserver_ClearRuntime(Object);
    MotorControlObserver_CopyConfig(&Object->config, &retainedConfig);
    Object->angleQ16 = InitialAngleQ16;
    Object->expectedDirection = ExpectedDirection;
    Object->lifecycle = MOTOR_CONTROL_OBSERVER_ACTIVE;
    return E_OK;
}

Std_ReturnType MotorControlObserver_ResetFromReady(
    MotorControlObserver_ObjectType *Object)
{
    MotorControlObserver_ConfigType retainedConfig;

    if ((Object == NULL_PTR) ||
        ((Object->lifecycle != MOTOR_CONTROL_OBSERVER_IDLE) &&
         (Object->lifecycle != MOTOR_CONTROL_OBSERVER_ACTIVE)) ||
        (Object->failure != MOTOR_CONTROL_OBSERVER_FAILURE_NONE) ||
        (MotorControlObserver_ConfigValid(&Object->config) != TRUE))
    {
        return E_NOT_OK;
    }

    MotorControlObserver_CopyConfig(&retainedConfig, &Object->config);
    MotorControlObserver_ClearRuntime(Object);
    MotorControlObserver_CopyConfig(&Object->config, &retainedConfig);
    return E_OK;
}

Std_ReturnType MotorControlObserver_Step(
    MotorControlObserver_ObjectType *Object,
    const MotorControlObserver_StepInputType *Input,
    MotorControlObserver_ResultType *Result)
{
    MotorControlObserver_FailureType calculationFailure;
    sint32 nextCurrentAlpha;
    sint32 nextCurrentBeta;
    sint32 nextBemfAlpha;
    sint32 nextBemfBeta;
    sint32 currentErrorAlpha;
    sint32 currentErrorBeta;
    uint64 residualMagnitude;
    uint64 bemfMagnitude;
    uint16 bemfAngle = 0U;
    uint16 rawRotorAngle = 0U;
    uint16 predictedAngle = 0U;
    uint16 candidateAngle = 0U;
    sint32 phaseError = 0;
    sint64 integralCorrection = 0;
    sint64 candidateIntegrator = 0;
    sint64 integratorLimit;
    sint64 candidateSpeedWide = 0;
    sint32 candidateSpeed = 0;
    sint64 proportionalCorrectionWide = 0;
    sint32 proportionalCorrection = 0;
    sint64 speedDelta = 0;
    boolean phaseAccepted = FALSE;
    sint64 nextSpeedSum = 0;
    uint64 nextSpeedSumSquares = 0U;
    uint32 nextWindowFill = 0U;
    uint32 nextWindowIndex = 0U;
    uint32 nextWarmupSamples = 0U;
    uint64 varianceNumerator = 0U;
    sint32 replacedSpeed = 0;
    uint64 firstVarianceProduct = 0U;
    uint64 secondVarianceProduct = 0U;
    uint32 nextUnreliable;
    boolean angleValid;
    boolean speedValid;
    boolean varianceValid;
    boolean bemfValid;
    boolean currentValid;
    boolean directionValid;
    boolean allReliable;
    uint32 index;

    if (Result != NULL_PTR)
    {
        MotorControlObserver_ClearResult(Result);
    }

    if ((Object == NULL_PTR) || (Input == NULL_PTR) || (Result == NULL_PTR))
    {
        return E_NOT_OK;
    }

    if (MotorControlObserver_LifecycleValid(Object->lifecycle) != TRUE)
    {
        return MotorControlObserver_LatchFailure(
            Object, MOTOR_CONTROL_OBSERVER_FAILURE_INTERNAL, Result);
    }

    if (Object->lifecycle == MOTOR_CONTROL_OBSERVER_FAILED)
    {
        MotorControlObserver_FillFailureResult(Object, Result);
        return E_NOT_OK;
    }

    if (Object->lifecycle != MOTOR_CONTROL_OBSERVER_ACTIVE)
    {
        MotorControlObserver_FillFailureResult(Object, Result);
        return E_NOT_OK;
    }

    if (MotorControlObserver_RuntimeObjectValid(Object) != TRUE)
    {
        return MotorControlObserver_LatchFailure(
            Object, MOTOR_CONTROL_OBSERVER_FAILURE_INTERNAL, Result);
    }

    if ((MotorControlObserver_IsBoolean(Input->currentSampleValid) != TRUE) ||
        (MotorControlObserver_IsBoolean(Input->voltageSampleValid) != TRUE) ||
        (MotorControlObserver_IsBoolean(Input->generationValid) != TRUE) ||
        (MotorControlObserver_IsQ15(Input->measuredCurrentAlphaQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(Input->measuredCurrentBetaQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(Input->appliedVoltageAlphaQ15) != TRUE) ||
        (MotorControlObserver_IsQ15(Input->appliedVoltageBetaQ15) != TRUE))
    {
        return MotorControlObserver_LatchFailure(
            Object,
            MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_REPRESENTATION,
            Result);
    }

    if ((Input->currentSampleValid != TRUE) ||
        (Input->voltageSampleValid != TRUE) ||
        (Input->generationValid != TRUE))
    {
        return MotorControlObserver_LatchFailure(
            Object, MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_QUALITY, Result);
    }

    calculationFailure = MotorControlObserver_ComputeAxis(
        Input->measuredCurrentAlphaQ15,
        Input->appliedVoltageAlphaQ15,
        Object->estimatedCurrentAlphaQ15,
        Object->estimatedBemfAlphaQ15,
        &Object->config,
        &nextCurrentAlpha,
        &nextBemfAlpha,
        &currentErrorAlpha);
    if (calculationFailure != MOTOR_CONTROL_OBSERVER_FAILURE_NONE)
    {
        return MotorControlObserver_LatchFailure(
            Object, calculationFailure, Result);
    }

    calculationFailure = MotorControlObserver_ComputeAxis(
        Input->measuredCurrentBetaQ15,
        Input->appliedVoltageBetaQ15,
        Object->estimatedCurrentBetaQ15,
        Object->estimatedBemfBetaQ15,
        &Object->config,
        &nextCurrentBeta,
        &nextBemfBeta,
        &currentErrorBeta);
    if (calculationFailure != MOTOR_CONTROL_OBSERVER_FAILURE_NONE)
    {
        return MotorControlObserver_LatchFailure(
            Object, calculationFailure, Result);
    }

    residualMagnitude =
        MotorControlObserver_Square(currentErrorAlpha) +
        MotorControlObserver_Square(currentErrorBeta);
    bemfMagnitude =
        MotorControlObserver_Square(nextBemfAlpha) +
        MotorControlObserver_Square(nextBemfBeta);

    if ((nextBemfAlpha != 0) || (nextBemfBeta != 0))
    {
        if (MotorControlObserver_CordicAngle(
                nextBemfAlpha, nextBemfBeta, &bemfAngle) != E_OK)
        {
            return MotorControlObserver_LatchFailure(
                Object, MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC, Result);
        }

        rawRotorAngle = (Object->expectedDirection > 0) ?
            MotorControlObserver_AddAngle(
                bemfAngle,
                -(sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN) :
            MotorControlObserver_AddAngle(
                bemfAngle,
                (sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
        predictedAngle = MotorControlObserver_AddAngle(
            Object->angleQ16, Object->speedAngleCounts);
        phaseError = MotorControlObserver_CircularDelta(
            rawRotorAngle, predictedAngle);

        integralCorrection =
            (sint64)Object->config.phaseIntegralQ15 *
            (sint64)phaseError;
        if (MotorControlObserver_AddSint64(
                Object->speedIntegratorQ15,
                integralCorrection,
                &candidateIntegrator) != TRUE)
        {
            return MotorControlObserver_LatchFailure(
                Object, MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC, Result);
        }

        candidateSpeedWide =
            MotorControlObserver_RoundQ15(candidateIntegrator);
        proportionalCorrectionWide = MotorControlObserver_RoundQ15(
            (sint64)Object->config.phaseProportionalQ15 *
            (sint64)phaseError);
        if ((candidateSpeedWide >= INT32_MIN) &&
            (candidateSpeedWide <= INT32_MAX) &&
            (proportionalCorrectionWide >= INT32_MIN) &&
            (proportionalCorrectionWide <= INT32_MAX))
        {
            candidateSpeed = (sint32)candidateSpeedWide;
            proportionalCorrection = (sint32)proportionalCorrectionWide;
            candidateAngle = MotorControlObserver_AddAngle(
                predictedAngle, proportionalCorrection);
            speedDelta = (sint64)candidateSpeed -
                         (sint64)Object->speedAngleCounts;
            integratorLimit =
                (sint64)Object->config.maxAbsSpeedAngleCounts *
                MOTOR_CONTROL_OBSERVER_Q15_SCALE;

            if ((candidateIntegrator >= -integratorLimit) &&
                (candidateIntegrator <= integratorLimit) &&
                (candidateSpeed >=
                    -(sint32)Object->config.maxAbsSpeedAngleCounts) &&
                (candidateSpeed <=
                    (sint32)Object->config.maxAbsSpeedAngleCounts) &&
                (speedDelta >=
                    -(sint64)Object->config.maxSpeedDeltaAngleCounts) &&
                (speedDelta <=
                    (sint64)Object->config.maxSpeedDeltaAngleCounts))
            {
                phaseAccepted = TRUE;
            }
        }
    }

    if (phaseAccepted == TRUE)
    {
        replacedSpeed =
            (Object->speedWindowFill < Object->config.speedWindowLength) ?
                0 : Object->speedRing[Object->speedWindowIndex];
        nextSpeedSum = Object->speedSum - (sint64)replacedSpeed +
                       (sint64)candidateSpeed;
        nextSpeedSumSquares =
            Object->speedSumSquares -
            MotorControlObserver_Square(replacedSpeed) +
            MotorControlObserver_Square(candidateSpeed);
        nextWindowFill =
            (Object->speedWindowFill < Object->config.speedWindowLength) ?
                (Object->speedWindowFill + 1U) :
                Object->config.speedWindowLength;
        nextWindowIndex =
            (Object->speedWindowIndex + 1U) &
            (Object->config.speedWindowLength - 1U);
        nextWarmupSamples =
            (Object->warmupSamples <
                Object->config.reliabilityWarmupSamples) ?
                (Object->warmupSamples + 1U) :
                Object->config.reliabilityWarmupSamples;

        if (nextWindowFill == Object->config.speedWindowLength)
        {
            firstVarianceProduct =
                (uint64)Object->config.speedWindowLength *
                nextSpeedSumSquares;
            secondVarianceProduct =
                (uint64)MotorControlObserver_Abs64(nextSpeedSum) *
                (uint64)MotorControlObserver_Abs64(nextSpeedSum);
            if (firstVarianceProduct < secondVarianceProduct)
            {
                return MotorControlObserver_LatchFailure(
                    Object, MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC, Result);
            }
            varianceNumerator =
                firstVarianceProduct - secondVarianceProduct;
        }
    }

    angleValid = phaseAccepted;
    speedValid = phaseAccepted;
    varianceValid =
        ((phaseAccepted == TRUE) &&
         (nextWindowFill == Object->config.speedWindowLength) &&
         (nextWarmupSamples >= Object->config.reliabilityWarmupSamples) &&
         (varianceNumerator <=
            Object->config.maxSpeedVarianceNumerator)) ? TRUE : FALSE;
    bemfValid =
        ((bemfMagnitude >= Object->config.minBemfMagnitudeSqQ30) &&
         (bemfMagnitude <= Object->config.maxBemfMagnitudeSqQ30)) ?
        TRUE : FALSE;
    currentValid =
        (residualMagnitude <=
            Object->config.maxCurrentResidualMagnitudeSqQ30) ? TRUE : FALSE;
    directionValid =
        ((phaseAccepted == TRUE) && (candidateSpeed != 0) &&
         (((candidateSpeed > 0) && (Object->expectedDirection > 0)) ||
          ((candidateSpeed < 0) && (Object->expectedDirection < 0)))) ?
        TRUE : FALSE;
    allReliable =
        ((angleValid == TRUE) &&
         (speedValid == TRUE) &&
         (varianceValid == TRUE) &&
         (bemfValid == TRUE) &&
         (currentValid == TRUE) &&
         (directionValid == TRUE)) ? TRUE : FALSE;

    if (allReliable == TRUE)
    {
        nextUnreliable = 0U;
    }
    else if (Object->consecutiveUnreliableSamples <
             Object->config.maxConsecutiveUnreliableSamples)
    {
        nextUnreliable = Object->consecutiveUnreliableSamples + 1U;
    }
    else
    {
        nextUnreliable = Object->config.maxConsecutiveUnreliableSamples;
    }

    Object->estimatedCurrentAlphaQ15 = nextCurrentAlpha;
    Object->estimatedCurrentBetaQ15 = nextCurrentBeta;
    Object->estimatedBemfAlphaQ15 = nextBemfAlpha;
    Object->estimatedBemfBetaQ15 = nextBemfBeta;
    if (phaseAccepted == TRUE)
    {
        Object->speedIntegratorQ15 = candidateIntegrator;
        Object->speedAngleCounts = candidateSpeed;
        Object->angleQ16 = candidateAngle;
        Object->speedRing[Object->speedWindowIndex] = candidateSpeed;
        Object->speedSum = nextSpeedSum;
        Object->speedSumSquares = nextSpeedSumSquares;
        Object->speedWindowFill = nextWindowFill;
        Object->speedWindowIndex = nextWindowIndex;
        Object->warmupSamples = nextWarmupSamples;
    }
    else
    {
        for (index = 0U;
             index < MOTOR_CONTROL_OBSERVER_MAX_SPEED_WINDOW;
             index++)
        {
            Object->speedRing[index] = 0;
        }
        Object->speedSum = 0;
        Object->speedSumSquares = 0U;
        Object->speedWindowFill = 0U;
        Object->speedWindowIndex = 0U;
        Object->warmupSamples = 0U;
    }
    Object->consecutiveUnreliableSamples = nextUnreliable;

    Result->estimatedCurrentAlphaQ15 = nextCurrentAlpha;
    Result->estimatedCurrentBetaQ15 = nextCurrentBeta;
    Result->estimatedBemfAlphaQ15 = nextBemfAlpha;
    Result->estimatedBemfBetaQ15 = nextBemfBeta;
    Result->angleQ16 = Object->angleQ16;
    Result->speedAngleCounts = Object->speedAngleCounts;
    Result->currentResidualMagnitudeSqQ30 = residualMagnitude;
    Result->bemfMagnitudeSqQ30 = bemfMagnitude;
    Result->speedVarianceNumerator = varianceNumerator;
    Result->consecutiveUnreliableSamples = nextUnreliable;
    Result->numericValid = TRUE;
    Result->angleValid = angleValid;
    Result->speedValid = speedValid;
    Result->varianceValid = varianceValid;
    Result->bemfValid = bemfValid;
    Result->currentValid = currentValid;
    Result->directionValid = directionValid;
    Result->lifecycle = Object->lifecycle;
    Result->failure = Object->failure;
    return E_OK;
}
