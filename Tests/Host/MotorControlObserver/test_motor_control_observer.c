#include <math.h>
#include <stdio.h>
#include <string.h>

#include "MotorControlObserver.h"

#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlObserver.c"

#define TEST_PI 3.14159265358979323846

#define TEST_CHECK(condition)                                                \
    do                                                                       \
    {                                                                        \
        if (!(condition))                                                    \
        {                                                                    \
            (void)printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__,           \
                         #condition);                                        \
            return 1;                                                        \
        }                                                                    \
    } while (0)

STATIC MotorControlObserver_ConfigType Test_Config(void)
{
    MotorControlObserver_ConfigType config;

    config.currentRetentionQ15 = 0;
    config.voltageInjectionQ15 = 0;
    config.bemfSubtractionQ15 = 0;
    config.currentErrorToCurrentQ15 = 0;
    config.currentErrorToBemfQ15 = 32767;
    config.phaseProportionalQ15 = 0;
    config.phaseIntegralQ15 = 32767;
    config.maxAbsCurrentQ15 = 32767;
    config.maxAbsBemfQ15 = 32767;
    config.maxAbsSpeedAngleCounts = 2048U;
    config.maxSpeedDeltaAngleCounts = 2048U;
    config.minBemfMagnitudeSqQ30 = 1U;
    config.maxBemfMagnitudeSqQ30 = 2147352578ULL;
    config.maxCurrentResidualMagnitudeSqQ30 = 8589672450ULL;
    config.speedWindowLength = 4U;
    config.maxSpeedVarianceNumerator = 67108864ULL;
    config.reliabilityWarmupSamples = 4U;
    config.maxConsecutiveUnreliableSamples = 3U;
    return config;
}

STATIC MotorControlObserver_StepInputType Test_Input(void)
{
    MotorControlObserver_StepInputType input;

    input.measuredCurrentAlphaQ15 = 0;
    input.measuredCurrentBetaQ15 = 0;
    input.appliedVoltageAlphaQ15 = 0;
    input.appliedVoltageBetaQ15 = 0;
    input.currentSampleValid = TRUE;
    input.voltageSampleValid = TRUE;
    input.generationValid = TRUE;
    return input;
}

STATIC sint64 Test_RoundQ15(sint64 value)
{
    return (value >= 0) ?
        ((value + 16384) >> 15U) :
        -(((-value) + 16384) >> 15U);
}

STATIC void Test_ReferenceAxis(
    sint32 measuredCurrentQ15,
    sint32 appliedVoltageQ15,
    sint32 estimatedCurrentQ15,
    sint32 estimatedBemfQ15,
    const MotorControlObserver_ConfigType *config,
    sint32 *nextCurrentQ15,
    sint32 *nextBemfQ15,
    sint32 *errorQ15)
{
    const sint64 model = Test_RoundQ15(
        ((sint64)config->currentRetentionQ15 * estimatedCurrentQ15) +
        ((sint64)config->voltageInjectionQ15 * appliedVoltageQ15) -
        ((sint64)config->bemfSubtractionQ15 * estimatedBemfQ15));
    const sint64 error = (sint64)measuredCurrentQ15 - model;

    *nextCurrentQ15 = (sint32)Test_RoundQ15(
        (model * 32768LL) +
        ((sint64)config->currentErrorToCurrentQ15 * error));
    *nextBemfQ15 = (sint32)Test_RoundQ15(
        ((sint64)estimatedBemfQ15 * 32768LL) +
        ((sint64)config->currentErrorToBemfQ15 * error));
    *errorQ15 = (sint32)error;
}

STATIC uint16 Test_ReferenceAngle(sint32 alpha, sint32 beta)
{
    double angle = atan2((double)beta, (double)alpha);
    double scaled = angle * 65536.0 / (2.0 * TEST_PI);
    sint64 rounded = (scaled >= 0.0) ?
        (sint64)floor(scaled + 0.5) :
        (sint64)ceil(scaled - 0.5);

    return (uint16)((uint64)rounded & 65535ULL);
}

STATIC uint32 Test_AngleError(uint16 actual, uint16 expected)
{
    sint32 delta = MotorControlObserver_CircularDelta(actual, expected);

    return (uint32)((delta < 0) ? -(sint64)delta : (sint64)delta);
}

STATIC sint32 Test_RoundDouble(double value)
{
    return (value >= 0.0) ?
        (sint32)floor(value + 0.5) :
        (sint32)ceil(value - 0.5);
}

STATIC boolean Test_ResultsEqual(
    const MotorControlObserver_ResultType *left,
    const MotorControlObserver_ResultType *right)
{
    return ((left->estimatedCurrentAlphaQ15 ==
                right->estimatedCurrentAlphaQ15) &&
            (left->estimatedCurrentBetaQ15 ==
                right->estimatedCurrentBetaQ15) &&
            (left->estimatedBemfAlphaQ15 == right->estimatedBemfAlphaQ15) &&
            (left->estimatedBemfBetaQ15 == right->estimatedBemfBetaQ15) &&
            (left->angleQ16 == right->angleQ16) &&
            (left->speedAngleCounts == right->speedAngleCounts) &&
            (left->currentResidualMagnitudeSqQ30 ==
                right->currentResidualMagnitudeSqQ30) &&
            (left->bemfMagnitudeSqQ30 == right->bemfMagnitudeSqQ30) &&
            (left->speedVarianceNumerator ==
                right->speedVarianceNumerator) &&
            (left->consecutiveUnreliableSamples ==
                right->consecutiveUnreliableSamples) &&
            (left->numericValid == right->numericValid) &&
            (left->angleValid == right->angleValid) &&
            (left->speedValid == right->speedValid) &&
            (left->varianceValid == right->varianceValid) &&
            (left->bemfValid == right->bemfValid) &&
            (left->currentValid == right->currentValid) &&
            (left->directionValid == right->directionValid) &&
            (left->lifecycle == right->lifecycle) &&
            (left->failure == right->failure)) ? TRUE : FALSE;
}

STATIC boolean Test_AllPredicatesFalse(
    const MotorControlObserver_ResultType *result)
{
    return ((result->numericValid == FALSE) &&
            (result->angleValid == FALSE) &&
            (result->speedValid == FALSE) &&
            (result->varianceValid == FALSE) &&
            (result->bemfValid == FALSE) &&
            (result->currentValid == FALSE) &&
            (result->directionValid == FALSE)) ? TRUE : FALSE;
}

STATIC void Test_BemfVector(
    uint16 rotorAngle,
    sint32 direction,
    sint32 magnitude,
    sint32 *alpha,
    sint32 *beta)
{
    uint16 bemfAngle = MotorControlObserver_AddAngle(
        rotorAngle,
        (direction > 0) ?
            (sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN :
            -(sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
    double radians = (double)bemfAngle * (2.0 * TEST_PI) / 65536.0;

    *alpha = Test_RoundDouble((double)magnitude * cos(radians));
    *beta = Test_RoundDouble((double)magnitude * sin(radians));
}

STATIC Std_ReturnType Test_PrimeBemfForRotor(
    MotorControlObserver_ObjectType *object,
    uint16 rotorAngle,
    sint32 direction,
    sint32 magnitude,
    uint16 *rawAngle)
{
    sint32 alpha;
    sint32 beta;
    uint16 bemfAngle;

    Test_BemfVector(rotorAngle, direction, magnitude, &alpha, &beta);
    object->estimatedBemfAlphaQ15 = alpha;
    object->estimatedBemfBetaQ15 = beta;
    if (MotorControlObserver_CordicAngle(alpha, beta, &bemfAngle) != E_OK)
    {
        return E_NOT_OK;
    }
    *rawAngle = MotorControlObserver_AddAngle(
        bemfAngle,
        (direction > 0) ?
            -(sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN :
            (sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
    return E_OK;
}

STATIC uint64 Test_VarianceOracle(const sint32 *ring, uint32 length)
{
    sint64 sum = 0;
    uint64 squares = 0U;
    uint64 magnitude;
    uint32 index;

    for (index = 0U; index < length; index++)
    {
        sum += ring[index];
        squares += (uint64)((sint64)ring[index] * (sint64)ring[index]);
    }
    magnitude = (uint64)((sum < 0) ? -sum : sum);
    return ((uint64)length * squares) - (magnitude * magnitude);
}

STATIC int Test_ConfigAndLifecycle(void)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ConfigType bad;
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ObjectType before;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input = Test_Input();

    (void)memset(&object, 0x5A, sizeof(object));
    before = object;
    TEST_CHECK(MotorControlObserver_Init(NULL_PTR, &config) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlObserver_Init(&object, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

#define TEST_REJECT_CONFIG(change)                                           \
    do                                                                       \
    {                                                                        \
        bad = config;                                                        \
        change;                                                              \
        (void)memset(&object, 0x5A, sizeof(object));                          \
        TEST_CHECK(MotorControlObserver_Init(&object, &bad) == E_NOT_OK);     \
        TEST_CHECK(object.lifecycle ==                                      \
                   MOTOR_CONTROL_OBSERVER_UNINITIALIZED);                    \
        TEST_CHECK(object.failure ==                                        \
                   MOTOR_CONTROL_OBSERVER_FAILURE_CONFIGURATION);            \
    } while (0)

    TEST_REJECT_CONFIG(bad.currentRetentionQ15 = -1);
    TEST_REJECT_CONFIG(bad.voltageInjectionQ15 = -1);
    TEST_REJECT_CONFIG(bad.bemfSubtractionQ15 = -1);
    TEST_REJECT_CONFIG(bad.currentErrorToCurrentQ15 = -1);
    TEST_REJECT_CONFIG(bad.phaseProportionalQ15 = -1);
    TEST_REJECT_CONFIG(bad.phaseIntegralQ15 = -1);
    TEST_REJECT_CONFIG(bad.currentErrorToBemfQ15 = 32768);
    TEST_REJECT_CONFIG(bad.maxAbsCurrentQ15 = 0);
    TEST_REJECT_CONFIG(bad.maxAbsCurrentQ15 = 32768);
    TEST_REJECT_CONFIG(bad.maxAbsBemfQ15 = 0);
    TEST_REJECT_CONFIG(bad.maxAbsSpeedAngleCounts = 0U);
    TEST_REJECT_CONFIG(bad.maxAbsSpeedAngleCounts = 32768U);
    TEST_REJECT_CONFIG(bad.maxSpeedDeltaAngleCounts = 0U);
    TEST_REJECT_CONFIG(bad.maxSpeedDeltaAngleCounts = 4097U);
    TEST_REJECT_CONFIG(bad.minBemfMagnitudeSqQ30 = 0U);
    TEST_REJECT_CONFIG(bad.minBemfMagnitudeSqQ30 =
                       bad.maxBemfMagnitudeSqQ30 + 1U);
    TEST_REJECT_CONFIG(bad.maxBemfMagnitudeSqQ30 = 2147352579ULL);
    TEST_REJECT_CONFIG(bad.maxCurrentResidualMagnitudeSqQ30 = 0U);
    TEST_REJECT_CONFIG(bad.maxCurrentResidualMagnitudeSqQ30 = 8589672451ULL);
    TEST_REJECT_CONFIG(bad.speedWindowLength = 1U);
    TEST_REJECT_CONFIG(bad.speedWindowLength = 3U);
    TEST_REJECT_CONFIG(bad.speedWindowLength = 64U);
    TEST_REJECT_CONFIG(bad.reliabilityWarmupSamples = 3U);
    TEST_REJECT_CONFIG(bad.maxSpeedVarianceNumerator = 0U);
    TEST_REJECT_CONFIG(bad.maxSpeedVarianceNumerator = 67108865ULL);
    TEST_REJECT_CONFIG(bad.maxConsecutiveUnreliableSamples = 0U);
#undef TEST_REJECT_CONFIG

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);
    TEST_CHECK(object.failure == MOTOR_CONTROL_OBSERVER_FAILURE_NONE);
    before = object;
    TEST_CHECK(MotorControlObserver_Begin(&object, 123U, 0) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlObserver_Begin(&object, 123U, 1) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_ACTIVE);
    TEST_CHECK(object.angleQ16 == 123U);
    before = object;
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_Step(NULL_PTR, &input, &result) == E_NOT_OK);
    TEST_CHECK(result.numericValid == FALSE);
    before = object;
    TEST_CHECK(MotorControlObserver_Step(&object, NULL_PTR, &result) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);
    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);
    TEST_CHECK(result.numericValid == FALSE);
    TEST_CHECK(MotorControlObserver_ResetFromReady(NULL_PTR) == E_NOT_OK);

    return 0;
}

STATIC int Test_LifecycleTransitionMatrix(void)
{
    static const sint32 invalidDirections[] = { 0, 2, -2, 32767, -32768 };
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ObjectType before;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input = Test_Input();
    uint32 index;

    TEST_CHECK(MotorControlObserver_Begin(NULL_PTR, 0U, 1) == E_NOT_OK);

    (void)memset(&object, 0, sizeof(object));
    before = object;
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    for (index = 0U;
         index < (uint32)(sizeof(invalidDirections) /
                          sizeof(invalidDirections[0]));
         index++)
    {
        before = object;
        TEST_CHECK(MotorControlObserver_Begin(
                       &object, 0U, invalidDirections[index]) == E_NOT_OK);
        TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    }

    TEST_CHECK(MotorControlObserver_Begin(&object, 123U, 1) == E_OK);
    before = object;
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, -1) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    object.lifecycle = MOTOR_CONTROL_OBSERVER_FAILED;
    object.failure = MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE;
    before = object;
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    object.lifecycle = (MotorControlObserver_LifecycleType)99;
    before = object;
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    before = object;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(result.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);

    (void)memset(&object, 0, sizeof(object));
    before = object;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(result.lifecycle == MOTOR_CONTROL_OBSERVER_UNINITIALIZED);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    object.lifecycle = (MotorControlObserver_LifecycleType)99;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_FAILED);
    TEST_CHECK(object.failure == MOTOR_CONTROL_OBSERVER_FAILURE_INTERNAL);
    before = object;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(result.failure == MOTOR_CONTROL_OBSERVER_FAILURE_INTERNAL);

    (void)memset(&object, 0, sizeof(object));
    before = object;
    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    object.lifecycle = (MotorControlObserver_LifecycleType)99;
    before = object;
    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    object.lifecycle = MOTOR_CONTROL_OBSERVER_FAILED;
    object.failure = MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC;
    before = object;
    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, -1) == E_OK);
    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);
    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);

    object.lifecycle = MOTOR_CONTROL_OBSERVER_FAILED;
    object.failure = MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);
    TEST_CHECK(object.failure == MOTOR_CONTROL_OBSERVER_FAILURE_NONE);
    return 0;
}

STATIC int Test_ConfigInclusiveBoundaries(void)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;

    config.currentRetentionQ15 = 0;
    config.voltageInjectionQ15 = 0;
    config.bemfSubtractionQ15 = 0;
    config.currentErrorToCurrentQ15 = 0;
    config.currentErrorToBemfQ15 = -32768;
    config.phaseProportionalQ15 = 0;
    config.phaseIntegralQ15 = 0;
    config.maxAbsCurrentQ15 = 1;
    config.maxAbsBemfQ15 = 1;
    config.maxAbsSpeedAngleCounts = 1U;
    config.maxSpeedDeltaAngleCounts = 1U;
    config.minBemfMagnitudeSqQ30 = 1U;
    config.maxBemfMagnitudeSqQ30 = 2U;
    config.maxCurrentResidualMagnitudeSqQ30 = 1U;
    config.speedWindowLength = 2U;
    config.maxSpeedVarianceNumerator = 1U;
    config.reliabilityWarmupSamples = 2U;
    config.maxConsecutiveUnreliableSamples = 1U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);

    config.currentRetentionQ15 = 32767;
    config.voltageInjectionQ15 = 32767;
    config.bemfSubtractionQ15 = 32767;
    config.currentErrorToCurrentQ15 = 32767;
    config.currentErrorToBemfQ15 = 32767;
    config.phaseProportionalQ15 = 32767;
    config.phaseIntegralQ15 = 32767;
    config.maxAbsCurrentQ15 = 32767;
    config.maxAbsBemfQ15 = 32767;
    config.maxAbsSpeedAngleCounts = 32767U;
    config.maxSpeedDeltaAngleCounts = 65534U;
    config.minBemfMagnitudeSqQ30 = 2147352578ULL;
    config.maxBemfMagnitudeSqQ30 = 2147352578ULL;
    config.maxCurrentResidualMagnitudeSqQ30 = 8589672450ULL;
    config.speedWindowLength = 32U;
    config.maxSpeedVarianceNumerator = 1099444519936ULL;
    config.reliabilityWarmupSamples = UINT32_MAX;
    config.maxConsecutiveUnreliableSamples = UINT32_MAX;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);

    return 0;
}

STATIC int Test_CordicAndCircularMath(void)
{
    uint16 angle;
    uint16 inverseAngle;
    sint32 alpha;
    sint32 beta;
    uint32 xIndex;
    uint32 yIndex;

    TEST_CHECK(MotorControlObserver_CordicAngle(0, 0, &angle) == E_NOT_OK);
    TEST_CHECK(angle == 0U);
    TEST_CHECK(MotorControlObserver_CordicAngle(1, 0, &angle) == E_OK);
    TEST_CHECK(angle == 0x0000U);
    TEST_CHECK(MotorControlObserver_CordicAngle(0, 1, &angle) == E_OK);
    TEST_CHECK(angle == 0x4000U);
    TEST_CHECK(MotorControlObserver_CordicAngle(-1, 0, &angle) == E_OK);
    TEST_CHECK(angle == 0x8000U);
    TEST_CHECK(MotorControlObserver_CordicAngle(0, -1, &angle) == E_OK);
    TEST_CHECK(angle == 0xC000U);
    TEST_CHECK(MotorControlObserver_CordicAngle(1, 1, NULL_PTR) == E_NOT_OK);

    for (alpha = -64; alpha <= 64; alpha++)
    {
        for (beta = -64; beta <= 64; beta++)
        {
            if ((alpha != 0) || (beta != 0))
            {
                TEST_CHECK(MotorControlObserver_CordicAngle(
                               alpha, beta, &angle) == E_OK);
                TEST_CHECK(Test_AngleError(
                               angle,
                               Test_ReferenceAngle(alpha, beta)) <= 4U);
            }
        }
    }

    for (xIndex = 0U; xIndex < 256U; xIndex++)
    {
        alpha = -32768 + (sint32)(xIndex * 257U);
        for (yIndex = 0U; yIndex < 256U; yIndex++)
        {
            beta = -32768 + (sint32)(yIndex * 257U);
            TEST_CHECK(MotorControlObserver_CordicAngle(
                           alpha, beta, &angle) == E_OK);
            TEST_CHECK(Test_AngleError(
                           angle,
                           Test_ReferenceAngle(alpha, beta)) <= 4U);
            if ((alpha != -32768) && (beta != -32768))
            {
                TEST_CHECK(MotorControlObserver_CordicAngle(
                               -alpha, -beta, &inverseAngle) == E_OK);
                TEST_CHECK(MotorControlObserver_CircularDelta(
                               inverseAngle, angle) == -32768);
            }
        }
    }

    TEST_CHECK(MotorControlObserver_CordicAngle(3, 7, &angle) == E_OK);
    TEST_CHECK(MotorControlObserver_CordicAngle(
                   3000, 7000, &inverseAngle) == E_OK);
    TEST_CHECK(Test_AngleError(angle, inverseAngle) <= 8U);

    TEST_CHECK(MotorControlObserver_CircularDelta(0x0000U, 0x8000U) ==
               -32768);
    TEST_CHECK(MotorControlObserver_CircularDelta(0x0001U, 0xFFFFU) == 2);
    TEST_CHECK(MotorControlObserver_CircularDelta(0xFFFFU, 0x0001U) == -2);
    TEST_CHECK(MotorControlObserver_AddAngle(0xFFFFU, 2) == 1U);
    TEST_CHECK(MotorControlObserver_AddAngle(0x0000U, -1) == 0xFFFFU);
    TEST_CHECK(MotorControlObserver_RoundQ15(16384) == 1);
    TEST_CHECK(MotorControlObserver_RoundQ15(-16384) == -1);
    TEST_CHECK(MotorControlObserver_RoundQ15(49152) == 2);
    TEST_CHECK(MotorControlObserver_RoundQ15(-49152) == -2);

    return 0;
}

STATIC int Test_ObserverEquationBothAxesAndTies(void)
{
    typedef struct
    {
        sint32 retention;
        sint32 voltageGain;
        sint32 bemfGain;
        sint32 currentCorrection;
        sint32 bemfCorrection;
        sint32 measuredAlpha;
        sint32 measuredBeta;
        sint32 voltageAlpha;
        sint32 voltageBeta;
        sint32 currentAlpha;
        sint32 currentBeta;
        sint32 bemfAlpha;
        sint32 bemfBeta;
    } Test_EquationVectorType;
    static const Test_EquationVectorType vectors[] =
    {
        { 16384, 0, 0, 16384, 16384,
          2, -2, 0, 0, 1, -1, 0, 0 },
        { 0, 16384, 0, 16384, -16384,
          2, -2, 1, -1, 0, 0, 0, 0 },
        { 0, 0, 16384, 16384, 16384,
          2, -2, 0, 0, 0, 0, -1, 1 },
        { 16384, 0, 0, 0, -16384,
          2, -2, 0, 0, 1, -1, 0, 0 }
    };
    MotorControlObserver_ConfigType config;
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input;
    MotorControlObserver_FailureType failure;
    sint32 expectedCurrentAlpha;
    sint32 expectedCurrentBeta;
    sint32 expectedBemfAlpha;
    sint32 expectedBemfBeta;
    sint32 expectedErrorAlpha;
    sint32 expectedErrorBeta;
    sint32 nextCurrent;
    sint32 nextBemf;
    sint32 error;
    uint32 index;

    for (index = 0U;
         index < (uint32)(sizeof(vectors) / sizeof(vectors[0]));
         index++)
    {
        config = Test_Config();
        config.currentRetentionQ15 = vectors[index].retention;
        config.voltageInjectionQ15 = vectors[index].voltageGain;
        config.bemfSubtractionQ15 = vectors[index].bemfGain;
        config.currentErrorToCurrentQ15 = vectors[index].currentCorrection;
        config.currentErrorToBemfQ15 = vectors[index].bemfCorrection;
        TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
        TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);

        object.estimatedCurrentAlphaQ15 = vectors[index].currentAlpha;
        object.estimatedCurrentBetaQ15 = vectors[index].currentBeta;
        object.estimatedBemfAlphaQ15 = vectors[index].bemfAlpha;
        object.estimatedBemfBetaQ15 = vectors[index].bemfBeta;
        input = Test_Input();
        input.measuredCurrentAlphaQ15 = vectors[index].measuredAlpha;
        input.measuredCurrentBetaQ15 = vectors[index].measuredBeta;
        input.appliedVoltageAlphaQ15 = vectors[index].voltageAlpha;
        input.appliedVoltageBetaQ15 = vectors[index].voltageBeta;

        Test_ReferenceAxis(
            input.measuredCurrentAlphaQ15,
            input.appliedVoltageAlphaQ15,
            object.estimatedCurrentAlphaQ15,
            object.estimatedBemfAlphaQ15,
            &config,
            &expectedCurrentAlpha,
            &expectedBemfAlpha,
            &expectedErrorAlpha);
        Test_ReferenceAxis(
            input.measuredCurrentBetaQ15,
            input.appliedVoltageBetaQ15,
            object.estimatedCurrentBetaQ15,
            object.estimatedBemfBetaQ15,
            &config,
            &expectedCurrentBeta,
            &expectedBemfBeta,
            &expectedErrorBeta);

        TEST_CHECK(MotorControlObserver_Step(
                       &object, &input, &result) == E_OK);
        TEST_CHECK(result.estimatedCurrentAlphaQ15 == expectedCurrentAlpha);
        TEST_CHECK(result.estimatedCurrentBetaQ15 == expectedCurrentBeta);
        TEST_CHECK(result.estimatedBemfAlphaQ15 == expectedBemfAlpha);
        TEST_CHECK(result.estimatedBemfBetaQ15 == expectedBemfBeta);
        TEST_CHECK(result.currentResidualMagnitudeSqQ30 ==
                   MotorControlObserver_Square(expectedErrorAlpha) +
                   MotorControlObserver_Square(expectedErrorBeta));
    }

    config = Test_Config();
    config.currentRetentionQ15 = 32767;
    config.voltageInjectionQ15 = 32767;
    config.bemfSubtractionQ15 = 32767;
    config.currentErrorToCurrentQ15 = 32767;
    config.currentErrorToBemfQ15 = -32768;
    failure = MotorControlObserver_ComputeAxis(
        -32768, 32767, 32767, -32767, &config,
        &nextCurrent, &nextBemf, &error);
    TEST_CHECK(failure == MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE);
    return 0;
}

STATIC int Test_TrackerBoundsAndNoWindup(void)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input = Test_Input();
    uint16 rawAngle;

    config.currentErrorToBemfQ15 = 0;
    config.maxAbsSpeedAngleCounts = 10U;
    config.maxSpeedDeltaAngleCounts = 20U;
    config.maxSpeedVarianceNumerator = 1600U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    TEST_CHECK(Test_PrimeBemfForRotor(
                   &object, 1000U, 1, 10000, &rawAngle) == E_OK);

    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, -10);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.angleValid == TRUE);
    TEST_CHECK(result.speedValid == TRUE);
    TEST_CHECK(object.speedAngleCounts == 10);
    TEST_CHECK(object.speedIntegratorQ15 == 327670);

    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    TEST_CHECK(Test_PrimeBemfForRotor(
                   &object, 1000U, 1, 10000, &rawAngle) == E_OK);
    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, -11);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.angleValid == FALSE);
    TEST_CHECK(result.speedValid == FALSE);
    TEST_CHECK(object.speedAngleCounts == 0);
    TEST_CHECK(object.speedIntegratorQ15 == 0);
    TEST_CHECK(object.speedWindowFill == 0U);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.speedIntegratorQ15 == 0);
    TEST_CHECK(object.angleQ16 == MotorControlObserver_AddAngle(rawAngle, -11));

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    config.maxAbsSpeedAngleCounts = 20U;
    config.maxSpeedDeltaAngleCounts = 10U;
    config.maxSpeedVarianceNumerator = 6400U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    TEST_CHECK(Test_PrimeBemfForRotor(
                   &object, 2000U, 1, 10000, &rawAngle) == E_OK);
    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, -10);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.speedAngleCounts == 10);
    TEST_CHECK(object.speedIntegratorQ15 == 327670);

    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    TEST_CHECK(Test_PrimeBemfForRotor(
                   &object, 2000U, 1, 10000, &rawAngle) == E_OK);
    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, -11);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.speedValid == FALSE);
    TEST_CHECK(object.speedAngleCounts == 0);
    TEST_CHECK(object.speedIntegratorQ15 == 0);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.speedIntegratorQ15 == 0);

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    config.phaseProportionalQ15 = 32767;
    config.maxAbsSpeedAngleCounts = 20U;
    config.maxSpeedDeltaAngleCounts = 20U;
    config.maxSpeedVarianceNumerator = 6400U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 65530U, 1) == E_OK);
    TEST_CHECK(Test_PrimeBemfForRotor(
                   &object, 2U, 1, 10000, &rawAngle) == E_OK);
    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, -8);
    TEST_CHECK(object.angleQ16 > rawAngle);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.speedAngleCounts == 8);
    TEST_CHECK(result.angleQ16 == rawAngle);

    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 2U, -1) == E_OK);
    TEST_CHECK(Test_PrimeBemfForRotor(
                   &object, 65530U, -1, 10000, &rawAngle) == E_OK);
    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, 8);
    TEST_CHECK(object.angleQ16 < rawAngle);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.speedAngleCounts == -8);
    TEST_CHECK(result.angleQ16 == rawAngle);

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    config.phaseProportionalQ15 = 0;
    config.phaseIntegralQ15 = 1;
    config.maxAbsSpeedAngleCounts = 1U;
    config.maxSpeedDeltaAngleCounts = 2U;
    config.maxSpeedVarianceNumerator = 16U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, -1) == E_OK);
    TEST_CHECK(Test_PrimeBemfForRotor(
                   &object, 32768U, -1, 10000, &rawAngle) == E_OK);
    TEST_CHECK(rawAngle == 32768U);
    object.angleQ16 = 0U;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.speedAngleCounts == -1);
    TEST_CHECK(object.speedIntegratorQ15 == -32768LL);

    return 0;
}

STATIC int Test_ReliabilityAndRollingWindow(sint32 direction)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input = Test_Input();
    uint16 requestedRotorAngle;
    sint32 desiredAlpha;
    sint32 desiredBeta;
    uint32 sample;
    uint64 expectedVariance;
    uint64 sumMagnitude;

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, direction) == E_OK);
    requestedRotorAngle = (direction > 0) ? 64U : (uint16)65472U;

    for (sample = 0U; sample < 10U; sample++)
    {
        Test_BemfVector(
            requestedRotorAngle,
            direction,
            8000,
            &desiredAlpha,
            &desiredBeta);
        input.measuredCurrentAlphaQ15 =
            desiredAlpha - object.estimatedBemfAlphaQ15;
        input.measuredCurrentBetaQ15 =
            desiredBeta - object.estimatedBemfBetaQ15;
        TEST_CHECK(MotorControlObserver_IsQ15(
                       input.measuredCurrentAlphaQ15) == TRUE);
        TEST_CHECK(MotorControlObserver_IsQ15(
                       input.measuredCurrentBetaQ15) == TRUE);
        TEST_CHECK(MotorControlObserver_Step(
                       &object, &input, &result) == E_OK);
        TEST_CHECK(result.numericValid == TRUE);
        TEST_CHECK(result.bemfValid == TRUE);
        TEST_CHECK(result.currentValid == TRUE);
        TEST_CHECK(result.angleValid == TRUE);
        TEST_CHECK(result.speedValid == TRUE);
        TEST_CHECK(result.directionValid == TRUE);

        if (object.speedWindowFill == config.speedWindowLength)
        {
            sumMagnitude = (uint64)((object.speedSum < 0) ?
                -object.speedSum : object.speedSum);
            expectedVariance =
                (uint64)config.speedWindowLength * object.speedSumSquares -
                sumMagnitude * sumMagnitude;
            TEST_CHECK(result.speedVarianceNumerator == expectedVariance);
        }

        requestedRotorAngle = MotorControlObserver_AddAngle(
            object.angleQ16, object.speedAngleCounts);
    }

    TEST_CHECK(result.varianceValid == TRUE);
    TEST_CHECK(result.consecutiveUnreliableSamples == 0U);
    TEST_CHECK(object.speedWindowFill == config.speedWindowLength);
    TEST_CHECK(object.speedWindowIndex < config.speedWindowLength);

    input.measuredCurrentAlphaQ15 = -object.estimatedBemfAlphaQ15;
    input.measuredCurrentBetaQ15 = -object.estimatedBemfBetaQ15;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.angleValid == FALSE);
    TEST_CHECK(result.speedValid == FALSE);
    TEST_CHECK(result.bemfValid == FALSE);
    TEST_CHECK(object.speedWindowFill == 0U);
    TEST_CHECK(object.speedWindowIndex == 0U);
    TEST_CHECK(object.warmupSamples == 0U);

    for (sample = 0U; sample < config.speedWindowLength; sample++)
    {
        requestedRotorAngle = MotorControlObserver_AddAngle(
            object.angleQ16, object.speedAngleCounts);
        Test_BemfVector(
            requestedRotorAngle,
            direction,
            8000,
            &desiredAlpha,
            &desiredBeta);
        input.measuredCurrentAlphaQ15 =
            desiredAlpha - object.estimatedBemfAlphaQ15;
        input.measuredCurrentBetaQ15 =
            desiredBeta - object.estimatedBemfBetaQ15;
        TEST_CHECK(MotorControlObserver_Step(
                       &object, &input, &result) == E_OK);
    }
    TEST_CHECK(object.speedWindowFill == config.speedWindowLength);
    TEST_CHECK(result.varianceValid == TRUE);
    TEST_CHECK(result.consecutiveUnreliableSamples == 0U);

    return 0;
}

STATIC int Test_RollingWindowIndependentOracle(void)
{
    static const sint32 firstSequence[] = { 1, -2, 3, -4, 5, -6 };
    static const sint32 refillSequence[] = { 7, 8, 9, 10 };
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input = Test_Input();
    sint32 oracleRing[4] = { 0, 0, 0, 0 };
    sint64 oracleSum = 0;
    uint64 oracleSquares = 0U;
    uint32 oracleFill = 0U;
    uint32 oracleIndex = 0U;
    uint16 rawAngle;
    uint32 sample;
    uint32 index;

    config.currentErrorToBemfQ15 = 0;
    config.phaseProportionalQ15 = 0;
    config.phaseIntegralQ15 = 0;
    config.maxAbsSpeedAngleCounts = 16U;
    config.maxSpeedDeltaAngleCounts = 32U;
    config.maxSpeedVarianceNumerator = 4096U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);

    for (sample = 0U;
         sample < (uint32)(sizeof(firstSequence) / sizeof(firstSequence[0]));
         sample++)
    {
        TEST_CHECK(Test_PrimeBemfForRotor(
                       &object,
                       (uint16)(1000U + (sample * 100U)),
                       1,
                       10000,
                       &rawAngle) == E_OK);
        object.speedAngleCounts = firstSequence[sample];
        object.speedIntegratorQ15 =
            (sint64)firstSequence[sample] * 32768LL;
        object.angleQ16 = MotorControlObserver_AddAngle(
            rawAngle, -firstSequence[sample]);
        TEST_CHECK(MotorControlObserver_Step(
                       &object, &input, &result) == E_OK);

        if (oracleFill == 4U)
        {
            oracleSum -= oracleRing[oracleIndex];
            oracleSquares -= (uint64)(
                (sint64)oracleRing[oracleIndex] * oracleRing[oracleIndex]);
        }
        else
        {
            oracleFill++;
        }
        oracleRing[oracleIndex] = firstSequence[sample];
        oracleSum += firstSequence[sample];
        oracleSquares += (uint64)(
            (sint64)firstSequence[sample] * firstSequence[sample]);
        oracleIndex = (oracleIndex + 1U) & 3U;

        TEST_CHECK(object.speedWindowFill == oracleFill);
        TEST_CHECK(object.speedWindowIndex == oracleIndex);
        TEST_CHECK(object.speedSum == oracleSum);
        TEST_CHECK(object.speedSumSquares == oracleSquares);
        for (index = 0U; index < 4U; index++)
        {
            TEST_CHECK(object.speedRing[index] == oracleRing[index]);
        }
        if (oracleFill == 4U)
        {
            TEST_CHECK(result.speedVarianceNumerator ==
                       Test_VarianceOracle(oracleRing, 4U));
        }
    }

    TEST_CHECK(oracleRing[0] == 5);
    TEST_CHECK(oracleRing[1] == -6);
    TEST_CHECK(oracleRing[2] == 3);
    TEST_CHECK(oracleRing[3] == -4);
    TEST_CHECK(oracleIndex == 2U);

    object.estimatedBemfAlphaQ15 = 0;
    object.estimatedBemfBetaQ15 = 0;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.speedWindowFill == 0U);
    TEST_CHECK(object.speedWindowIndex == 0U);
    TEST_CHECK(object.speedSum == 0);
    TEST_CHECK(object.speedSumSquares == 0U);
    for (index = 0U; index < 4U; index++)
    {
        TEST_CHECK(object.speedRing[index] == 0);
        oracleRing[index] = 0;
    }

    for (sample = 0U;
         sample < (uint32)(sizeof(refillSequence) /
                           sizeof(refillSequence[0]));
         sample++)
    {
        TEST_CHECK(Test_PrimeBemfForRotor(
                       &object,
                       (uint16)(3000U + (sample * 100U)),
                       1,
                       10000,
                       &rawAngle) == E_OK);
        object.speedAngleCounts = refillSequence[sample];
        object.speedIntegratorQ15 =
            (sint64)refillSequence[sample] * 32768LL;
        object.angleQ16 = MotorControlObserver_AddAngle(
            rawAngle, -refillSequence[sample]);
        TEST_CHECK(MotorControlObserver_Step(
                       &object, &input, &result) == E_OK);
        oracleRing[sample] = refillSequence[sample];
    }
    TEST_CHECK(object.speedWindowFill == 4U);
    TEST_CHECK(object.speedWindowIndex == 0U);
    TEST_CHECK(result.speedVarianceNumerator ==
               Test_VarianceOracle(oracleRing, 4U));
    for (index = 0U; index < 4U; index++)
    {
        TEST_CHECK(object.speedRing[index] == oracleRing[index]);
    }
    return 0;
}

STATIC int Test_ExactReliabilityThresholds(void)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input = Test_Input();
    sint32 alpha;
    sint32 beta;
    uint16 bemfAngle;
    uint16 rawAngle;

    config.currentErrorToBemfQ15 = 0;
    config.minBemfMagnitudeSqQ30 = 25U;
    config.maxBemfMagnitudeSqQ30 = 25U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.estimatedBemfAlphaQ15 = 3;
    object.estimatedBemfBetaQ15 = 4;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.bemfMagnitudeSqQ30 == 25U);
    TEST_CHECK(result.bemfValid == TRUE);

    config.minBemfMagnitudeSqQ30 = 1U;
    config.maxBemfMagnitudeSqQ30 = 24U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.estimatedBemfAlphaQ15 = 3;
    object.estimatedBemfBetaQ15 = 4;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.bemfMagnitudeSqQ30 == 25U);
    TEST_CHECK(result.bemfValid == FALSE);

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    config.maxCurrentResidualMagnitudeSqQ30 = 25U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.estimatedBemfBetaQ15 = 1;
    input.measuredCurrentAlphaQ15 = 3;
    input.measuredCurrentBetaQ15 = 4;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.currentResidualMagnitudeSqQ30 == 25U);
    TEST_CHECK(result.currentValid == TRUE);

    config.maxCurrentResidualMagnitudeSqQ30 = 24U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.estimatedBemfBetaQ15 = 1;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.currentResidualMagnitudeSqQ30 == 25U);
    TEST_CHECK(result.currentValid == FALSE);

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    config.maxAbsSpeedAngleCounts = 10U;
    config.maxSpeedDeltaAngleCounts = 10U;
    config.speedWindowLength = 2U;
    config.reliabilityWarmupSamples = 2U;
    config.maxSpeedVarianceNumerator = 4U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    Test_BemfVector(1000U, 1, 10000, &alpha, &beta);
    object.estimatedBemfAlphaQ15 = alpha;
    object.estimatedBemfBetaQ15 = beta;
    TEST_CHECK(MotorControlObserver_CordicAngle(
                   alpha, beta, &bemfAngle) == E_OK);
    rawAngle = MotorControlObserver_AddAngle(
        bemfAngle,
        -(sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
    object.speedRing[0] = 1;
    object.speedRing[1] = 3;
    object.speedSum = 4;
    object.speedSumSquares = 10U;
    object.speedWindowFill = 2U;
    object.speedWindowIndex = 0U;
    object.warmupSamples = 2U;
    object.speedAngleCounts = 3;
    object.speedIntegratorQ15 = 3LL * 32768LL;
    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, -5);
    input = Test_Input();
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.speedVarianceNumerator == 4U);
    TEST_CHECK(result.varianceValid == TRUE);

    config.maxSpeedVarianceNumerator = 3U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.estimatedBemfAlphaQ15 = alpha;
    object.estimatedBemfBetaQ15 = beta;
    object.speedRing[0] = 1;
    object.speedRing[1] = 3;
    object.speedSum = 4;
    object.speedSumSquares = 10U;
    object.speedWindowFill = 2U;
    object.speedWindowIndex = 0U;
    object.warmupSamples = 2U;
    object.speedAngleCounts = 3;
    object.speedIntegratorQ15 = 3LL * 32768LL;
    object.angleQ16 = MotorControlObserver_AddAngle(rawAngle, -5);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.speedVarianceNumerator == 4U);
    TEST_CHECK(result.varianceValid == FALSE);

    return 0;
}

STATIC int Test_PublicInvalidMatrix(void)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input;
    uint32 field;

    for (field = 0U; field < 7U; field++)
    {
        input = Test_Input();
        switch (field)
        {
            case 0U:
                input.measuredCurrentAlphaQ15 = 32768;
                break;
            case 1U:
                input.measuredCurrentBetaQ15 = -32769;
                break;
            case 2U:
                input.appliedVoltageAlphaQ15 = 32768;
                break;
            case 3U:
                input.appliedVoltageBetaQ15 = -32769;
                break;
            case 4U:
                input.currentSampleValid = (boolean)2U;
                break;
            case 5U:
                input.voltageSampleValid = (boolean)2U;
                break;
            default:
                input.generationValid = (boolean)2U;
                break;
        }
        TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
        TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
        TEST_CHECK(MotorControlObserver_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK(object.failure ==
                   MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_REPRESENTATION);
        TEST_CHECK(Test_AllPredicatesFalse(&result) == TRUE);
    }

    for (field = 0U; field < 3U; field++)
    {
        input = Test_Input();
        if (field == 0U)
        {
            input.currentSampleValid = FALSE;
        }
        else if (field == 1U)
        {
            input.voltageSampleValid = FALSE;
        }
        else
        {
            input.generationValid = FALSE;
        }
        TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
        TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
        TEST_CHECK(MotorControlObserver_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK(object.failure ==
                   MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_QUALITY);
        TEST_CHECK(Test_AllPredicatesFalse(&result) == TRUE);
    }

    return 0;
}

STATIC int Test_IndependentPredicatesAndFailures(void)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType object;
    MotorControlObserver_ObjectType beforeFailure;
    MotorControlObserver_ResultType result;
    MotorControlObserver_StepInputType input = Test_Input();
    sint32 alpha;
    sint32 beta;

    config.currentErrorToBemfQ15 = 0;
    config.minBemfMagnitudeSqQ30 = 1000000U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.estimatedBemfBetaQ15 = 100;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.bemfValid == FALSE);
    TEST_CHECK(result.numericValid == TRUE);
    TEST_CHECK(result.varianceValid == FALSE);

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    config.maxCurrentResidualMagnitudeSqQ30 = 1U;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.estimatedBemfBetaQ15 = 1000;
    input.measuredCurrentAlphaQ15 = 2;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.currentValid == FALSE);
    TEST_CHECK(result.bemfValid == TRUE);

    config = Test_Config();
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    input = Test_Input();
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.angleValid == FALSE);
    TEST_CHECK(result.speedValid == FALSE);
    TEST_CHECK(result.directionValid == FALSE);
    TEST_CHECK(result.consecutiveUnreliableSamples == 1U);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.consecutiveUnreliableSamples == 3U);

    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    input.currentSampleValid = (boolean)2U;
    input.generationValid = FALSE;
    beforeFailure = object;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_OBSERVER_FAILED);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_REPRESENTATION);
    TEST_CHECK(result.numericValid == FALSE);
    TEST_CHECK(result.bemfValid == FALSE);
    TEST_CHECK(object.estimatedBemfAlphaQ15 ==
               beforeFailure.estimatedBemfAlphaQ15);
    TEST_CHECK(MotorControlObserver_ResetFromReady(&object) == E_NOT_OK);

    input = Test_Input();
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_REPRESENTATION);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    input.generationValid = FALSE;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_QUALITY);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    input = Test_Input();
    input.measuredCurrentAlphaQ15 = 32768;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_REPRESENTATION);

    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    object.speedSum = 1;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_OBSERVER_FAILURE_INTERNAL);

    config = Test_Config();
    config.voltageInjectionQ15 = 32767;
    config.maxAbsCurrentQ15 = 100;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, 1) == E_OK);
    input = Test_Input();
    input.appliedVoltageBetaQ15 = 32767;
    beforeFailure = object;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE);
    TEST_CHECK(object.estimatedCurrentAlphaQ15 ==
               beforeFailure.estimatedCurrentAlphaQ15);
    TEST_CHECK(object.estimatedCurrentBetaQ15 ==
               beforeFailure.estimatedCurrentBetaQ15);

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    TEST_CHECK(MotorControlObserver_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&object, 0U, -1) == E_OK);
    Test_BemfVector(65000U, -1, 10000, &alpha, &beta);
    object.estimatedBemfAlphaQ15 = alpha;
    object.estimatedBemfBetaQ15 = beta;
    beforeFailure = object;
    input = Test_Input();
    input.voltageSampleValid = FALSE;
    TEST_CHECK(MotorControlObserver_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_QUALITY);
    TEST_CHECK(object.estimatedBemfAlphaQ15 ==
               beforeFailure.estimatedBemfAlphaQ15);

    return 0;
}

STATIC int Test_MetamorphicSequences(void)
{
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType first;
    MotorControlObserver_ObjectType second;
    MotorControlObserver_ResultType firstResult;
    MotorControlObserver_ResultType secondResult;
    MotorControlObserver_StepInputType firstInput = Test_Input();
    MotorControlObserver_StepInputType secondInput = Test_Input();
    uint16 bemfAngle;
    uint16 positiveRaw;
    uint16 negativeRaw;

    config.currentRetentionQ15 = 16384;
    config.voltageInjectionQ15 = 8192;
    config.bemfSubtractionQ15 = 4096;
    config.currentErrorToCurrentQ15 = 8192;
    config.currentErrorToBemfQ15 = -4096;
    config.phaseProportionalQ15 = 0;
    config.phaseIntegralQ15 = 0;

    TEST_CHECK(MotorControlObserver_Init(&first, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Init(&second, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&first, 1000U, 1) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&second, 1000U, 1) == E_OK);
    first.estimatedCurrentAlphaQ15 = 100;
    first.estimatedCurrentBetaQ15 = -200;
    first.estimatedBemfAlphaQ15 = 300;
    first.estimatedBemfBetaQ15 = 400;
    second.estimatedCurrentAlphaQ15 = first.estimatedCurrentBetaQ15;
    second.estimatedCurrentBetaQ15 = first.estimatedCurrentAlphaQ15;
    second.estimatedBemfAlphaQ15 = first.estimatedBemfBetaQ15;
    second.estimatedBemfBetaQ15 = first.estimatedBemfAlphaQ15;
    firstInput.measuredCurrentAlphaQ15 = 500;
    firstInput.measuredCurrentBetaQ15 = -600;
    firstInput.appliedVoltageAlphaQ15 = 700;
    firstInput.appliedVoltageBetaQ15 = -800;
    secondInput.measuredCurrentAlphaQ15 = firstInput.measuredCurrentBetaQ15;
    secondInput.measuredCurrentBetaQ15 = firstInput.measuredCurrentAlphaQ15;
    secondInput.appliedVoltageAlphaQ15 = firstInput.appliedVoltageBetaQ15;
    secondInput.appliedVoltageBetaQ15 = firstInput.appliedVoltageAlphaQ15;
    TEST_CHECK(MotorControlObserver_Step(
                   &first, &firstInput, &firstResult) == E_OK);
    TEST_CHECK(MotorControlObserver_Step(
                   &second, &secondInput, &secondResult) == E_OK);
    TEST_CHECK(firstResult.estimatedCurrentAlphaQ15 ==
               secondResult.estimatedCurrentBetaQ15);
    TEST_CHECK(firstResult.estimatedCurrentBetaQ15 ==
               secondResult.estimatedCurrentAlphaQ15);
    TEST_CHECK(firstResult.estimatedBemfAlphaQ15 ==
               secondResult.estimatedBemfBetaQ15);
    TEST_CHECK(firstResult.estimatedBemfBetaQ15 ==
               secondResult.estimatedBemfAlphaQ15);
    TEST_CHECK(firstResult.currentResidualMagnitudeSqQ30 ==
               secondResult.currentResidualMagnitudeSqQ30);
    TEST_CHECK(firstResult.bemfMagnitudeSqQ30 ==
               secondResult.bemfMagnitudeSqQ30);

    TEST_CHECK(MotorControlObserver_Init(&first, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Init(&second, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&first, 1234U, 1) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(
                   &second,
                   MotorControlObserver_AddAngle(1234U, 32768),
                   1) == E_OK);
    first.estimatedCurrentAlphaQ15 = 100;
    first.estimatedCurrentBetaQ15 = -200;
    first.estimatedBemfAlphaQ15 = 300;
    first.estimatedBemfBetaQ15 = 400;
    second.estimatedCurrentAlphaQ15 = -first.estimatedCurrentAlphaQ15;
    second.estimatedCurrentBetaQ15 = -first.estimatedCurrentBetaQ15;
    second.estimatedBemfAlphaQ15 = -first.estimatedBemfAlphaQ15;
    second.estimatedBemfBetaQ15 = -first.estimatedBemfBetaQ15;
    firstInput.measuredCurrentAlphaQ15 = 500;
    firstInput.measuredCurrentBetaQ15 = -600;
    firstInput.appliedVoltageAlphaQ15 = 700;
    firstInput.appliedVoltageBetaQ15 = -800;
    secondInput.measuredCurrentAlphaQ15 = -firstInput.measuredCurrentAlphaQ15;
    secondInput.measuredCurrentBetaQ15 = -firstInput.measuredCurrentBetaQ15;
    secondInput.appliedVoltageAlphaQ15 = -firstInput.appliedVoltageAlphaQ15;
    secondInput.appliedVoltageBetaQ15 = -firstInput.appliedVoltageBetaQ15;
    TEST_CHECK(MotorControlObserver_Step(
                   &first, &firstInput, &firstResult) == E_OK);
    TEST_CHECK(MotorControlObserver_Step(
                   &second, &secondInput, &secondResult) == E_OK);
    TEST_CHECK(firstResult.estimatedCurrentAlphaQ15 ==
               -secondResult.estimatedCurrentAlphaQ15);
    TEST_CHECK(firstResult.estimatedCurrentBetaQ15 ==
               -secondResult.estimatedCurrentBetaQ15);
    TEST_CHECK(firstResult.estimatedBemfAlphaQ15 ==
               -secondResult.estimatedBemfAlphaQ15);
    TEST_CHECK(firstResult.estimatedBemfBetaQ15 ==
               -secondResult.estimatedBemfBetaQ15);
    TEST_CHECK(secondResult.angleQ16 ==
               MotorControlObserver_AddAngle(firstResult.angleQ16, 32768));

    config = Test_Config();
    config.currentErrorToBemfQ15 = 0;
    config.phaseProportionalQ15 = 0;
    config.phaseIntegralQ15 = 0;
    TEST_CHECK(MotorControlObserver_Init(&first, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Init(&second, &config) == E_OK);
    first.estimatedBemfAlphaQ15 = 3000;
    first.estimatedBemfBetaQ15 = 4000;
    second.estimatedBemfAlphaQ15 = first.estimatedBemfAlphaQ15;
    second.estimatedBemfBetaQ15 = first.estimatedBemfBetaQ15;
    TEST_CHECK(MotorControlObserver_CordicAngle(
                   3000, 4000, &bemfAngle) == E_OK);
    positiveRaw = MotorControlObserver_AddAngle(
        bemfAngle, -(sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
    negativeRaw = MotorControlObserver_AddAngle(
        bemfAngle, (sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
    TEST_CHECK(MotorControlObserver_Begin(&first, positiveRaw, 1) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&second, negativeRaw, -1) == E_OK);
    first.estimatedBemfAlphaQ15 = 3000;
    first.estimatedBemfBetaQ15 = 4000;
    second.estimatedBemfAlphaQ15 = 3000;
    second.estimatedBemfBetaQ15 = 4000;
    firstInput = Test_Input();
    secondInput = Test_Input();
    TEST_CHECK(MotorControlObserver_Step(
                   &first, &firstInput, &firstResult) == E_OK);
    TEST_CHECK(MotorControlObserver_Step(
                   &second, &secondInput, &secondResult) == E_OK);
    TEST_CHECK(secondResult.angleQ16 ==
               MotorControlObserver_AddAngle(firstResult.angleQ16, 32768));

    TEST_CHECK(MotorControlObserver_Init(&first, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Init(&second, &config) == E_OK);
    first.estimatedBemfAlphaQ15 = 1000;
    first.estimatedBemfBetaQ15 = 2000;
    second.estimatedBemfAlphaQ15 = 2000;
    second.estimatedBemfBetaQ15 = 4000;
    TEST_CHECK(MotorControlObserver_CordicAngle(
                   1000, 2000, &positiveRaw) == E_OK);
    TEST_CHECK(MotorControlObserver_CordicAngle(
                   2000, 4000, &negativeRaw) == E_OK);
    TEST_CHECK(positiveRaw == negativeRaw);
    positiveRaw = MotorControlObserver_AddAngle(
        positiveRaw, -(sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
    TEST_CHECK(MotorControlObserver_Begin(&first, positiveRaw, 1) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&second, positiveRaw, 1) == E_OK);
    first.estimatedBemfAlphaQ15 = 1000;
    first.estimatedBemfBetaQ15 = 2000;
    second.estimatedBemfAlphaQ15 = 2000;
    second.estimatedBemfBetaQ15 = 4000;
    TEST_CHECK(MotorControlObserver_Step(
                   &first, &firstInput, &firstResult) == E_OK);
    TEST_CHECK(MotorControlObserver_Step(
                   &second, &secondInput, &secondResult) == E_OK);
    TEST_CHECK(secondResult.estimatedBemfAlphaQ15 ==
               (2 * firstResult.estimatedBemfAlphaQ15));
    TEST_CHECK(secondResult.estimatedBemfBetaQ15 ==
               (2 * firstResult.estimatedBemfBetaQ15));
    TEST_CHECK(secondResult.bemfMagnitudeSqQ30 ==
               (4U * firstResult.bemfMagnitudeSqQ30));
    TEST_CHECK(secondResult.angleQ16 == firstResult.angleQ16);
    return 0;
}

STATIC int Test_DeterministicReplay(void)
{
    static const sint32 measuredAlpha[] =
        { 1234, -111, 222, -333, 444, -555, 666, -777 };
    static const sint32 measuredBeta[] =
        { -2345, 101, -202, 303, -404, 505, -606, 707 };
    static const sint32 voltageAlpha[] =
        { 300, -30, 60, -90, 120, -150, 180, -210 };
    static const sint32 voltageBeta[] =
        { -400, 40, -80, 120, -160, 200, -240, 280 };
    MotorControlObserver_ConfigType config = Test_Config();
    MotorControlObserver_ObjectType first;
    MotorControlObserver_ObjectType second;
    MotorControlObserver_ResultType firstResult;
    MotorControlObserver_ResultType secondResult;
    MotorControlObserver_StepInputType input = Test_Input();
    uint32 sample;

    TEST_CHECK(MotorControlObserver_Init(&first, &config) == E_OK);
    TEST_CHECK(MotorControlObserver_Begin(&first, 12345U, -1) == E_OK);
    second = first;
    for (sample = 0U;
         sample < (uint32)(sizeof(measuredAlpha) / sizeof(measuredAlpha[0]));
         sample++)
    {
        input.measuredCurrentAlphaQ15 = measuredAlpha[sample];
        input.measuredCurrentBetaQ15 = measuredBeta[sample];
        input.appliedVoltageAlphaQ15 = voltageAlpha[sample];
        input.appliedVoltageBetaQ15 = voltageBeta[sample];
        TEST_CHECK(MotorControlObserver_Step(
                       &first, &input, &firstResult) == E_OK);
        TEST_CHECK(MotorControlObserver_Step(
                       &second, &input, &secondResult) == E_OK);
        TEST_CHECK(memcmp(&first, &second, sizeof(first)) == 0);
        TEST_CHECK(Test_ResultsEqual(&firstResult, &secondResult) == TRUE);
    }
    return 0;
}

int main(void)
{
    TEST_CHECK(Test_ConfigAndLifecycle() == 0);
    TEST_CHECK(Test_LifecycleTransitionMatrix() == 0);
    TEST_CHECK(Test_ConfigInclusiveBoundaries() == 0);
    TEST_CHECK(Test_CordicAndCircularMath() == 0);
    TEST_CHECK(Test_ObserverEquationBothAxesAndTies() == 0);
    TEST_CHECK(Test_TrackerBoundsAndNoWindup() == 0);
    TEST_CHECK(Test_ReliabilityAndRollingWindow(1) == 0);
    TEST_CHECK(Test_ReliabilityAndRollingWindow(-1) == 0);
    TEST_CHECK(Test_RollingWindowIndependentOracle() == 0);
    TEST_CHECK(Test_ExactReliabilityThresholds() == 0);
    TEST_CHECK(Test_PublicInvalidMatrix() == 0);
    TEST_CHECK(Test_IndependentPredicatesAndFailures() == 0);
    TEST_CHECK(Test_MetamorphicSequences() == 0);
    TEST_CHECK(Test_DeterministicReplay() == 0);
    (void)printf("PASS: MotorControl Observer Step-8B2 host matrix\n");
    return 0;
}
