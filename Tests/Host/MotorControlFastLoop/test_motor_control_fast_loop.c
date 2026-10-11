#include <math.h>
#include <stdio.h>
#include <string.h>

#include "MotorControlFastLoop.h"

#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlAlgorithm.c"
#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlStartup.c"
#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlObserver.c"
#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlFastLoop.c"

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

STATIC MotorControlFastLoop_ConfigType Test_Config(void)
{
    MotorControlFastLoop_ConfigType config = {0};

    config.observer.currentRetentionQ15 = 0;
    config.observer.voltageInjectionQ15 = 0;
    config.observer.bemfSubtractionQ15 = 0;
    config.observer.currentErrorToCurrentQ15 = 0;
    config.observer.currentErrorToBemfQ15 = 0;
    config.observer.phaseProportionalQ15 = 32767;
    config.observer.phaseIntegralQ15 = 32767;
    config.observer.maxAbsCurrentQ15 = 32767;
    config.observer.maxAbsBemfQ15 = 32767;
    config.observer.maxAbsSpeedAngleCounts = 2048U;
    config.observer.maxSpeedDeltaAngleCounts = 2048U;
    config.observer.minBemfMagnitudeSqQ30 = 1U;
    config.observer.maxBemfMagnitudeSqQ30 = 2147352578ULL;
    config.observer.maxCurrentResidualMagnitudeSqQ30 = 8589672450ULL;
    config.observer.speedWindowLength = 2U;
    config.observer.maxSpeedVarianceNumerator = 16777216ULL;
    config.observer.reliabilityWarmupSamples = 2U;
    config.observer.maxConsecutiveUnreliableSamples = 4U;

    config.startup.alignCycles = 2U;
    config.startup.openLoopRampCycles = 4U;
    config.startup.qualificationCycles = 4U;
    config.startup.handoverCycles = 2U;
    config.startup.outputConfirmCycles = 2U;
    config.startup.startAngleQ16 = 0U;
    config.startup.alignDRefQ15 = 0;
    config.startup.alignQRefQ15 = 0;
    config.startup.finalDRefQ15 = 1000;
    config.startup.finalQRefQ15 = 500;
    config.startup.absoluteCurrentCapQ15 = 2000;
    config.startup.finalOpenLoopSpeedAngleCounts = 64;
    config.startup.qualificationRequiredCycles = 2U;
    config.startup.minimumObservableSpeedAngleCounts = 1U;
    config.startup.qualificationMaxAngleErrorQ16 = 32U;
    config.startup.qualificationMaxSpeedErrorAngleCounts = 128U;
    config.startup.handoverMaxAngleCorrectionQ16 = 128U;
    config.startup.handoverMaxSpeedSlewAngleCounts = 128U;
    config.startup.finalMaxAngleErrorQ16 = 128U;
    config.startup.finalMaxSpeedErrorAngleCounts = 32U;

    config.dPi.kpQ15 = 8192;
    config.dPi.kiQ15 = 0;
    config.dPi.lowerQ15 = -32768;
    config.dPi.upperQ15 = 32767;
    config.qPi = config.dPi;
    config.phaseSumToleranceQ15 = 0U;
    config.trigNormToleranceQ30 =
        MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30;
    return config;
}

STATIC MotorControlFastLoop_StepInputType Test_Input(uint32 generation)
{
    MotorControlFastLoop_StepInputType input = {0};

    input.sampleGenerationId = generation;
    input.appliedVoltageGenerationId = generation;
    input.currentSampleValid = TRUE;
    input.appliedVoltageValid = TRUE;
    input.generationOwnerValid = TRUE;
    input.safetyValid = TRUE;
    input.commandContinueValid = TRUE;
    input.stopRequested = FALSE;
    return input;
}

STATIC sint32 Test_RoundDouble(double value)
{
    return (value >= 0.0) ?
        (sint32)floor(value + 0.5) :
        (sint32)ceil(value - 0.5);
}

STATIC uint32 Test_AbsDifference(sint32 first, sint32 second)
{
    const sint64 delta = (sint64)first - (sint64)second;

    return (uint32)((delta >= 0) ? delta : -delta);
}

STATIC void Test_SetBemfForRotor(
    MotorControlFastLoop_Type *object,
    uint16 rotorAngle)
{
    const uint16 bemfAngle = MotorControlObserver_AddAngle(
        rotorAngle,
        (object->expectedDirection > 0) ?
            (sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN :
            -(sint32)MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN);
    const double radians =
        (double)bemfAngle * (2.0 * TEST_PI) / 65536.0;

    object->observer.estimatedBemfAlphaQ15 =
        Test_RoundDouble(10000.0 * cos(radians));
    object->observer.estimatedBemfBetaQ15 =
        Test_RoundDouble(10000.0 * sin(radians));
}

STATIC void Test_PrimeForNextCycle(MotorControlFastLoop_Type *object)
{
    uint16 target;

    if (object->mode == MOTOR_CONTROL_FAST_LOOP_MODE_RUN)
    {
        target = MotorControlObserver_AddAngle(
            object->observer.angleQ16,
            object->config.startup.finalOpenLoopSpeedAngleCounts);
    }
    else if (object->startup.phase == MOTOR_CONTROL_STARTUP_PHASE_ALIGN)
    {
        target = object->startup.selectedAngleQ16;
    }
    else
    {
        target = MotorControlObserver_AddAngle(
            object->startup.selectedAngleQ16,
            object->startup.selectedSpeedAngleCounts);
    }
    Test_SetBemfForRotor(object, target);
}

STATIC int Test_BeginDefault(MotorControlFastLoop_Type *object)
{
    MotorControlFastLoop_ConfigType config = Test_Config();

    TEST_CHECK(MotorControlFastLoop_Init(object, &config) == E_OK);
    TEST_CHECK(MotorControlFastLoop_Begin(object, 1, 10U) == E_OK);
    return 0;
}

STATIC int Test_ReachRun(
    MotorControlFastLoop_Type *object,
    uint32 *generation,
    MotorControlFastLoop_ResultType *lastResult)
{
    uint32 cycle;

    TEST_CHECK(Test_BeginDefault(object) == 0);
    for (cycle = 0U;
         (cycle < 32U) &&
         (object->mode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP);
         cycle++)
    {
        MotorControlFastLoop_StepInputType input = Test_Input(*generation);
        Std_ReturnType stepStatus;

        Test_PrimeForNextCycle(object);
        stepStatus = MotorControlFastLoop_Step(object, &input, lastResult);
        if (stepStatus != E_OK)
        {
            (void)printf(
                "TRACE: cycle=%lu phase=%d mode=%d lifecycle=%d failure=%d "
                "startupFailure=%d observerFailure=%d selected=%u/%ld "
                "observer=%u/%ld qualified=%lu\n",
                (unsigned long)cycle,
                (int)object->startup.phase,
                (int)object->mode,
                (int)object->lifecycle,
                (int)object->failure,
                (int)object->startup.failure,
                (int)object->observer.failure,
                (unsigned int)object->startup.selectedAngleQ16,
                (long)object->startup.selectedSpeedAngleCounts,
                (unsigned int)object->observer.angleQ16,
                (long)object->observer.speedAngleCounts,
                (unsigned long)object->startup.qualificationConsecutiveCycles);
        }
        TEST_CHECK(stepStatus == E_OK);
        TEST_CHECK(lastResult->controlValid == TRUE);
        TEST_CHECK(lastResult->mode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP);
        (*generation)++;
    }
    TEST_CHECK(object->mode == MOTOR_CONTROL_FAST_LOOP_MODE_RUN);
    TEST_CHECK(object->startup.phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE);
    TEST_CHECK(object->retainedStartupResult.phase ==
               MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM);
    TEST_CHECK(object->retainedStartupResult.readyForRun == TRUE);
    TEST_CHECK(lastResult->readyForRun == TRUE);
    return 0;
}

STATIC int Test_SinCosExhaustive(void)
{
    sint32 sinQ15;
    sint32 cosQ15;
    sint32 quarterSin;
    sint32 quarterCos;
    sint32 halfSin;
    sint32 halfCos;
    uint32 angle;
    uint32 maxComponentError = 0U;
    uint64 maxNormError = 0U;

    sinQ15 = 1;
    TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                   0U, &sinQ15, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(sinQ15 == 0);
    cosQ15 = 1;
    TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                   0U, NULL_PTR, &cosQ15) == E_NOT_OK);
    TEST_CHECK(cosQ15 == 0);

    for (angle = 0U; angle <= 65535U; angle++)
    {
        const double radians =
            (double)angle * (2.0 * TEST_PI) / 65536.0;
        const sint32 expectedSin =
            Test_RoundDouble(32767.0 * sin(radians));
        const sint32 expectedCos =
            Test_RoundDouble(32767.0 * cos(radians));
        uint64 norm;
        uint64 normError;
        uint32 componentError;

        TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                       (uint16)angle, &sinQ15, &cosQ15) == E_OK);
        TEST_CHECK((sinQ15 >= -32768) && (sinQ15 <= 32767));
        TEST_CHECK((cosQ15 >= -32768) && (cosQ15 <= 32767));
        componentError = Test_AbsDifference(sinQ15, expectedSin);
        if (Test_AbsDifference(cosQ15, expectedCos) > componentError)
        {
            componentError = Test_AbsDifference(cosQ15, expectedCos);
        }
        if (componentError > maxComponentError)
        {
            maxComponentError = componentError;
        }
        norm = (uint64)((sint64)sinQ15 * (sint64)sinQ15) +
               (uint64)((sint64)cosQ15 * (sint64)cosQ15);
        normError = (norm >= 1073741824ULL) ?
            (norm - 1073741824ULL) : (1073741824ULL - norm);
        if (normError > maxNormError)
        {
            maxNormError = normError;
        }

        TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                       (uint16)(angle + 16384U),
                       &quarterSin, &quarterCos) == E_OK);
        TEST_CHECK(quarterSin == cosQ15);
        TEST_CHECK(quarterCos == -sinQ15);
        TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                       (uint16)(angle + 32768U),
                       &halfSin, &halfCos) == E_OK);
        TEST_CHECK(halfSin == -sinQ15);
        TEST_CHECK(halfCos == -cosQ15);
    }

    TEST_CHECK(maxComponentError == 10U);
    TEST_CHECK(maxNormError == 110174ULL);
    TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                   0U, &sinQ15, &cosQ15) == E_OK);
    TEST_CHECK((sinQ15 == 0) && (cosQ15 == 32767));
    TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                   16384U, &sinQ15, &cosQ15) == E_OK);
    TEST_CHECK((sinQ15 == 32767) && (cosQ15 == 0));
    TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                   32768U, &sinQ15, &cosQ15) == E_OK);
    TEST_CHECK((sinQ15 == 0) && (cosQ15 == -32767));
    TEST_CHECK(MotorControlFastLoop_SinCosQ15(
                   49152U, &sinQ15, &cosQ15) == E_OK);
    TEST_CHECK((sinQ15 == -32767) && (cosQ15 == 0));
    return 0;
}

STATIC int Test_RoundingTies(void)
{
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(16383) == 0);
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(16384) == 1);
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(49151) == 1);
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(49152) == 2);
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(-16383) == 0);
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(-16384) == -1);
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(-49151) == -1);
    TEST_CHECK(MotorControlFastLoop_RoundQ30ToQ15(-49152) == -2);
    return 0;
}

STATIC int Test_ConfigurationAndLifecycle(void)
{
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_Type before;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input = Test_Input(1U);

    TEST_CHECK(MotorControlFastLoop_Init(NULL_PTR, &config) == E_NOT_OK);
    (void)memset(&object, 0xA5, sizeof(object));
    TEST_CHECK(MotorControlFastLoop_Init(&object, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_UNINITIALIZED);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_CONFIGURATION);

    config = Test_Config();
    config.phaseSumToleranceQ15 = 98302U;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);
    config = Test_Config();
    config.trigNormToleranceQ30 = 262143U;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);
    config = Test_Config();
    config.trigNormToleranceQ30 = 1073741825U;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);
    config = Test_Config();
    config.observer.maxAbsSpeedAngleCounts = 3U;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);
    config = Test_Config();
    config.startup.minimumObservableSpeedAngleCounts = 2049U;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);
    config = Test_Config();
    config.startup.minimumObservableSpeedAngleCounts =
        config.observer.maxAbsSpeedAngleCounts;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    config = Test_Config();
    config.startup.finalOpenLoopSpeedAngleCounts =
        (sint32)config.observer.maxAbsSpeedAngleCounts;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    config = Test_Config();
    config.startup.finalOpenLoopSpeedAngleCounts =
        -(sint32)config.observer.maxAbsSpeedAngleCounts;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    config = Test_Config();
    config.dPi.lowerQ15 = 1;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);
    config = Test_Config();
    config.startup.absoluteCurrentCapQ15 = 0;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);

    config = Test_Config();
    config.phaseSumToleranceQ15 =
        MOTOR_CONTROL_FAST_LOOP_PHASE_SUM_MAX_Q15;
    config.trigNormToleranceQ30 =
        MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MAX_Q30;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_IDLE);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE);
    before = object;
    TEST_CHECK(MotorControlFastLoop_ResetFromReady(&object) == E_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    before = object;
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 0, 10U) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, -1, 10U) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 0U) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlFastLoop_Begin(
                   &object, 1, MOTOR_CONTROL_STARTUP_MAX_CYCLES) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 99986U) == E_OK);
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    before = object;
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 99987U) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlFastLoop_Begin(NULL_PTR, 1, 10U) == E_NOT_OK);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_NOT_OK);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_OK);
    before = object;
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlFastLoop_ResetFromReady(&object) == E_NOT_OK);
    TEST_CHECK(MotorControlFastLoop_Step(
                   NULL_PTR, &input, &result) == E_NOT_OK);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, NULL_PTR, &result) == E_NOT_OK);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, NULL_PTR) == E_NOT_OK);

    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    object.lifecycle = (MotorControlFastLoop_LifecycleType)99;
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_NOT_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_FAILED);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL);

    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    object.dPi.integratorQ30 = ((sint64)32768 * 32768);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL);

    config = Test_Config();
    config.startup.finalOpenLoopSpeedAngleCounts = -64;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    before = object;
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, -1, 10U) == E_OK);
    return 0;
}

STATIC int Test_GenerationAndStop(void)
{
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input;

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    Test_PrimeForNextCycle(&object);
    input = Test_Input(0xFFFFFFFFUL);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.controlValid == TRUE);
    TEST_CHECK(result.sourceGenerationId == 0xFFFFFFFFUL);
    TEST_CHECK(result.nextGenerationId == 0U);

    Test_PrimeForNextCycle(&object);
    input = Test_Input(0U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.sourceGenerationId == 0U);

    Test_PrimeForNextCycle(&object);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR);
    TEST_CHECK(result.controlValid == FALSE);
    TEST_CHECK(result.lifecycle == MOTOR_CONTROL_FAST_LOOP_FAILED);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_NOT_OK);
    TEST_CHECK(result.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    input = Test_Input(55U);
    input.appliedVoltageGenerationId = 56U;
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    input = Test_Input(55U);
    input.currentSampleValid = FALSE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    input = Test_Input(55U);
    input.commandContinueValid = (boolean)2U;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_INPUT_REPRESENTATION);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    Test_PrimeForNextCycle(&object);
    input = Test_Input(10U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    Test_PrimeForNextCycle(&object);
    input = Test_Input(12U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    Test_PrimeForNextCycle(&object);
    input = Test_Input(10U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    Test_PrimeForNextCycle(&object);
    input = Test_Input(9U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure ==
               MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    input = Test_Input(55U);
    input.stopRequested = TRUE;
    Test_PrimeForNextCycle(&object);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED);
    TEST_CHECK(object.startup.phase == MOTOR_CONTROL_STARTUP_PHASE_ABORTED);
    TEST_CHECK(result.controlValid == FALSE);
    TEST_CHECK(result.stopRequired == TRUE);
    TEST_CHECK(MotorControlFastLoop_ResetFromReady(&object) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_IDLE);
    TEST_CHECK(object.generationEpochValid == FALSE);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_OK);
    Test_PrimeForNextCycle(&object);
    input = Test_Input(0x12345678UL);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.sourceGenerationId == 0x12345678UL);
    return 0;
}

STATIC int Test_StartupToRunAndRunGuards(void)
{
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input;
    MotorControlStartup_ResultType retained;
    uint32 generation = 100U;
    uint32 runCycle;
    uint16 previousAngle;
    sint32 previousSpeed;

    TEST_CHECK(Test_ReachRun(&object, &generation, &result) == 0);
    retained = object.retainedStartupResult;
    previousAngle = object.observer.angleQ16;
    previousSpeed = object.observer.speedAngleCounts;
    for (runCycle = 0U; runCycle < 3U; runCycle++)
    {
        const sint32 requestedAdvance =
            object.config.startup.finalOpenLoopSpeedAngleCounts +
            (sint32)((runCycle + 1U) * 16U);
        const uint16 target = MotorControlObserver_AddAngle(
            object.observer.angleQ16, requestedAdvance);

        Test_SetBemfForRotor(&object, target);
        input = Test_Input(generation++);
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_OK);
        TEST_CHECK(result.mode == MOTOR_CONTROL_FAST_LOOP_MODE_RUN);
        TEST_CHECK(result.controlValid == TRUE);
        TEST_CHECK(result.readyForRun == TRUE);
        TEST_CHECK(result.selectedAngleQ16 == result.observer.angleQ16);
        TEST_CHECK(result.selectedSpeedAngleCounts ==
                   result.observer.speedAngleCounts);
        TEST_CHECK(result.selectedAngleQ16 != previousAngle);
        TEST_CHECK(result.selectedSpeedAngleCounts != previousSpeed);
        TEST_CHECK(result.dRefQ15 == object.config.startup.finalDRefQ15);
        TEST_CHECK(result.qRefQ15 == object.config.startup.finalQRefQ15);
        TEST_CHECK(memcmp(
                       &object.retainedStartupResult,
                       &retained,
                       sizeof(retained)) == 0);
        previousAngle = result.selectedAngleQ16;
        previousSpeed = result.selectedSpeedAngleCounts;
    }

    Test_PrimeForNextCycle(&object);
    input = Test_Input(generation);
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED);
    TEST_CHECK(object.startup.phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE);
    TEST_CHECK(result.controlValid == FALSE);
    TEST_CHECK(result.stopRequired == TRUE);
    TEST_CHECK(MotorControlFastLoop_ResetFromReady(&object) == E_OK);
    TEST_CHECK(object.startup.phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE);
    TEST_CHECK(object.observer.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);

    generation = 1000U;
    TEST_CHECK(Test_ReachRun(&object, &generation, &result) == 0);
    Test_PrimeForNextCycle(&object);
    input = Test_Input(generation);
    input.safetyValid = FALSE;
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD);

    generation = 2000U;
    TEST_CHECK(Test_ReachRun(&object, &generation, &result) == 0);
    object.observer.estimatedBemfAlphaQ15 = 0;
    object.observer.estimatedBemfBetaQ15 = 0;
    input = Test_Input(generation);
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_OBSERVER);
    return 0;
}

STATIC int Test_AlgorithmAndInternalFailures(void)
{
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input;
    MotorControlFastLoop_Type failed;

    config.phaseSumToleranceQ15 = 0U;
    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_OK);
    input = Test_Input(1U);
    input.phaseAQ15 = 1;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    object.observer.speedSum = 1;
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    object.config.trigNormToleranceQ30 = 0U;
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL);

    TEST_CHECK(Test_BeginDefault(&object) == 0);
    object.failure = (MotorControlFastLoop_FailureType)99;
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_FAILED);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL);
    TEST_CHECK((result.lifecycle == MOTOR_CONTROL_FAST_LOOP_FAILED) &&
               (result.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL));
    TEST_CHECK((result.controlValid == FALSE) &&
               (result.readyForRun == FALSE) &&
               (result.stopRequired == FALSE));
    TEST_CHECK(object.generationEpochValid == FALSE);
    failed = object;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(memcmp(&object, &failed, sizeof(object)) == 0);
    TEST_CHECK((result.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL) &&
               (result.controlValid == FALSE));
    return 0;
}

STATIC int Test_DeterministicReplay(void)
{
    MotorControlFastLoop_Type first;
    MotorControlFastLoop_Type second;
    MotorControlFastLoop_ResultType firstResult;
    MotorControlFastLoop_ResultType secondResult;
    MotorControlFastLoop_StepInputType input;
    uint32 generation;
    uint32 cycle;

    TEST_CHECK(Test_BeginDefault(&first) == 0);
    TEST_CHECK(Test_BeginDefault(&second) == 0);
    generation = 0xABCDEF00UL;
    for (cycle = 0U; cycle < 12U; cycle++)
    {
        Test_PrimeForNextCycle(&first);
        Test_PrimeForNextCycle(&second);
        input = Test_Input(generation++);
        TEST_CHECK(MotorControlFastLoop_Step(
                       &first, &input, &firstResult) == E_OK);
        TEST_CHECK(MotorControlFastLoop_Step(
                       &second, &input, &secondResult) == E_OK);
        TEST_CHECK(firstResult.controlValid == secondResult.controlValid);
        TEST_CHECK(firstResult.mode == secondResult.mode);
        TEST_CHECK(firstResult.selectedAngleQ16 ==
                   secondResult.selectedAngleQ16);
        TEST_CHECK(firstResult.selectedSpeedAngleCounts ==
                   secondResult.selectedSpeedAngleCounts);
        TEST_CHECK(firstResult.requestedVoltage.alphaQ15 ==
                   secondResult.requestedVoltage.alphaQ15);
        TEST_CHECK(firstResult.requestedVoltage.betaQ15 ==
                   secondResult.requestedVoltage.betaQ15);
        TEST_CHECK(firstResult.requestedSvpwm.dutyAQ15 ==
                   secondResult.requestedSvpwm.dutyAQ15);
        TEST_CHECK(firstResult.requestedSvpwm.dutyBQ15 ==
                   secondResult.requestedSvpwm.dutyBQ15);
        TEST_CHECK(firstResult.requestedSvpwm.dutyCQ15 ==
                   secondResult.requestedSvpwm.dutyCQ15);
    }
    return 0;
}

STATIC int Test_FirstVector(
    sint32 phaseA,
    sint32 phaseB,
    sint32 phaseC,
    MotorControlFastLoop_ResultType *result)
{
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_StepInputType input = Test_Input(1U);

    TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 10U) == E_OK);
    Test_PrimeForNextCycle(&object);
    input.phaseAQ15 = phaseA;
    input.phaseBQ15 = phaseB;
    input.phaseCQ15 = phaseC;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, result) == E_OK);
    TEST_CHECK((result->controlValid == TRUE) &&
               (result->mode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP));
    return 0;
}

STATIC int Test_MetamorphicComposition(void)
{
    MotorControlFastLoop_ResultType base;
    MotorControlFastLoop_ResultType sign;
    MotorControlFastLoop_ResultType scale;
    MotorControlFastLoop_ResultType rotation;

    TEST_CHECK(Test_FirstVector(1000, -500, -500, &base) == 0);
    TEST_CHECK(Test_FirstVector(-1000, 500, 500, &sign) == 0);
    TEST_CHECK(Test_FirstVector(2000, -1000, -1000, &scale) == 0);
    TEST_CHECK(Test_FirstVector(-500, 1000, -500, &rotation) == 0);

    TEST_CHECK(Test_AbsDifference(
                   sign.requestedVoltage.alphaQ15,
                   -base.requestedVoltage.alphaQ15) <= 1U);
    TEST_CHECK(Test_AbsDifference(
                   sign.requestedVoltage.betaQ15,
                   -base.requestedVoltage.betaQ15) <= 1U);
    TEST_CHECK(Test_AbsDifference(
                   scale.requestedVoltage.alphaQ15,
                   2 * base.requestedVoltage.alphaQ15) <= 1U);
    TEST_CHECK(Test_AbsDifference(
                   scale.requestedVoltage.betaQ15,
                   2 * base.requestedVoltage.betaQ15) <= 1U);

    TEST_CHECK(Test_AbsDifference(
                   (sint32)rotation.requestedSvpwm.dutyAQ15,
                   (sint32)base.requestedSvpwm.dutyCQ15) <= 1U);
    TEST_CHECK(Test_AbsDifference(
                   (sint32)rotation.requestedSvpwm.dutyBQ15,
                   (sint32)base.requestedSvpwm.dutyAQ15) <= 1U);
    TEST_CHECK(Test_AbsDifference(
                   (sint32)rotation.requestedSvpwm.dutyCQ15,
                   (sint32)base.requestedSvpwm.dutyBQ15) <= 1U);

    TEST_CHECK(Test_AbsDifference(
                   (sint32)base.requestedSvpwm.dutyAQ15 +
                       (sint32)sign.requestedSvpwm.dutyAQ15,
                   32767) <= 1U);
    TEST_CHECK(Test_AbsDifference(
                   (sint32)base.requestedSvpwm.dutyBQ15 +
                       (sint32)sign.requestedSvpwm.dutyBQ15,
                   32767) <= 1U);
    TEST_CHECK(Test_AbsDifference(
                   (sint32)base.requestedSvpwm.dutyCQ15 +
                       (sint32)sign.requestedSvpwm.dutyCQ15,
                   32767) <= 1U);
    return 0;
}

int main(void)
{
    TEST_CHECK(Test_SinCosExhaustive() == 0);
    TEST_CHECK(Test_RoundingTies() == 0);
    TEST_CHECK(Test_ConfigurationAndLifecycle() == 0);
    TEST_CHECK(Test_GenerationAndStop() == 0);
    TEST_CHECK(Test_StartupToRunAndRunGuards() == 0);
    TEST_CHECK(Test_AlgorithmAndInternalFailures() == 0);
    TEST_CHECK(Test_DeterministicReplay() == 0);
    TEST_CHECK(Test_MetamorphicComposition() == 0);

    (void)printf("PASS: MotorControl FastLoop Step-8C host matrix\n");
    return 0;
}
