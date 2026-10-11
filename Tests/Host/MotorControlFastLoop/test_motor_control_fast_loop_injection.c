#include <stdio.h>
#include <string.h>

#include "Std_Types.h"

Std_ReturnType Stub_SinCos(
    uint16 AngleQ16,
    sint32 *SinQ15,
    sint32 *CosQ15);

#define MOTOR_CONTROL_FAST_LOOP_HOST_TEST
#define MOTOR_CONTROL_FAST_LOOP_SINCOS_CALL Stub_SinCos
#define MotorControlAlgorithm_Clarke Stub_Clarke
#define MotorControlAlgorithm_Park Stub_Park
#define MotorControlAlgorithm_InversePark Stub_InversePark
#define MotorControlAlgorithm_PiInit Stub_PiInit
#define MotorControlAlgorithm_PiReset Stub_PiReset
#define MotorControlAlgorithm_PiStep Stub_PiStep
#define MotorControlAlgorithm_Svpwm Stub_Svpwm
#define MotorControlObserver_Init Stub_ObserverInit
#define MotorControlObserver_Begin Stub_ObserverBegin
#define MotorControlObserver_Step Stub_ObserverStep
#define MotorControlObserver_ResetFromReady Stub_ObserverReset
#define MotorControlStartup_Init Stub_StartupInit
#define MotorControlStartup_Begin Stub_StartupBegin
#define MotorControlStartup_ValidateCycle Stub_StartupValidateCycle
#define MotorControlStartup_Step Stub_StartupStep
#define MotorControlStartup_RearmFromReady Stub_StartupRearm

#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlFastLoop.c"

#undef MotorControlAlgorithm_Clarke
#undef MotorControlAlgorithm_Park
#undef MotorControlAlgorithm_InversePark
#undef MotorControlAlgorithm_PiInit
#undef MotorControlAlgorithm_PiReset
#undef MotorControlAlgorithm_PiStep
#undef MotorControlAlgorithm_Svpwm
#undef MotorControlObserver_Init
#undef MotorControlObserver_Begin
#undef MotorControlObserver_Step
#undef MotorControlObserver_ResetFromReady
#undef MotorControlStartup_Init
#undef MotorControlStartup_Begin
#undef MotorControlStartup_ValidateCycle
#undef MotorControlStartup_Step
#undef MotorControlStartup_RearmFromReady
#undef MOTOR_CONTROL_FAST_LOOP_SINCOS_CALL
#undef MOTOR_CONTROL_FAST_LOOP_HOST_TEST

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

STATIC uint32 Stub_ExpectedStages[9];
STATIC uint32 Stub_ExpectedCount;
STATIC uint32 Stub_StageIndex;
STATIC uint32 Stub_OrderError;
STATIC uint32 Stub_FailStage;
STATIC uint32 Stub_InitFailStage;
STATIC uint32 Stub_BeginFailStage;
STATIC uint32 Stub_ResetFailStage;
STATIC uint32 Stub_PiInitCall;
STATIC uint32 Stub_PiResetCall;
STATIC uint32 Stub_PiStepCall;
STATIC boolean Stub_StartupTransition;
STATIC boolean Stub_ObserverReliable;
STATIC uint32 Stub_InvalidObserverPredicate;
STATIC boolean Stub_ForceExtremeDq;
STATIC uint16 Stub_ObserverAngle;
STATIC sint32 Stub_ObserverSpeed;
STATIC MotorControlObserver_StepInputType Stub_LastObserverInput;
STATIC MotorControlStartup_StepInputType Stub_LastStartupInput;
STATIC sint32 Stub_LastPhaseA;
STATIC sint32 Stub_LastPhaseB;
STATIC sint32 Stub_LastPhaseC;
STATIC uint32 Stub_LastPhaseTolerance;
STATIC sint32 Stub_LastDError;
STATIC sint32 Stub_LastQError;

STATIC void Stub_ClearControls(void)
{
    Stub_ExpectedCount = 0U;
    Stub_StageIndex = 0U;
    Stub_OrderError = 0U;
    Stub_FailStage = 0U;
    Stub_InitFailStage = 0U;
    Stub_BeginFailStage = 0U;
    Stub_ResetFailStage = 0U;
    Stub_PiInitCall = 0U;
    Stub_PiResetCall = 0U;
    Stub_PiStepCall = 0U;
    Stub_StartupTransition = FALSE;
    Stub_ObserverReliable = TRUE;
    Stub_InvalidObserverPredicate = 0U;
    Stub_ForceExtremeDq = FALSE;
    Stub_ObserverAngle = 100U;
    Stub_ObserverSpeed = 64;
    Stub_LastObserverInput = (MotorControlObserver_StepInputType){0};
    Stub_LastStartupInput = (MotorControlStartup_StepInputType){0};
    Stub_LastPhaseA = 0;
    Stub_LastPhaseB = 0;
    Stub_LastPhaseC = 0;
    Stub_LastPhaseTolerance = 0U;
    Stub_LastDError = 0;
    Stub_LastQError = 0;
}

STATIC void Stub_SetStages(const uint32 *stages, uint32 count)
{
    uint32 index;

    Stub_ExpectedCount = count;
    Stub_StageIndex = 0U;
    Stub_OrderError = 0U;
    Stub_PiStepCall = 0U;
    for (index = 0U; index < count; index++)
    {
        Stub_ExpectedStages[index] = stages[index];
    }
}

STATIC void Stub_RecordStage(uint32 stage)
{
    if ((Stub_StageIndex >= Stub_ExpectedCount) ||
        (Stub_ExpectedStages[Stub_StageIndex] != stage))
    {
        Stub_OrderError = 1U;
    }
    Stub_StageIndex++;
}

Std_ReturnType Stub_ObserverInit(
    MotorControlObserver_ObjectType *Object,
    const MotorControlObserver_ConfigType *Config)
{
    if ((Object == NULL_PTR) || (Config == NULL_PTR) ||
        (Stub_InitFailStage == 1U))
    {
        return E_NOT_OK;
    }
    *Object = (MotorControlObserver_ObjectType){0};
    Object->config = *Config;
    Object->lifecycle = MOTOR_CONTROL_OBSERVER_IDLE;
    return E_OK;
}

Std_ReturnType Stub_ObserverBegin(
    MotorControlObserver_ObjectType *Object,
    uint16 InitialAngleQ16,
    sint32 ExpectedDirection)
{
    if (Stub_BeginFailStage == 1U)
    {
        return E_NOT_OK;
    }
    Object->angleQ16 = InitialAngleQ16;
    Object->expectedDirection = ExpectedDirection;
    Object->lifecycle = MOTOR_CONTROL_OBSERVER_ACTIVE;
    return E_OK;
}

Std_ReturnType Stub_ObserverStep(
    MotorControlObserver_ObjectType *Object,
    const MotorControlObserver_StepInputType *Input,
    MotorControlObserver_ResultType *Result)
{
    Stub_RecordStage(2U);
    Stub_LastObserverInput = *Input;
    if (Stub_FailStage == 2U)
    {
        Object->failure = MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_QUALITY;
        Object->lifecycle = MOTOR_CONTROL_OBSERVER_FAILED;
        return E_NOT_OK;
    }
    *Result = (MotorControlObserver_ResultType){0};
    Result->angleQ16 = Stub_ObserverAngle;
    Result->speedAngleCounts = Stub_ObserverSpeed;
    Result->numericValid = TRUE;
    Result->angleValid = Stub_ObserverReliable;
    Result->speedValid = TRUE;
    Result->varianceValid = TRUE;
    Result->bemfValid = TRUE;
    Result->currentValid = TRUE;
    Result->directionValid = TRUE;
    if (Stub_InvalidObserverPredicate == 1U)
    {
        Result->numericValid = FALSE;
    }
    else if (Stub_InvalidObserverPredicate == 2U)
    {
        Result->angleValid = FALSE;
    }
    else if (Stub_InvalidObserverPredicate == 3U)
    {
        Result->speedValid = FALSE;
    }
    else if (Stub_InvalidObserverPredicate == 4U)
    {
        Result->varianceValid = FALSE;
    }
    else if (Stub_InvalidObserverPredicate == 5U)
    {
        Result->bemfValid = FALSE;
    }
    else if (Stub_InvalidObserverPredicate == 6U)
    {
        Result->currentValid = FALSE;
    }
    else if (Stub_InvalidObserverPredicate == 7U)
    {
        Result->directionValid = FALSE;
    }
    Result->lifecycle = MOTOR_CONTROL_OBSERVER_ACTIVE;
    Object->angleQ16 = Result->angleQ16;
    Object->speedAngleCounts = Result->speedAngleCounts;
    Object->speedIntegratorQ15 =
        (sint64)Result->speedAngleCounts * (sint64)32768;
    return E_OK;
}

Std_ReturnType Stub_ObserverReset(MotorControlObserver_ObjectType *Object)
{
    if (Stub_ResetFailStage == 2U)
    {
        return E_NOT_OK;
    }
    Object->angleQ16 = 0U;
    Object->speedAngleCounts = 0;
    Object->speedIntegratorQ15 = 0;
    Object->expectedDirection = 0;
    Object->lifecycle = MOTOR_CONTROL_OBSERVER_IDLE;
    return E_OK;
}

Std_ReturnType Stub_StartupInit(
    MotorControlStartup_Type *Object,
    const MotorControlStartup_ConfigType *Config)
{
    if ((Object == NULL_PTR) || (Config == NULL_PTR) ||
        (Stub_InitFailStage == 2U))
    {
        return E_NOT_OK;
    }
    *Object = (MotorControlStartup_Type){0};
    Object->config = *Config;
    Object->initialized = TRUE;
    Object->phase = MOTOR_CONTROL_STARTUP_PHASE_IDLE;
    return E_OK;
}

Std_ReturnType Stub_StartupBegin(
    MotorControlStartup_Type *Object,
    uint32 AlreadyConsumedStartingCycles)
{
    if ((AlreadyConsumedStartingCycles == 0U) ||
        (Stub_BeginFailStage == 2U))
    {
        return E_NOT_OK;
    }
    Object->phase = MOTOR_CONTROL_STARTUP_PHASE_ALIGN;
    Object->selectedAngleQ16 = Object->config.startAngleQ16;
    Object->dRefQ15 = Object->config.alignDRefQ15;
    Object->qRefQ15 = Object->config.alignQRefQ15;
    return E_OK;
}

Std_ReturnType Stub_StartupValidateCycle(
    const MotorControlStartup_Type *Object,
    const MotorControlStartup_StepInputType *Input,
    sint32 *CurrentQRefQ15)
{
    if ((Object == NULL_PTR) || (Input == NULL_PTR) ||
        (CurrentQRefQ15 == NULL_PTR) ||
        (Input->generationValid != TRUE) ||
        (Input->safetyValid != TRUE) ||
        (Stub_FailStage == 3U))
    {
        return E_NOT_OK;
    }
    *CurrentQRefQ15 = Object->qRefQ15;
    return E_OK;
}

Std_ReturnType Stub_StartupStep(
    MotorControlStartup_Type *Object,
    const MotorControlStartup_StepInputType *Input,
    MotorControlStartup_ResultType *Result)
{
    Stub_RecordStage(3U);
    Stub_LastStartupInput = *Input;
    *Result = (MotorControlStartup_ResultType){0};
    if ((Stub_FailStage == 3U) || (Input->safetyValid != TRUE))
    {
        Object->phase = MOTOR_CONTROL_STARTUP_PHASE_FAILED;
        Object->failure = MOTOR_CONTROL_STARTUP_FAILURE_GUARD;
        Result->failure = MOTOR_CONTROL_STARTUP_FAILURE_GUARD;
        return E_NOT_OK;
    }
    if ((Input->stopRequested == TRUE) ||
        (Input->commandContinueValid != TRUE))
    {
        Object->phase = MOTOR_CONTROL_STARTUP_PHASE_ABORTED;
        Result->phase = MOTOR_CONTROL_STARTUP_PHASE_ALIGN;
        Result->stopRequired = TRUE;
        return E_OK;
    }
    Result->phase = MOTOR_CONTROL_STARTUP_PHASE_ALIGN;
    Result->selectedAngleQ16 = Object->selectedAngleQ16;
    Result->selectedSpeedAngleCounts = 64;
    Result->dRefQ15 = Object->config.finalDRefQ15;
    Result->qRefQ15 = Object->config.finalQRefQ15;
    if (Stub_StartupTransition == TRUE)
    {
        Object->phase = MOTOR_CONTROL_STARTUP_PHASE_COMPLETE;
        Object->readyForRun = TRUE;
        Result->phase = MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM;
        Result->readyForRun = TRUE;
    }
    return E_OK;
}

Std_ReturnType Stub_StartupRearm(MotorControlStartup_Type *Object)
{
    if (Stub_ResetFailStage == 1U)
    {
        return E_NOT_OK;
    }
    Object->phase = MOTOR_CONTROL_STARTUP_PHASE_IDLE;
    Object->failure = MOTOR_CONTROL_STARTUP_FAILURE_NONE;
    Object->readyForRun = FALSE;
    return E_OK;
}

Std_ReturnType Stub_PiInit(
    MotorControlAlgorithm_PiType *Object,
    const MotorControlAlgorithm_PiConfigType *Config,
    sint32 InitialIntegratorQ15)
{
    Stub_PiInitCall++;
    if (Stub_InitFailStage == (Stub_PiInitCall + 2U))
    {
        return E_NOT_OK;
    }
    Object->config = *Config;
    Object->integratorQ30 = (sint64)InitialIntegratorQ15 * (sint64)32768;
    Object->initialized = TRUE;
    return E_OK;
}

Std_ReturnType Stub_PiReset(
    MotorControlAlgorithm_PiType *Object,
    sint32 IntegratorQ15)
{
    Stub_PiResetCall++;
    if ((Stub_BeginFailStage == (Stub_PiResetCall + 2U)) ||
        (Stub_ResetFailStage == (Stub_PiResetCall + 2U)))
    {
        return E_NOT_OK;
    }
    Object->integratorQ30 = (sint64)IntegratorQ15 * (sint64)32768;
    return E_OK;
}

Std_ReturnType Stub_Clarke(
    sint32 PhaseAQ15,
    sint32 PhaseBQ15,
    sint32 PhaseCQ15,
    uint32 PhaseSumToleranceQ15,
    MotorControlAlgorithm_AlphaBetaType *Result)
{
    Stub_RecordStage(1U);
    Stub_LastPhaseA = PhaseAQ15;
    Stub_LastPhaseB = PhaseBQ15;
    Stub_LastPhaseC = PhaseCQ15;
    Stub_LastPhaseTolerance = PhaseSumToleranceQ15;
    if (Stub_FailStage == 1U)
    {
        return E_NOT_OK;
    }
    Result->alphaQ15 = 11;
    Result->betaQ15 = 22;
    return E_OK;
}

Std_ReturnType Stub_SinCos(
    uint16 AngleQ16,
    sint32 *SinQ15,
    sint32 *CosQ15)
{
    (void)AngleQ16;
    Stub_RecordStage(9U);
    *SinQ15 = 0;
    *CosQ15 = 0;
    if (Stub_FailStage == 9U)
    {
        return E_NOT_OK;
    }
    *CosQ15 = 32767;
    return E_OK;
}

Std_ReturnType Stub_Park(
    const MotorControlAlgorithm_AlphaBetaType *AlphaBeta,
    sint32 SinQ15,
    sint32 CosQ15,
    uint32 TrigNormToleranceQ30,
    MotorControlAlgorithm_DqType *Result)
{
    Stub_RecordStage(4U);
    if ((Stub_FailStage == 4U) || (AlphaBeta->alphaQ15 != 11) ||
        (AlphaBeta->betaQ15 != 22) || (SinQ15 < -32768) ||
        (CosQ15 > 32767) ||
        (TrigNormToleranceQ30 !=
         MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30))
    {
        return E_NOT_OK;
    }
    Result->dQ15 = (Stub_ForceExtremeDq == TRUE) ? -32768 : 0;
    Result->qQ15 = 0;
    return E_OK;
}

Std_ReturnType Stub_PiStep(
    MotorControlAlgorithm_PiType *Object,
    sint32 ErrorQ15,
    MotorControlAlgorithm_PiResultType *Result)
{
    uint32 stage;

    (void)Object;
    Stub_PiStepCall++;
    stage = Stub_PiStepCall + 4U;
    Stub_RecordStage(stage);
    if (Stub_PiStepCall == 1U)
    {
        Stub_LastDError = ErrorQ15;
    }
    else
    {
        Stub_LastQError = ErrorQ15;
    }
    if (Stub_FailStage == stage)
    {
        return E_NOT_OK;
    }
    Result->outputQ15 = ErrorQ15;
    Result->limited = FALSE;
    return E_OK;
}

Std_ReturnType Stub_InversePark(
    const MotorControlAlgorithm_DqType *Dq,
    sint32 SinQ15,
    sint32 CosQ15,
    uint32 TrigNormToleranceQ30,
    MotorControlAlgorithm_AlphaBetaType *Result)
{
    Stub_RecordStage(7U);
    (void)SinQ15;
    (void)CosQ15;
    (void)TrigNormToleranceQ30;
    if (Stub_FailStage == 7U)
    {
        return E_NOT_OK;
    }
    Result->alphaQ15 = Dq->dQ15;
    Result->betaQ15 = Dq->qQ15;
    return E_OK;
}

Std_ReturnType Stub_Svpwm(
    const MotorControlAlgorithm_AlphaBetaType *AlphaBeta,
    MotorControlAlgorithm_SvpwmResultType *Result)
{
    Stub_RecordStage(8U);
    if (Stub_FailStage == 8U)
    {
        return E_NOT_OK;
    }
    Result->dutyAQ15 = (uint16)(AlphaBeta->alphaQ15 + 1000);
    Result->dutyBQ15 = (uint16)(AlphaBeta->betaQ15 + 1000);
    Result->dutyCQ15 = 1000U;
    Result->sector = 1U;
    Result->limited = FALSE;
    return E_OK;
}

STATIC MotorControlFastLoop_ConfigType Test_Config(void)
{
    MotorControlFastLoop_ConfigType config = {0};

    config.observer.maxAbsSpeedAngleCounts = 100U;
    config.startup.alignCycles = 1U;
    config.startup.openLoopRampCycles = 1U;
    config.startup.qualificationCycles = 1U;
    config.startup.handoverCycles = 1U;
    config.startup.outputConfirmCycles = 1U;
    config.startup.finalDRefQ15 = 100;
    config.startup.finalQRefQ15 = 200;
    config.startup.absoluteCurrentCapQ15 = 1000;
    config.startup.finalOpenLoopSpeedAngleCounts = 64;
    config.startup.minimumObservableSpeedAngleCounts = 1U;
    config.dPi.lowerQ15 = -32768;
    config.dPi.upperQ15 = 32767;
    config.qPi = config.dPi;
    config.phaseSumToleranceQ15 = 3U;
    config.trigNormToleranceQ30 =
        MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30;
    return config;
}

STATIC MotorControlFastLoop_StepInputType Test_Input(uint32 generation)
{
    MotorControlFastLoop_StepInputType input = {0};

    input.phaseAQ15 = 10;
    input.phaseBQ15 = -4;
    input.phaseCQ15 = -6;
    input.sampleGenerationId = generation;
    input.appliedVoltage.alphaQ15 = 31;
    input.appliedVoltage.betaQ15 = -32;
    input.appliedVoltageGenerationId = generation;
    input.currentSampleValid = TRUE;
    input.appliedVoltageValid = TRUE;
    input.generationOwnerValid = TRUE;
    input.safetyValid = TRUE;
    input.commandContinueValid = TRUE;
    return input;
}

STATIC int Test_InitBegin(
    MotorControlFastLoop_Type *object,
    MotorControlFastLoop_ConfigType *config)
{
    Stub_PiInitCall = 0U;
    Stub_PiResetCall = 0U;
    TEST_CHECK(MotorControlFastLoop_Init(object, config) == E_OK);
    TEST_CHECK(MotorControlFastLoop_Begin(object, 1, 1U) == E_OK);
    return 0;
}

STATIC int Test_MappingAndStageFailures(void)
{
    static const uint32 allStages[] =
        {1U, 2U, 3U, 9U, 4U, 5U, 6U, 7U, 8U};
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input = Test_Input(77U);
    uint32 failureStage;

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_SetStages(allStages, 9U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((Stub_OrderError == 0U) && (Stub_StageIndex == 9U));
    TEST_CHECK((Stub_LastPhaseA == 10) && (Stub_LastPhaseB == -4) &&
               (Stub_LastPhaseC == -6) && (Stub_LastPhaseTolerance == 3U));
    TEST_CHECK((Stub_LastObserverInput.measuredCurrentAlphaQ15 == 11) &&
               (Stub_LastObserverInput.measuredCurrentBetaQ15 == 22));
    TEST_CHECK((Stub_LastObserverInput.appliedVoltageAlphaQ15 == 31) &&
               (Stub_LastObserverInput.appliedVoltageBetaQ15 == -32));
    TEST_CHECK((Stub_LastObserverInput.currentSampleValid == TRUE) &&
               (Stub_LastObserverInput.voltageSampleValid == TRUE) &&
               (Stub_LastObserverInput.generationValid == TRUE));
    TEST_CHECK((Stub_LastStartupInput.observer.angleQ16 == 100U) &&
               (Stub_LastStartupInput.observer.speedAngleCounts == 64));
    TEST_CHECK((Stub_LastStartupInput.observer.numericValid == TRUE) &&
               (Stub_LastStartupInput.observer.angleValid == TRUE) &&
               (Stub_LastStartupInput.observer.speedValid == TRUE) &&
               (Stub_LastStartupInput.observer.varianceValid == TRUE) &&
               (Stub_LastStartupInput.observer.bemfValid == TRUE) &&
               (Stub_LastStartupInput.observer.currentValid == TRUE) &&
               (Stub_LastStartupInput.observer.directionValid == TRUE));
    TEST_CHECK((Stub_LastStartupInput.generationValid == TRUE) &&
               (Stub_LastStartupInput.safetyValid == TRUE) &&
               (Stub_LastStartupInput.commandContinueValid == TRUE) &&
               (Stub_LastStartupInput.stopRequested == FALSE));
    TEST_CHECK((Stub_LastDError == 100) && (Stub_LastQError == 200));
    TEST_CHECK(result.controlValid == TRUE);

    for (failureStage = 1U; failureStage <= 9U; failureStage++)
    {
        MotorControlFastLoop_FailureType firstFailure;
        uint32 expectedStopIndex;
        uint32 index;

        Stub_ClearControls();
        TEST_CHECK(Test_InitBegin(&object, &config) == 0);
        Stub_SetStages(allStages, 9U);
        Stub_FailStage = failureStage;
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_FAILED);
        TEST_CHECK(result.controlValid == FALSE);
        TEST_CHECK(result.stopRequired == FALSE);
        expectedStopIndex = 0U;
        for (index = 0U; index < 9U; index++)
        {
            if (allStages[index] == failureStage)
            {
                expectedStopIndex = index + 1U;
            }
        }
        TEST_CHECK((expectedStopIndex != 0U) &&
                   (Stub_StageIndex == expectedStopIndex) &&
                   (Stub_OrderError == 0U));
        if (failureStage == 2U)
        {
            TEST_CHECK(object.failure ==
                       MOTOR_CONTROL_FAST_LOOP_FAILURE_OBSERVER);
        }
        else if (failureStage == 3U)
        {
            TEST_CHECK(object.failure ==
                       MOTOR_CONTROL_FAST_LOOP_FAILURE_STARTUP);
        }
        else if (failureStage == 9U)
        {
            TEST_CHECK(object.failure ==
                       MOTOR_CONTROL_FAST_LOOP_FAILURE_NUMERIC);
        }
        else
        {
            TEST_CHECK(object.failure ==
                       MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM);
        }
        firstFailure = object.failure;
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK((Stub_StageIndex == expectedStopIndex) &&
                   (object.failure == firstFailure));
        TEST_CHECK((result.controlValid == FALSE) &&
                   (result.readyForRun == FALSE) &&
                   (result.stopRequired == FALSE) &&
                   (result.failure == firstFailure));
    }
    return 0;
}

STATIC int Test_ModeAndPrecedence(void)
{
    static const uint32 startupStages[] =
        {1U, 2U, 3U, 9U, 4U, 5U, 6U, 7U, 8U};
    static const uint32 runStages[] =
        {1U, 2U, 9U, 4U, 5U, 6U, 7U, 8U};
    static const uint32 prefixStages[] = {1U, 2U};
    static const uint32 startupPrefix[] = {1U, 2U, 3U};
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input;
    uint32 predicate;

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_StartupTransition = TRUE;
    Stub_SetStages(startupStages, 9U);
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.mode == MOTOR_CONTROL_FAST_LOOP_MODE_RUN);
    TEST_CHECK(result.mode == MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP);

    Stub_StartupTransition = FALSE;
    Stub_ObserverAngle = 444U;
    Stub_SetStages(runStages, 8U);
    input = Test_Input(2U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((Stub_OrderError == 0U) && (Stub_StageIndex == 8U));
    TEST_CHECK(result.mode == MOTOR_CONTROL_FAST_LOOP_MODE_RUN);
    TEST_CHECK(result.selectedAngleQ16 == 444U);
    TEST_CHECK(result.startup.phase ==
               MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM);

    Stub_SetStages(prefixStages, 2U);
    input = Test_Input(3U);
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((Stub_OrderError == 0U) && (Stub_StageIndex == 2U));
    TEST_CHECK((result.stopRequired == TRUE) &&
               (result.controlValid == FALSE));

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_SetStages(startupPrefix, 3U);
    input = Test_Input(1U);
    input.safetyValid = FALSE;
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_STARTUP);
    TEST_CHECK((Stub_LastStartupInput.generationValid == TRUE) &&
               (Stub_LastStartupInput.safetyValid == FALSE) &&
               (Stub_LastStartupInput.stopRequested == TRUE));

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_StartupTransition = TRUE;
    Stub_SetStages(startupStages, 9U);
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    Stub_StartupTransition = FALSE;
    Stub_SetStages(prefixStages, 2U);
    input = Test_Input(2U);
    input.safetyValid = FALSE;
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD);

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_StartupTransition = TRUE;
    Stub_SetStages(startupStages, 9U);
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    Stub_StartupTransition = FALSE;
    Stub_ObserverReliable = FALSE;
    Stub_SetStages(prefixStages, 2U);
    input = Test_Input(2U);
    input.safetyValid = FALSE;
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_OBSERVER);

    for (predicate = 1U; predicate <= 7U; predicate++)
    {
        Stub_ClearControls();
        TEST_CHECK(Test_InitBegin(&object, &config) == 0);
        Stub_StartupTransition = TRUE;
        Stub_SetStages(startupStages, 9U);
        input = Test_Input(1U);
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_OK);
        Stub_StartupTransition = FALSE;
        Stub_InvalidObserverPredicate = predicate;
        Stub_SetStages(prefixStages, 2U);
        input = Test_Input(2U);
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK(object.failure ==
                   MOTOR_CONTROL_FAST_LOOP_FAILURE_OBSERVER);
    }

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_SetStages(startupPrefix, 3U);
    input = Test_Input(1U);
    input.commandContinueValid = FALSE;
    TEST_CHECK(MotorControlFastLoop_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((result.stopRequired == TRUE) &&
               (result.controlValid == FALSE));
    TEST_CHECK((Stub_LastStartupInput.generationValid == TRUE) &&
               (Stub_LastStartupInput.safetyValid == TRUE) &&
               (Stub_LastStartupInput.commandContinueValid == FALSE) &&
               (Stub_LastStartupInput.stopRequested == FALSE));
    return 0;
}

STATIC int Test_NumericErrorClassification(void)
{
    static const uint32 prefixThroughPark[] = {1U, 2U, 3U, 9U, 4U};
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input = Test_Input(1U);

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_ForceExtremeDq = TRUE;
    Stub_SetStages(prefixThroughPark, 5U);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_NOT_OK);
    TEST_CHECK((Stub_OrderError == 0U) && (Stub_StageIndex == 5U));
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_NUMERIC);
    TEST_CHECK(result.controlValid == FALSE);
    return 0;
}

STATIC int Test_CommissioningCapPrecedenceEveryMode(void)
{
    static const uint32 startupStopStages[] = {1U, 2U, 3U};
    static const uint32 criticalPrefixStages[] = {1U, 2U};
    static const uint32 startupStages[] =
        {1U, 2U, 3U, 9U, 4U, 5U, 6U, 7U, 8U};
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input;
    uint32 phase;

    for (phase = (uint32)MOTOR_CONTROL_STARTUP_PHASE_ALIGN;
         phase <= (uint32)MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM;
         phase++)
    {
        Stub_ClearControls();
        TEST_CHECK(Test_InitBegin(&object, &config) == 0);
        object.startup.phase = (MotorControlStartup_PhaseType)phase;
        object.startup.qRefQ15 = config.startup.finalQRefQ15;
        Stub_SetStages(startupStopStages, 3U);
        input = Test_Input(1U);
        input.stopRequested = TRUE;
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_OK);
        TEST_CHECK((result.stopRequired == TRUE) &&
                   (object.lifecycle ==
                    MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED));
        TEST_CHECK((Stub_OrderError == 0U) && (Stub_StageIndex == 3U));

        Stub_ClearControls();
        TEST_CHECK(Test_InitBegin(&object, &config) == 0);
        object.startup.phase = (MotorControlStartup_PhaseType)phase;
        object.startup.qRefQ15 =
            config.startup.absoluteCurrentCapQ15 + 1;
        Stub_SetStages(criticalPrefixStages, 2U);
        input = Test_Input(1U);
        input.stopRequested = TRUE;
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD);
        TEST_CHECK((result.stopRequired == FALSE) &&
                   (Stub_OrderError == 0U) && (Stub_StageIndex == 2U));
    }

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    object.startup.qRefQ15 = 100;
    object.config.startup.absoluteCurrentCapQ15 = 150;
    object.startup.config.absoluteCurrentCapQ15 = 150;
    Stub_SetStages(startupStopStages, 3U);
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD);
    TEST_CHECK((Stub_OrderError == 0U) && (Stub_StageIndex == 3U));
    TEST_CHECK(Stub_PiStepCall == 0U);

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_StartupTransition = TRUE;
    Stub_SetStages(startupStages, 9U);
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_OK);
    Stub_StartupTransition = FALSE;
    Stub_SetStages(criticalPrefixStages, 2U);
    input = Test_Input(2U);
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_OK);
    TEST_CHECK((result.stopRequired == TRUE) &&
               (Stub_OrderError == 0U) && (Stub_StageIndex == 2U));

    Stub_ClearControls();
    TEST_CHECK(Test_InitBegin(&object, &config) == 0);
    Stub_StartupTransition = TRUE;
    Stub_SetStages(startupStages, 9U);
    input = Test_Input(1U);
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_OK);
    Stub_StartupTransition = FALSE;
    object.config.startup.finalQRefQ15 =
        object.config.startup.absoluteCurrentCapQ15 + 1;
    object.startup.config.finalQRefQ15 =
        object.config.startup.finalQRefQ15;
    object.runQRefQ15 = object.config.startup.finalQRefQ15;
    Stub_SetStages(criticalPrefixStages, 2U);
    input = Test_Input(2U);
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlFastLoop_Step(
                   &object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD);
    TEST_CHECK((result.stopRequired == FALSE) &&
               (Stub_OrderError == 0U) && (Stub_StageIndex == 2U));
    return 0;
}

STATIC int Test_TransactionalOperations(void)
{
    static const uint32 startupStages[] =
        {1U, 2U, 3U, 9U, 4U, 5U, 6U, 7U, 8U};
    static const uint32 prefixStages[] = {1U, 2U};
    MotorControlFastLoop_ConfigType config = Test_Config();
    MotorControlFastLoop_Type object;
    MotorControlFastLoop_ResultType result;
    MotorControlFastLoop_StepInputType input;
    uint32 failure;

    for (failure = 1U; failure <= 4U; failure++)
    {
        Stub_ClearControls();
        Stub_InitFailStage = failure;
        (void)memset(&object, 0xA5, sizeof(object));
        TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_NOT_OK);
        TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_UNINITIALIZED);
        TEST_CHECK(object.failure ==
                   MOTOR_CONTROL_FAST_LOOP_FAILURE_CONFIGURATION);
    }

    for (failure = 1U; failure <= 4U; failure++)
    {
        Stub_ClearControls();
        Stub_PiInitCall = 0U;
        TEST_CHECK(MotorControlFastLoop_Init(&object, &config) == E_OK);
        Stub_BeginFailStage = failure;
        Stub_PiResetCall = 0U;
        TEST_CHECK(MotorControlFastLoop_Begin(&object, 1, 1U) == E_NOT_OK);
        TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_FAILED);
        TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL);
        TEST_CHECK(object.observer.lifecycle == MOTOR_CONTROL_OBSERVER_IDLE);
        TEST_CHECK(object.startup.phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE);
    }

    for (failure = 1U; failure <= 4U; failure++)
    {
        Stub_ClearControls();
        TEST_CHECK(Test_InitBegin(&object, &config) == 0);
        Stub_StartupTransition = TRUE;
        Stub_SetStages(startupStages, 9U);
        input = Test_Input(1U);
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_OK);
        Stub_StartupTransition = FALSE;
        Stub_SetStages(prefixStages, 2U);
        input = Test_Input(2U);
        input.stopRequested = TRUE;
        TEST_CHECK(MotorControlFastLoop_Step(
                       &object, &input, &result) == E_OK);
        Stub_ResetFailStage = failure;
        Stub_PiResetCall = 0U;
        TEST_CHECK(MotorControlFastLoop_ResetFromReady(&object) == E_NOT_OK);
        TEST_CHECK(object.lifecycle == MOTOR_CONTROL_FAST_LOOP_FAILED);
        TEST_CHECK(object.failure == MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL);
        TEST_CHECK(object.observer.lifecycle == MOTOR_CONTROL_OBSERVER_ACTIVE);
        TEST_CHECK(object.startup.phase ==
                   MOTOR_CONTROL_STARTUP_PHASE_COMPLETE);
    }
    return 0;
}

int main(void)
{
    TEST_CHECK(Test_MappingAndStageFailures() == 0);
    TEST_CHECK(Test_ModeAndPrecedence() == 0);
    TEST_CHECK(Test_NumericErrorClassification() == 0);
    TEST_CHECK(Test_CommissioningCapPrecedenceEveryMode() == 0);
    TEST_CHECK(Test_TransactionalOperations() == 0);

    (void)printf("PASS: MotorControl FastLoop Step-8C injection matrix\n");
    return 0;
}
