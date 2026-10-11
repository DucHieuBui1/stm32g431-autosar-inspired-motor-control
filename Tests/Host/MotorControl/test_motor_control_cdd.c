#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "Adc.h"
#include "MotorControlCdd.h"
#include "MotorControlCdd_Cfg.h"
#include "StateManager.h"
#include "stm32g431xx.h"

ADC_TypeDef MotorHost_Adc1;
GPIO_TypeDef MotorHost_GpioA;
GPIO_TypeDef MotorHost_GpioB;
TIM_TypeDef MotorHost_Tim1;
RCC_TypeDef MotorHost_Rcc;

static boolean Test_CommonValid;
static uint32_t Test_CalibrationClearObservation;
static uint32_t Test_ReadySetObservation;
static uint32_t Test_NvicPriority[3];
static boolean Test_NvicEnabled[3];
static uint32_t Test_NvicClearCount[3];
static boolean Test_OverrideNextGeneration;
static MotorControlCdd_GenerationType Test_NextGenerationOverride;
static boolean Test_CommitCrossesUpdate;
static boolean Test_ReadWhileWriting;
static Std_ReturnType Test_ReadWhileWritingResult;
static boolean Test_PublishWhileReading;
static uint32 Test_PublishWhileReadingGeneration;
static boolean Test_MailboxStateMismatch;
static boolean Test_MotorConfigApproved;
static boolean Test_MotorConfigCorrupt;
static uint32 Test_ControlApprovalMask;
static uint32 Test_StartPostConfigFault;
static boolean Test_StatusAfterClear;
static boolean Test_HoldCalibrationStop;
static boolean Test_MonitorCalibrationW1c;
static boolean Test_CalibrationW1cTouchedEoc;
static uint32 Test_CriticalFaultMask;
static Std_ReturnType Test_FastLoopInitReturn;
static Std_ReturnType Test_FastLoopBeginReturn;
static Std_ReturnType Test_FastLoopStepReturn;
static uint32 Test_FastLoopInitCount;
static uint32 Test_FastLoopBeginCount;
static uint32 Test_FastLoopStepCount;
static MotorControlFastLoop_StepInputType Test_LastFastLoopInput;
static MotorControlFastLoop_ResultType Test_FastLoopStepResult;
static uint32 Test_ControlStagingInterleaveStage;
static boolean Test_ControlStagingClaimResult;
static boolean Test_ControlStagingDuplicateResult;
static MotorControlCdd_ControlRequestType Test_InterleaveControlRequest;
static boolean Test_FastSafetyConsumeWhileWriting;
static boolean Test_FastSafetyPublishWhileReading;
static Std_ReturnType Test_FastSafetyPublishWhileReadingResult;
static MotorControlCdd_FastSafetySnapshotType Test_InterleaveFastSafety;
static void Test_MotorConfigHook(
    MotorControlCdd_CalibrationConfigType *config);
static void Test_ControlConfigHook(
    MotorControlCdd_ControlConfigType *config);
static void Test_ControlStagingHook(uint32 stage);
static void Test_FastSafetyHook(uint32 stage, uint32 slot);

static uint32_t Test_IrqIndex(IRQn_Type irq)
{
    if (irq == TIM1_BRK_TIM15_IRQn)
    {
        return 0U;
    }
    if (irq == ADC1_2_IRQn)
    {
        return 1U;
    }
    return 2U;
}

void MotorHost_NvicSetPriority(IRQn_Type irq, uint32_t priority)
{
    Test_NvicPriority[Test_IrqIndex(irq)] = priority;
}

void MotorHost_NvicClearPending(IRQn_Type irq)
{
    Test_NvicClearCount[Test_IrqIndex(irq)]++;
}

void MotorHost_NvicEnable(IRQn_Type irq)
{
    Test_NvicEnabled[Test_IrqIndex(irq)] = TRUE;
}

void MotorHost_WaitHook(uint32_t stage, uint32_t observation)
{
    if ((stage == 1U) && (observation == Test_CalibrationClearObservation))
    {
        ADC1->CR &= ~ADC_CR_ADCAL;
    }
    if ((stage == 2U) && (observation == Test_ReadySetObservation))
    {
        ADC1->ISR |= ADC_ISR_ADRDY;
    }
}

void MotorHost_AdcStatusClearHook(uint32_t mask)
{
    if ((Test_MonitorCalibrationW1c == TRUE) &&
        ((mask & ADC_ISR_EOC) != 0U))
    {
        Test_CalibrationW1cTouchedEoc = TRUE;
    }
    ADC1->ISR &= ~mask;
    if (Test_StatusAfterClear == TRUE)
    {
        ADC1->ISR |= ADC_ISR_EOC;
        Test_StatusAfterClear = FALSE;
    }
}

void MotorHost_TimStatusClearHook(uint32_t mask)
{
    (void)mask;
    TIM1->SR = 0U;
}

void MotorHost_CalibrationDrReadHook(void)
{
    ADC1->ISR &= ~ADC_ISR_EOC;
}

void MotorHost_CalibrationControlHook(void)
{
    if (((ADC1->CR & ADC_CR_ADSTP) != 0U) &&
        (Test_HoldCalibrationStop != TRUE))
    {
        ADC1->CR &= ~(ADC_CR_ADSTART | ADC_CR_ADSTP);
    }
}

Std_ReturnType Adc_CheckCommonConfiguration(void)
{
    return (Test_CommonValid == TRUE) ? E_OK : E_NOT_OK;
}

void StateManager_NotifyCriticalFault(uint32 CriticalClassMask)
{
    Test_CriticalFaultMask |= CriticalClassMask;
}

#define MOTOR_CONTROL_CDD_TEST_CALIBRATION_DR_READ_HOOK() \
    MotorHost_CalibrationDrReadHook()
#define MOTOR_CONTROL_CDD_TEST_CALIBRATION_CONTROL_HOOK() \
    MotorHost_CalibrationControlHook()
#define MOTOR_CONTROL_CDD_CFG_TEST_HOOK(Config) Test_MotorConfigHook(Config)
#define MOTOR_CONTROL_CDD_CONTROL_CFG_TEST_HOOK(Config) \
    Test_ControlConfigHook(Config)
#define MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK(Stage) \
    Test_ControlStagingHook(Stage)
#define MOTOR_CONTROL_CDD_TEST_FAST_SAFETY_HOOK(Stage, Slot) \
    Test_FastSafetyHook((Stage), (Slot))
#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlCdd_Cfg.c"
#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlCdd.c"

Std_ReturnType MotorControlFastLoop_Init(
    MotorControlFastLoop_Type *Object,
    const MotorControlFastLoop_ConfigType *Config)
{
    Test_FastLoopInitCount++;
    if ((Object == NULL_PTR) || (Config == NULL_PTR) ||
        (Test_FastLoopInitReturn != E_OK))
    {
        return E_NOT_OK;
    }
    *Object = (MotorControlFastLoop_Type){ 0 };
    Object->config = *Config;
    Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_IDLE;
    return E_OK;
}

Std_ReturnType MotorControlFastLoop_Begin(
    MotorControlFastLoop_Type *Object,
    sint32 ExpectedDirection,
    uint32 AlreadyConsumedStartingCycles)
{
    Test_FastLoopBeginCount++;
    if ((Object == NULL_PTR) ||
        ((ExpectedDirection != -1) && (ExpectedDirection != 1)) ||
        (AlreadyConsumedStartingCycles == 0U) ||
        (Test_FastLoopBeginReturn != E_OK))
    {
        return E_NOT_OK;
    }
    Object->expectedDirection = ExpectedDirection;
    Object->lifecycle = MOTOR_CONTROL_FAST_LOOP_ACTIVE;
    Object->mode = MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP;
    Object->generationEpochValid = TRUE;
    return E_OK;
}

Std_ReturnType MotorControlFastLoop_Step(
    MotorControlFastLoop_Type *Object,
    const MotorControlFastLoop_StepInputType *Input,
    MotorControlFastLoop_ResultType *Result)
{
    Test_FastLoopStepCount++;
    if ((Object == NULL_PTR) || (Input == NULL_PTR) ||
        (Result == NULL_PTR) || (Test_FastLoopStepReturn != E_OK))
    {
        return E_NOT_OK;
    }
    Test_LastFastLoopInput = *Input;
    *Result = Test_FastLoopStepResult;
    return E_OK;
}

static void Test_ControlStagingHook(uint32 stage)
{
    if (stage == Test_ControlStagingInterleaveStage)
    {
        Test_ControlStagingInterleaveStage = 0U;
        if ((stage == MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_WRITING) ||
            (stage == MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READY) ||
            (stage == MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_PUBLISHED))
        {
            Test_ControlStagingClaimResult =
                MotorControlCdd_ClaimStagedControl(1U);
        }
        else if (stage == MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READING)
        {
            Test_ControlStagingDuplicateResult =
                (MotorControlCdd_StageControl(
                     &Test_InterleaveControlRequest) == E_OK) ? TRUE : FALSE;
        }
        else
        {
            /* FREE is observed without a nested operation. */
        }
    }
}

static void Test_FastSafetyHook(uint32 stage, uint32 slot)
{
    (void)slot;
    if ((stage == MOTOR_CONTROL_CDD_TEST_MAILBOX_WRITING) &&
        (Test_FastSafetyConsumeWhileWriting == TRUE))
    {
        Test_FastSafetyConsumeWhileWriting = FALSE;
        MotorControlCdd_ConsumeNewestFastSafety();
    }
    if ((stage == MOTOR_CONTROL_CDD_TEST_MAILBOX_READING) &&
        (Test_FastSafetyPublishWhileReading == TRUE))
    {
        Test_FastSafetyPublishWhileReading = FALSE;
        Test_FastSafetyPublishWhileReadingResult =
            MotorControlCdd_PublishSafetySnapshot(
                &Test_InterleaveFastSafety);
    }
}

static void Test_MotorConfigHook(
    MotorControlCdd_CalibrationConfigType *config)
{
    uint32 phase;

    if (Test_MotorConfigApproved == TRUE)
    {
        config->measurementApproved = TRUE;
        config->identity.evidenceSha256[0] = 1U;
        for (phase = 0U;
             phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
             phase++)
        {
            config->phase[phase].absoluteRailMinimum = 100U;
            config->phase[phase].absoluteRailMaximum = 4000U;
            config->phase[phase].meanMinimum = 2000U;
            config->phase[phase].meanMaximum = 2200U;
            config->phase[phase].maximumPeakToPeak = 10U;
        }
    }
    if (Test_MotorConfigCorrupt == TRUE)
    {
        config->identity.schemaVersion = 0U;
    }
    switch (Test_StartPostConfigFault)
    {
        case 1U:
            ADC1->IER = ADC_IER_JEOSIE;
            break;
        case 2U:
            Test_StatusAfterClear = TRUE;
            break;
        case 3U:
            TIM1->CR1 |= TIM_CR1_CEN;
            break;
        case 4U:
            TIM1->CR2 |= TIM_CR2_MMS;
            break;
        case 5U:
            GPIOB->ODR |= GPIO_ODR_OD5;
            break;
        case 6U:
            TIM1->BDTR |= TIM_BDTR_MOE;
            break;
        default:
            break;
    }
}

static void Test_ControlConfigHook(
    MotorControlCdd_ControlConfigType *config)
{
    if (Test_ControlApprovalMask != 0U)
    {
        *config = (MotorControlCdd_ControlConfigType){ 0 };
        config->currentScaleApproved =
            ((Test_ControlApprovalMask & 1U) != 0U) ? TRUE : FALSE;
        config->controlParametersApproved =
            ((Test_ControlApprovalMask & 2U) != 0U) ? TRUE : FALSE;
        config->samplingTimingApproved =
            ((Test_ControlApprovalMask & 4U) != 0U) ? TRUE : FALSE;
        config->currentBaseMa = 4096U;
        config->rawMilliampNumerator = 1U;
        config->rawMilliampDenominator = 1U;
        config->rawToQ15Multiplier = 16U;
        config->rawToQ15Shift = 1U;
        config->rawScaleMaximumErrorMicroamp = 500U;
        config->alignQReferenceMagnitudeMa = 100U;
        config->fastLoop.observer.phaseProportionalQ15 = 32767;
        config->fastLoop.observer.phaseIntegralQ15 = 32767;
        config->fastLoop.observer.maxAbsCurrentQ15 = 32767;
        config->fastLoop.observer.maxAbsBemfQ15 = 32767;
        config->fastLoop.observer.maxAbsSpeedAngleCounts = 2048U;
        config->fastLoop.observer.maxSpeedDeltaAngleCounts = 2048U;
        config->fastLoop.observer.minBemfMagnitudeSqQ30 = 1U;
        config->fastLoop.observer.maxBemfMagnitudeSqQ30 = 2147352578ULL;
        config->fastLoop.observer.maxCurrentResidualMagnitudeSqQ30 =
            8589672450ULL;
        config->fastLoop.observer.speedWindowLength = 2U;
        config->fastLoop.observer.maxSpeedVarianceNumerator = 16777216ULL;
        config->fastLoop.observer.reliabilityWarmupSamples = 2U;
        config->fastLoop.observer.maxConsecutiveUnreliableSamples = 4U;
        config->fastLoop.startup.alignCycles = 2U;
        config->fastLoop.startup.openLoopRampCycles = 4U;
        config->fastLoop.startup.qualificationCycles = 4U;
        config->fastLoop.startup.handoverCycles = 2U;
        config->fastLoop.startup.outputConfirmCycles = 2U;
        config->fastLoop.startup.finalOpenLoopSpeedAngleCounts = 64;
        config->fastLoop.startup.qualificationRequiredCycles = 2U;
        config->fastLoop.startup.minimumObservableSpeedAngleCounts = 1U;
        config->fastLoop.startup.qualificationMaxAngleErrorQ16 = 32U;
        config->fastLoop.startup.qualificationMaxSpeedErrorAngleCounts = 128U;
        config->fastLoop.startup.handoverMaxAngleCorrectionQ16 = 128U;
        config->fastLoop.startup.handoverMaxSpeedSlewAngleCounts = 128U;
        config->fastLoop.startup.finalMaxAngleErrorQ16 = 128U;
        config->fastLoop.startup.finalMaxSpeedErrorAngleCounts = 32U;
        config->fastLoop.dPi.kpQ15 = 8192;
        config->fastLoop.dPi.lowerQ15 = -32768;
        config->fastLoop.dPi.upperQ15 = 32767;
        config->fastLoop.qPi = config->fastLoop.dPi;
        config->fastLoop.trigNormToleranceQ30 =
            MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30;
        config->switchingBlankingCounts = 20U;
        config->triggerLatencyCounts = 10U;
        config->apertureCounts = 4U;
        config->rankSeparationCounts = 8U;
        config->triggerArmMinimum = 32U;
        config->commitMarginCounts = 64U;
        config->updateArmLimitCounts = 32U;
        config->commitCutoffCounts = 512U;
        config->appliedVoltageMaximumErrorQ15 = 24U;
        config->identity.schemaVersion =
            MOTOR_CONTROL_CDD_CONTROL_SCHEMA_VERSION;
        config->identity.boardProfileCode =
            SC1_MEASUREMENT_BOARD_PROFILE_CODE;
        config->identity.firmwareIdentity = 1U;
        config->identity.evidenceSha256[0] = 1U;
        config->identity.configurationSha256[0] = 2U;
    }
}

void MotorHost_NextGenerationHook(void *generation)
{
    if (Test_OverrideNextGeneration == TRUE)
    {
        *((MotorControlCdd_GenerationType *)generation) =
            Test_NextGenerationOverride;
        Test_OverrideNextGeneration = FALSE;
    }
}

void MotorHost_CommitHook(void)
{
    if (Test_CommitCrossesUpdate == TRUE)
    {
        TIM1->SR |= TIM_SR_UIF;
        Test_CommitCrossesUpdate = FALSE;
    }
}

void MotorHost_MailboxHook(uint32_t stage, uint32_t slot)
{
    if (MotorControlCdd_CurrentSampleMailbox[slot].state != stage)
    {
        Test_MailboxStateMismatch = TRUE;
    }
    if ((stage == MOTOR_CONTROL_CDD_TEST_MAILBOX_WRITING) &&
        (Test_ReadWhileWriting == TRUE))
    {
        MotorControlCdd_CurrentSampleType sample;

        Test_ReadWhileWriting = FALSE;
        Test_ReadWhileWritingResult =
            MotorControlCdd_GetCurrentSample(&sample);
    }
    else if ((stage == MOTOR_CONTROL_CDD_TEST_MAILBOX_READING) &&
             (Test_PublishWhileReading == TRUE))
    {
        uint32 generationId = Test_PublishWhileReadingGeneration;

        Test_PublishWhileReading = FALSE;
        MotorControlCdd_PublishCurrentSample(
            21, 22, -43, generationId, TRUE);
    }
    else
    {
        /* No injected interleaving. */
    }
}

#define TEST_NEVER_OBSERVATION UINT_MAX

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
    (void)memset(&MotorHost_Adc1, 0, sizeof(MotorHost_Adc1));
    (void)memset(&MotorHost_GpioA, 0, sizeof(MotorHost_GpioA));
    (void)memset(&MotorHost_GpioB, 0, sizeof(MotorHost_GpioB));
    (void)memset(&MotorHost_Tim1, 0, sizeof(MotorHost_Tim1));
    (void)memset(&MotorHost_Rcc, 0, sizeof(MotorHost_Rcc));
    (void)memset(Test_NvicPriority, 0xFF, sizeof(Test_NvicPriority));
    (void)memset(Test_NvicEnabled, 0, sizeof(Test_NvicEnabled));
    (void)memset(Test_NvicClearCount, 0, sizeof(Test_NvicClearCount));

    RCC->AHB2ENR = MOTOR_CONTROL_CDD_REQUIRED_AHB2_GATES;
    RCC->APB2ENR = RCC_APB2ENR_TIM1EN;
    RCC->CCIPR = RCC_CCIPR_ADC12SEL_0;
    ADC1->CR = ADC_CR_DEEPPWD;
    Test_CommonValid = TRUE;
    Test_FastLoopInitReturn = E_OK;
    Test_FastLoopBeginReturn = E_OK;
    Test_FastLoopStepReturn = E_OK;
    Test_FastLoopInitCount = 0U;
    Test_FastLoopBeginCount = 0U;
    Test_FastLoopStepCount = 0U;
    Test_LastFastLoopInput = (MotorControlFastLoop_StepInputType){ 0 };
    Test_FastLoopStepResult = (MotorControlFastLoop_ResultType){ 0 };
    Test_ControlStagingInterleaveStage = 0U;
    Test_ControlStagingClaimResult = FALSE;
    Test_ControlStagingDuplicateResult = FALSE;
    Test_InterleaveControlRequest =
        (MotorControlCdd_ControlRequestType){ 0 };
    Test_FastSafetyConsumeWhileWriting = FALSE;
    Test_FastSafetyPublishWhileReading = FALSE;
    Test_FastSafetyPublishWhileReadingResult = E_NOT_OK;
    Test_InterleaveFastSafety =
        (MotorControlCdd_FastSafetySnapshotType){ 0 };
    Test_CalibrationClearObservation = 0U;
    Test_ReadySetObservation = 0U;

    MotorControlCdd_EarlySafeInitialized = FALSE;
    MotorControlCdd_State = MOTOR_CONTROL_CDD_UNINITIALIZED;
    MotorControlCdd_BreakEvent = 0U;
    MotorControlCdd_UpdateEvent = 0U;
    MotorControlCdd_AdcEvent = 0U;
    MotorControlCdd_CalibrationEvent = 0U;
    (void)memset(&MotorControlCdd_CurrentOffsets, 0,
                 sizeof(MotorControlCdd_CurrentOffsets));
    MotorControlCdd_CurrentOffsetsValid = FALSE;
    (void)memset(&MotorControlCdd_PreparedGeneration, 0,
                 sizeof(MotorControlCdd_PreparedGeneration));
    (void)memset(&MotorControlCdd_QueuedGeneration, 0,
                 sizeof(MotorControlCdd_QueuedGeneration));
    MotorControlCdd_PreparedGenerationValid = FALSE;
    MotorControlCdd_QueuedGenerationValid = FALSE;
    MotorControlCdd_DiscardNextSample = TRUE;
    MotorControlCdd_ConsumedGenerationId = 0U;
    (void)memset(MotorControlCdd_CurrentSampleMailbox, 0,
                 sizeof(MotorControlCdd_CurrentSampleMailbox));
    MotorControlCdd_SampleMailboxWriteCursor = 0U;
    MotorControlCdd_SampleDropCount = 0U;
    MotorControlCdd_CalibrationState = MOTOR_CONTROL_CDD_CALIBRATION_IDLE;
    MotorControlCdd_CalibrationFailure =
        MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE;
    MotorControlCdd_ActiveCalibrationConfig =
        (MotorControlCdd_CalibrationConfigType){ 0 };
    MotorControlCdd_CalibrationBaseline =
        (MotorControlCdd_CalibrationBaselineType){ 0 };
    MotorControlCdd_ResetCalibrationData();
    Test_OverrideNextGeneration = FALSE;
    (void)memset(&Test_NextGenerationOverride, 0,
                 sizeof(Test_NextGenerationOverride));
    Test_CommitCrossesUpdate = FALSE;
    Test_ReadWhileWriting = FALSE;
    Test_ReadWhileWritingResult = E_OK;
    Test_PublishWhileReading = FALSE;
    Test_PublishWhileReadingGeneration = 0U;
    Test_MailboxStateMismatch = FALSE;
    Test_MotorConfigApproved = FALSE;
    Test_MotorConfigCorrupt = FALSE;
    Test_ControlApprovalMask = 0U;
    Test_StartPostConfigFault = 0U;
    Test_StatusAfterClear = FALSE;
    Test_HoldCalibrationStop = FALSE;
    Test_MonitorCalibrationW1c = FALSE;
    Test_CalibrationW1cTouchedEoc = FALSE;
    Test_CriticalFaultMask = 0U;
}

static int Test_RunSuccessfulInit(void)
{
    TEST_CHECK(MotorControlCdd_EarlySafeInit() == E_OK);
    TEST_CHECK(MotorControlCdd_Init() == E_OK);
    return 0;
}

static int Test_StaticInitIsSafeAndComplete(void)
{
    uint32_t gpioAUnrelatedMode = (2UL << 12U);
    uint32_t gpioAUnrelatedAf = (9UL << 12U);
    uint32_t gpioBUnrelatedMode = (1UL << 16U);

    Test_ResetFixture();
    GPIOA->MODER = gpioAUnrelatedMode;
    GPIOA->AFR[1] = gpioAUnrelatedAf;
    GPIOB->MODER = gpioBUnrelatedMode;

    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_CheckStaticConfiguration() == E_OK);
    TEST_CHECK((GPIOA->MODER & gpioAUnrelatedMode) == gpioAUnrelatedMode);
    TEST_CHECK((GPIOA->AFR[1] & gpioAUnrelatedAf) == gpioAUnrelatedAf);
    TEST_CHECK((GPIOB->MODER & gpioBUnrelatedMode) == gpioBUnrelatedMode);
    TEST_CHECK((GPIOB->ODR & MOTOR_CONTROL_CDD_SAFE_PIN_MASK) == 0U);
    TEST_CHECK((TIM1->CR1 & TIM_CR1_CEN) == 0U);
    TEST_CHECK((TIM1->CR2 & TIM_CR2_MMS) == 0U);
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_AOE) == 0U);
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_BKE) != 0U);
    TEST_CHECK(TIM1->DIER == TIM_DIER_BIE);
    TEST_CHECK(ADC1->IER == 0U);
    TEST_CHECK(ADC1->CFGR == ADC_CFGR_JQM);
    TEST_CHECK(ADC1->JSQR == 0U);
    TEST_CHECK((ADC1->CR & ADC_CR_ADEN) != 0U);
    TEST_CHECK(Test_NvicPriority[0] == 0U);
    TEST_CHECK(Test_NvicPriority[1] == 1U);
    TEST_CHECK(Test_NvicPriority[2] == 2U);
    TEST_CHECK(Test_NvicEnabled[0] == TRUE);
    TEST_CHECK(Test_NvicEnabled[1] == TRUE);
    TEST_CHECK(Test_NvicEnabled[2] == TRUE);
    TEST_CHECK(Test_NvicClearCount[0] == 1U);
    TEST_CHECK(Test_NvicClearCount[1] == 1U);
    TEST_CHECK(Test_NvicClearCount[2] == 1U);
    return 0;
}

static int Test_InitIsOneShotAndChecksPreconditions(void)
{
    ADC_TypeDef adcSnapshot;
    TIM_TypeDef timerSnapshot;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    adcSnapshot = *ADC1;
    timerSnapshot = *TIM1;
    TEST_CHECK(MotorControlCdd_Init() == E_NOT_OK);
    TEST_CHECK(memcmp(&adcSnapshot, ADC1, sizeof(adcSnapshot)) == 0);
    TEST_CHECK(memcmp(&timerSnapshot, TIM1, sizeof(timerSnapshot)) == 0);

    Test_ResetFixture();
    RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
    TEST_CHECK(MotorControlCdd_EarlySafeInit() == E_OK);
    TEST_CHECK(MotorControlCdd_Init() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);

    Test_ResetFixture();
    Test_CommonValid = FALSE;
    TEST_CHECK(MotorControlCdd_EarlySafeInit() == E_OK);
    TEST_CHECK(MotorControlCdd_Init() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(MotorControlCdd_Init() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(MotorControlCdd_EarlySafeInit() == E_OK);
    ADC1->CR |= ADC_CR_ADEN;
    TEST_CHECK(MotorControlCdd_Init() == E_NOT_OK);
    return 0;
}

static int Test_AdcWaitsAreBounded(void)
{
    Test_ResetFixture();
    Test_CalibrationClearObservation =
        MOTOR_CONTROL_CDD_ADC_TIMEOUT_OBSERVATIONS - 1U;
    Test_ReadySetObservation =
        MOTOR_CONTROL_CDD_ADC_TIMEOUT_OBSERVATIONS - 1U;
    TEST_CHECK(Test_RunSuccessfulInit() == 0);

    Test_ResetFixture();
    Test_CalibrationClearObservation = TEST_NEVER_OBSERVATION;
    TEST_CHECK(MotorControlCdd_EarlySafeInit() == E_OK);
    TEST_CHECK(MotorControlCdd_Init() == E_NOT_OK);

    Test_ResetFixture();
    Test_ReadySetObservation = TEST_NEVER_OBSERVATION;
    TEST_CHECK(MotorControlCdd_EarlySafeInit() == E_OK);
    TEST_CHECK(MotorControlCdd_Init() == E_NOT_OK);
    return 0;
}

static int Test_StaticReadbackDetectsCorruption(void)
{
    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    GPIOA->AFR[1] ^= 1U;
    TEST_CHECK(MotorControlCdd_CheckStaticConfiguration() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TIM1->BDTR |= TIM_BDTR_MOE;
    TEST_CHECK(MotorControlCdd_CheckStaticConfiguration() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    ADC1->CFGR = 0U;
    TEST_CHECK(MotorControlCdd_CheckStaticConfiguration() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_CommonValid = FALSE;
    TEST_CHECK(MotorControlCdd_CheckStaticConfiguration() == E_NOT_OK);
    return 0;
}

static int Test_InterruptsFailSafeAndPublishOwnedEvents(void)
{
    MotorControlCdd_FaultEventsType events;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR2 |= 0x70U;
    TIM1->SR |= TIM_SR_BIF;
    TIM1_BRK_TIM15_IRQHandler();
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK((TIM1->CR2 & TIM_CR2_MMS) == 0U);
    TEST_CHECK((TIM1->DIER & (TIM_DIER_BIE | TIM_DIER_UIE)) == 0U);
    TEST_CHECK(GPIOB->BSRR == GPIO_BSRR_BR5);
    TEST_CHECK(MotorControlCdd_GetFaultEvents(&events) == E_OK);
    TEST_CHECK(events.breakEvent == 0x00010001U);
    TEST_CHECK(events.updateEvent == 0U);
    TEST_CHECK(events.adcEvent == 0U);

    TIM1->SR |= TIM_SR_BIF;
    TIM1_BRK_TIM15_IRQHandler();
    TEST_CHECK(MotorControlCdd_GetFaultEvents(&events) == E_OK);
    TEST_CHECK(events.breakEvent == 0x00020001U);

    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->SR |= TIM_SR_UIF;
    TIM1_UP_TIM16_IRQHandler();
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK((TIM1->DIER & TIM_DIER_UIE) == 0U);
    TEST_CHECK(MotorControlCdd_GetFaultEvents(&events) == E_OK);
    TEST_CHECK(events.updateEvent == 0x00010002U);

    TIM1->BDTR |= TIM_BDTR_MOE;
    ADC1->IER |= ADC_IER_JEOSIE;
    ADC1->ISR |= (ADC_ISR_JEOS | ADC_ISR_JQOVF);
    ADC1_2_IRQHandler();
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK((ADC1->IER & ADC_IER_JEOSIE) == 0U);
    TEST_CHECK((ADC1->ISR & (ADC_ISR_JEOS | ADC_ISR_JQOVF)) == 0U);
    TEST_CHECK(MotorControlCdd_GetFaultEvents(&events) == E_OK);
    TEST_CHECK(events.adcEvent == 0x00010004U);
    TEST_CHECK(MotorControlCdd_GetFaultEvents(NULL_PTR) == E_NOT_OK);
    return 0;
}

static MotorControlCdd_GenerationType Test_MakeGeneration(
    uint32 generationId,
    uint8 sector,
    MotorControlCdd_PhaseType rank1Phase,
    MotorControlCdd_PhaseType rank2Phase,
    MotorControlCdd_PhaseType reconstructedPhase,
    MotorControlCdd_TriggerEdgeType edge)
{
    MotorControlCdd_GenerationType generation = { 0 };

    generation.generationId = generationId;
    generation.ccr1 = (uint16)(1000U + generationId);
    generation.ccr2 = (uint16)(2000U + generationId);
    generation.ccr3 = (uint16)(3000U + generationId);
    generation.ccr4 = (uint16)(4000U - generationId);
    generation.sector = sector;
    generation.rank1Phase = rank1Phase;
    generation.rank2Phase = rank2Phase;
    generation.reconstructedPhase = reconstructedPhase;
    generation.triggerEdge = edge;
    generation.numericValid = TRUE;
    generation.windowValid = TRUE;
    generation.safetyValid = TRUE;
    return generation;
}

static void Test_OverrideNext(const MotorControlCdd_GenerationType *generation)
{
    Test_NextGenerationOverride = *generation;
    Test_OverrideNextGeneration = TRUE;
}

static void Test_TriggerUpdateAt(uint32 counter)
{
    TIM1->CNT = counter;
    TIM1->CR1 &= ~TIM_CR1_DIR;
    TIM1->SR |= TIM_SR_UIF;
    TIM1_UP_TIM16_IRQHandler();
}

static void Test_CompleteInjectedPair(uint16 firstRaw, uint16 secondRaw)
{
    ADC1->JDR1 = firstRaw;
    ADC1->JDR2 = secondRaw;
    ADC1->JSQR = 0U; /* Model JQM=1 end-empty hardware consumption. */
    ADC1->ISR |= ADC_ISR_JEOS;
    ADC1_2_IRQHandler();
}

static int Test_StartSamplingPipeline(void)
{
    MotorControlCdd_CurrentOffsetsType offsets = {2000U, 2100U, 2200U};

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_InstallCurrentOffsets(&offsets) == E_OK);
    MotorControlCdd_CalibrationState = MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE;
    TEST_CHECK(MotorControlCdd_StartSampling() == E_OK);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING);
    TEST_CHECK(MotorControlCdd_PreparedGenerationValid == TRUE);
    TEST_CHECK(MotorControlCdd_PreparedGeneration.generation.generationId == 1U);
    TEST_CHECK((TIM1->CR1 & TIM_CR1_CEN) != 0U);
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK((GPIOB->ODR & MOTOR_CONTROL_CDD_SAFE_PIN_MASK) == 0U);
    return 0;
}

typedef struct
{
    uint8 sector;
    MotorControlCdd_PhaseType rank1Phase;
    MotorControlCdd_PhaseType rank2Phase;
    MotorControlCdd_PhaseType reconstructedPhase;
    MotorControlCdd_TriggerEdgeType edge;
} Test_GenerationCaseType;

static int Test_GenerationPipelineAllContexts(void)
{
    static const Test_GenerationCaseType cases[] =
    {
        {1U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_RISING},
        {2U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_FALLING},
        {3U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_RISING},
        {4U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_FALLING},
        {5U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_RISING},
        {6U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_FALLING},
        {1U, MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_PHASE_C,
         MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_TRIGGER_RISING},
        {6U, MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_PHASE_C,
         MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_TRIGGER_FALLING},
        {2U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_C,
         MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_TRIGGER_RISING},
        {3U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_C,
         MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_TRIGGER_FALLING},
        {4U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_RISING},
        {5U, MOTOR_CONTROL_CDD_PHASE_A, MOTOR_CONTROL_CDD_PHASE_B,
         MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_TRIGGER_FALLING}
    };
    static const uint16 offsets[3] = {2000U, 2100U, 2200U};
    MotorControlCdd_CurrentSampleType sample;
    uint32 caseIndex;

    TEST_CHECK(Test_StartSamplingPipeline() == 0);

    for (caseIndex = 0U;
         caseIndex < (uint32)(sizeof(cases) / sizeof(cases[0]));
         caseIndex++)
    {
        uint32 generationId = caseIndex + 1U;
        const Test_GenerationCaseType *currentCase = &cases[caseIndex];
        uint32 expectedFirstChannel;
        uint32 expectedSecondChannel;
        uint16 raw[3] = {2050U, 2130U, 2180U};
        sint32 expected[3] = {0, 0, 0};

        TEST_CHECK(MotorControlCdd_PreparedGeneration.generation.sector ==
                   currentCase->sector);
        TEST_CHECK(MotorControlCdd_PreparedGeneration.generation.rank1Phase ==
                   currentCase->rank1Phase);
        TEST_CHECK(MotorControlCdd_PreparedGeneration.generation.rank2Phase ==
                   currentCase->rank2Phase);
        TEST_CHECK(
            MotorControlCdd_PreparedGeneration.generation.reconstructedPhase ==
            currentCase->reconstructedPhase);

        Test_TriggerUpdateAt(0U);
        TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING);
        TEST_CHECK(MotorControlCdd_QueuedGenerationValid == TRUE);
        TEST_CHECK((TIM1->CR2 & TIM_CR2_MMS) ==
                   MOTOR_CONTROL_CDD_TRGO_OC4REF_VALUE);
        TEST_CHECK(((ADC1->JSQR >> ADC_JSQR_JEXTEN_Pos) & 3U) ==
                   (uint32)currentCase->edge);

        if (currentCase->rank1Phase == MOTOR_CONTROL_CDD_PHASE_A)
        {
            expectedFirstChannel = MOTOR_CONTROL_CDD_ADC_CHANNEL_A;
        }
        else
        {
            expectedFirstChannel = MOTOR_CONTROL_CDD_ADC_CHANNEL_B;
        }
        if (currentCase->rank2Phase == MOTOR_CONTROL_CDD_PHASE_B)
        {
            expectedSecondChannel = MOTOR_CONTROL_CDD_ADC_CHANNEL_B;
        }
        else
        {
            expectedSecondChannel = MOTOR_CONTROL_CDD_ADC_CHANNEL_C;
        }
        TEST_CHECK(((ADC1->JSQR >> ADC_JSQR_JSQ1_Pos) & 31U) ==
                   expectedFirstChannel);
        TEST_CHECK(((ADC1->JSQR >> ADC_JSQR_JSQ2_Pos) & 31U) ==
                   expectedSecondChannel);

        if ((caseIndex + 1U) <
            (uint32)(sizeof(cases) / sizeof(cases[0])))
        {
            const Test_GenerationCaseType *nextCase = &cases[caseIndex + 1U];
            MotorControlCdd_GenerationType nextGeneration =
                Test_MakeGeneration(
                    generationId + 1U, nextCase->sector,
                    nextCase->rank1Phase, nextCase->rank2Phase,
                    nextCase->reconstructedPhase, nextCase->edge);
            Test_OverrideNext(&nextGeneration);
        }

        TIM1->CR1 &= ~TIM_CR1_DIR;
        TIM1->CNT = 1000U;
        Test_CompleteInjectedPair(
            raw[currentCase->rank1Phase], raw[currentCase->rank2Phase]);
        TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_OK);
        TEST_CHECK(sample.generationId == generationId);
        TEST_CHECK(sample.valid == ((generationId == 1U) ? FALSE : TRUE));
        expected[currentCase->rank1Phase] =
            (sint32)raw[currentCase->rank1Phase] -
            (sint32)offsets[currentCase->rank1Phase];
        expected[currentCase->rank2Phase] =
            (sint32)raw[currentCase->rank2Phase] -
            (sint32)offsets[currentCase->rank2Phase];
        expected[currentCase->reconstructedPhase] =
            -(expected[currentCase->rank1Phase] +
              expected[currentCase->rank2Phase]);
        TEST_CHECK(sample.phaseA == (sint16)expected[MOTOR_CONTROL_CDD_PHASE_A]);
        TEST_CHECK(sample.phaseB == (sint16)expected[MOTOR_CONTROL_CDD_PHASE_B]);
        TEST_CHECK(sample.phaseC == (sint16)expected[MOTOR_CONTROL_CDD_PHASE_C]);
        TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_NOT_OK);
    }

    TEST_CHECK(MotorControlCdd_StopSampling() == E_OK);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_STATIC_SAFE);
    TEST_CHECK((TIM1->CR1 & TIM_CR1_CEN) == 0U);
    TEST_CHECK((TIM1->CR2 & TIM_CR2_MMS) == 0U);
    TEST_CHECK(MotorControlCdd_CheckStaticConfiguration() == E_OK);
    TEST_CHECK(MotorControlCdd_GetCurrentSample(NULL_PTR) == E_NOT_OK);
    return 0;
}

static int Test_PipelineRejectsUnsafeOrLateWork(void)
{
    MotorControlCdd_FaultEventsType events;
    uint32 invalidCase;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_StartSampling() == E_NOT_OK);

    {
        MotorControlCdd_GenerationType generation =
            Test_MakeGeneration(
                1U, 1U, MOTOR_CONTROL_CDD_PHASE_A,
                MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_PHASE_C,
                MOTOR_CONTROL_CDD_TRIGGER_RISING);
        TEST_CHECK(MotorControlCdd_CommitPreparedGeneration(&generation) ==
                   E_NOT_OK);
        TEST_CHECK(MotorControlCdd_PreparedGenerationValid == FALSE);
        TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_STATIC_SAFE);
    }

    for (invalidCase = 0U; invalidCase < 17U; invalidCase++)
    {
        MotorControlCdd_GenerationType generation =
            Test_MakeGeneration(
                2U, 1U, MOTOR_CONTROL_CDD_PHASE_A,
                MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_PHASE_C,
                MOTOR_CONTROL_CDD_TRIGGER_RISING);

        TEST_CHECK(Test_StartSamplingPipeline() == 0);
        Test_TriggerUpdateAt(0U);
        switch (invalidCase)
        {
            case 0U: generation.generationId = 3U; break;
            case 1U: generation.ccr1 = 4251U; break;
            case 2U: generation.ccr2 = 4251U; break;
            case 3U: generation.ccr3 = 4251U; break;
            case 4U: generation.ccr4 = 0U; break;
            case 5U: generation.ccr4 = 4250U; break;
            case 6U: generation.sector = 0U; break;
            case 7U: generation.sector = 7U; break;
            case 8U: generation.triggerEdge =
                         (MotorControlCdd_TriggerEdgeType)0; break;
            case 9U: generation.rank1Phase =
                         (MotorControlCdd_PhaseType)3; break;
            case 10U: generation.rank2Phase = MOTOR_CONTROL_CDD_PHASE_A; break;
            case 11U: generation.reconstructedPhase =
                          MOTOR_CONTROL_CDD_PHASE_B; break;
            case 12U:
                generation.rank1Phase = MOTOR_CONTROL_CDD_PHASE_B;
                generation.rank2Phase = MOTOR_CONTROL_CDD_PHASE_C;
                generation.reconstructedPhase = MOTOR_CONTROL_CDD_PHASE_A;
                generation.sector = 2U;
                break;
            case 13U: generation.numericValid = FALSE; break;
            case 14U: generation.windowValid = FALSE; break;
            case 15U: generation.safetyValid = FALSE; break;
            default:
                generation.triggerEdge = (MotorControlCdd_TriggerEdgeType)3;
                break;
        }
        Test_OverrideNext(&generation);
        TIM1->CNT = 1000U;
        Test_CompleteInjectedPair(2050U, 2130U);
        TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);
        TEST_CHECK(MotorControlCdd_GetFaultEvents(&events) == E_OK);
        TEST_CHECK((events.adcEvent & MOTOR_CONTROL_CDD_EVENT_CAUSE_MASK) ==
                   MOTOR_CONTROL_CDD_EVENT_CAUSE_INVALID_GENERATION);
    }

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(0U);
    {
        MotorControlCdd_GenerationType generation =
            Test_MakeGeneration(
                2U, 2U, MOTOR_CONTROL_CDD_PHASE_A,
                MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_PHASE_B,
                MOTOR_CONTROL_CDD_TRIGGER_FALLING);
        Test_OverrideNext(&generation);
    }
    TIM1->CR1 |= TIM_CR1_DIR;
    TIM1->CNT = MOTOR_CONTROL_CDD_COMMIT_CUTOFF_COUNTS + 1U;
    Test_CompleteInjectedPair(2050U, 2130U);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING);
    TEST_CHECK(MotorControlCdd_PreparedGenerationValid == TRUE);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(0U);
    {
        MotorControlCdd_GenerationType generation =
            Test_MakeGeneration(
                2U, 2U, MOTOR_CONTROL_CDD_PHASE_A,
                MOTOR_CONTROL_CDD_PHASE_C, MOTOR_CONTROL_CDD_PHASE_B,
                MOTOR_CONTROL_CDD_TRIGGER_FALLING);
        Test_OverrideNext(&generation);
    }
    TIM1->CR1 |= TIM_CR1_DIR;
    TIM1->CNT = MOTOR_CONTROL_CDD_COMMIT_CUTOFF_COUNTS;
    Test_CompleteInjectedPair(2050U, 2130U);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);
    TEST_CHECK((TIM1->CR1 & TIM_CR1_CEN) == 0U);
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK(MotorControlCdd_GetFaultEvents(&events) == E_OK);
    TEST_CHECK((events.adcEvent & MOTOR_CONTROL_CDD_EVENT_CAUSE_MASK) ==
               MOTOR_CONTROL_CDD_EVENT_CAUSE_INVALID_GENERATION);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(0U);
    Test_CommitCrossesUpdate = TRUE;
    Test_CompleteInjectedPair(2050U, 2130U);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(MOTOR_CONTROL_CDD_UPDATE_ARM_LIMIT_COUNTS);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING);
    Test_CompleteInjectedPair(2050U, 2130U);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(MOTOR_CONTROL_CDD_UPDATE_ARM_LIMIT_COUNTS + 1U);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);

    for (invalidCase = 0U; invalidCase < 6U; invalidCase++)
    {
        TEST_CHECK(Test_StartSamplingPipeline() == 0);
        TIM1->CNT = 0U;
        TIM1->CR1 &= ~TIM_CR1_DIR;
        switch (invalidCase)
        {
            case 0U:
                MotorControlCdd_PreparedGeneration.generation.generationId = 2U;
                break;
            case 1U: TIM1->CCR2++; break;
            case 2U: MotorControlCdd_QueuedGenerationValid = TRUE; break;
            case 3U: ADC1->CR |= ADC_CR_JADSTART; break;
            case 4U:
                ADC1->JSQR =
                    (1UL << ADC_JSQR_JEXTEN_Pos);
                break;
            default: ADC1->ISR |= ADC_ISR_JEOS; break;
        }
        TIM1->SR |= TIM_SR_UIF;
        TIM1_UP_TIM16_IRQHandler();
        TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);
    }

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(0U);
    ADC1_2_IRQHandler();
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(0U);
    ADC1->JSQR = 0U;
    ADC1->ISR |= (ADC_ISR_JEOS | ADC_ISR_JQOVF);
    ADC1_2_IRQHandler();
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);
    TEST_CHECK(MotorControlCdd_GetFaultEvents(&events) == E_OK);
    TEST_CHECK((events.adcEvent & MOTOR_CONTROL_CDD_EVENT_CAUSE_MASK) ==
               MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    ADC1->CR |= ADC_CR_JADSTART;
    TEST_CHECK(MotorControlCdd_StopSampling() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    ADC1->JSQR = (1UL << ADC_JSQR_JEXTEN_Pos);
    TEST_CHECK(MotorControlCdd_StopSampling() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);

    TEST_CHECK(Test_StartSamplingPipeline() == 0);
    Test_TriggerUpdateAt(0U);
    TEST_CHECK(MotorControlCdd_StopSampling() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);
    return 0;
}

static int Test_ThreeSlotMailboxInterleavingsAndExpiry(void)
{
    MotorControlCdd_CurrentSampleType sample;
    uint32 dropCount;
    uint32 slotIndex;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    __atomic_store_n(&MotorControlCdd_ConsumedGenerationId, 12U,
                     __ATOMIC_RELAXED);
    MotorControlCdd_PublishCurrentSample(10, 20, -30, 10U, TRUE);
    MotorControlCdd_PublishCurrentSample(11, 21, -32, 11U, TRUE);
    MotorControlCdd_PublishCurrentSample(12, 22, -34, 12U, TRUE);
    MotorControlCdd_PublishCurrentSample(13, 23, -36, 13U, TRUE);
    TEST_CHECK(MotorControlCdd_GetSampleDropCount(&dropCount) == E_OK);
    TEST_CHECK(dropCount == 1U);
    TEST_CHECK(MotorControlCdd_GetSampleDropCount(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCurrentGenerationId(&dropCount) == E_OK);
    TEST_CHECK(dropCount == 12U);
    TEST_CHECK(MotorControlCdd_GetCurrentGenerationId(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_OK);
    TEST_CHECK(sample.generationId == 12U);
    TEST_CHECK(sample.phaseA == 12);
    for (slotIndex = 0U;
         slotIndex < MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
         slotIndex++)
    {
        TEST_CHECK(MotorControlCdd_CurrentSampleMailbox[slotIndex].state ==
                   MOTOR_CONTROL_CDD_MAILBOX_FREE);
    }

    __atomic_store_n(&MotorControlCdd_ConsumedGenerationId, 100U,
                     __ATOMIC_RELAXED);
    MotorControlCdd_PublishCurrentSample(1, 2, -3, 59U, TRUE);
    TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_NOT_OK);

    __atomic_store_n(&MotorControlCdd_ConsumedGenerationId, 1U,
                     __ATOMIC_RELAXED);
    MotorControlCdd_PublishCurrentSample(3, 4, -7, UINT_MAX, TRUE);
    TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_OK);
    TEST_CHECK(sample.generationId == UINT_MAX);

    Test_ReadWhileWriting = TRUE;
    MotorControlCdd_PublishCurrentSample(4, 5, -9, 1U, TRUE);
    TEST_CHECK(Test_ReadWhileWritingResult == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_OK);
    TEST_CHECK(sample.generationId == 1U);

    __atomic_store_n(&MotorControlCdd_ConsumedGenerationId, 21U,
                     __ATOMIC_RELAXED);
    MotorControlCdd_PublishCurrentSample(20, 21, -41, 20U, TRUE);
    Test_PublishWhileReading = TRUE;
    Test_PublishWhileReadingGeneration = 21U;
    TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_OK);
    TEST_CHECK(sample.generationId == 21U);
    TEST_CHECK(sample.phaseA == 21);
    TEST_CHECK(MotorControlCdd_GetCurrentSample(&sample) == E_NOT_OK);
    TEST_CHECK(Test_MailboxStateMismatch == FALSE);
    return 0;
}

static void Test_InitCalibrationEnvironment(
    MotorControlCdd_CalibrationEnvironmentType *environment,
    uint32 sequence,
    uint32 tick)
{
    uint32 index;

    (void)memset(environment, 0, sizeof(*environment));
    environment->vbusRaw = 2100U;
    environment->temperatureRaw = 900U;
    environment->safetyAcquisitionSequence = sequence;
    environment->safetyTick = tick;
    environment->identity.schemaVersion =
        SC1_MEASUREMENT_SCHEMA_VERSION;
    environment->identity.boardProfileCode =
        SC1_MEASUREMENT_BOARD_PROFILE_CODE;
    for (index = 0U;
         index < SC1_MEASUREMENT_IDENTITY_DIGEST_WORD_COUNT;
         index++)
    {
        environment->identity.evidenceSha256[index] = 0U;
    }
}

static void Test_CompleteRegularPair(uint32 phase, uint16 sample)
{
    ADC1->CR &= ~ADC_CR_ADSTART;
    ADC1->DR = sample;
    ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_OVR;
    if (phase == 2U)
    {
        ADC1->ISR |= ADC_ISR_EOS;
    }
}

static int Test_Sc1aCalibrationCompletesWithoutReadiness(void)
{
    static const uint16 phaseSample[3] = {2000U, 2100U, 2200U};
    MotorControlCdd_CalibrationEnvironmentType environment;
    MotorControlCdd_CalibrationReadinessType readiness;
    MotorControlCdd_CalibrationResultType result;
    uint32 sampleIndex;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    Test_MonitorCalibrationW1c = TRUE;
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    TEST_CHECK(MotorControlCdd_IsCalibrationActive() == TRUE);
    TEST_CHECK((ADC1->CFGR & MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_MASK) ==
               MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_VALUE);
    TEST_CHECK(ADC1->SQR1 == MOTOR_CONTROL_CDD_ADC_REGULAR_SQR1_VALUE);
    TEST_CHECK(ADC1->SQR2 == MOTOR_CONTROL_CDD_ADC_REGULAR_SQR2_VALUE);
    TEST_CHECK(ADC1->IER == 0U);
    TEST_CHECK(ADC1->JSQR == 0U);

    for (sampleIndex = 0U;
         sampleIndex < MOTOR_CONTROL_CDD_CALIBRATION_TOTAL_SAMPLES;
         sampleIndex++)
    {
        uint32 phase = sampleIndex % 3U;

        Test_CompleteRegularPair(phase, phaseSample[phase]);
        environment.safetyAcquisitionSequence++;
        environment.safetyTick += 2U; /* ES0431 greater-than-1-ms idle case. */
        TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_OK);
    }
    TEST_CHECK(MotorControlCdd_CalibrationState ==
               MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST);

    environment.safetyAcquisitionSequence++;
    environment.safetyTick += 2U;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationState ==
               MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY);
    environment.safetyAcquisitionSequence++;
    environment.safetyTick += 2U;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_OK);
    TEST_CHECK(MotorControlCdd_IsCalibrationActive() == FALSE);
    TEST_CHECK(MotorControlCdd_GetCalibrationResult(&result) == E_OK);
    TEST_CHECK(result.valid == TRUE);
    TEST_CHECK(result.measurementApproved == FALSE);
    TEST_CHECK(result.offsetsInstalled == FALSE);
    TEST_CHECK(result.firstSafetySequence == 1U);
    TEST_CHECK(result.lastSafetySequence == 195U);
    TEST_CHECK(result.phase[0].mean == 2000U);
    TEST_CHECK(result.phase[1].mean == 2100U);
    TEST_CHECK(result.phase[2].mean == 2200U);
    TEST_CHECK(result.phase[0].peakToPeak == 0U);
    TEST_CHECK(Test_CalibrationW1cTouchedEoc == FALSE);
    TEST_CHECK(MotorControlCdd_GetCalibrationReadiness(&readiness) == E_OK);
    TEST_CHECK(readiness.structuralValid == TRUE);
    TEST_CHECK(readiness.measurementApproved == FALSE);
    TEST_CHECK(readiness.calibrationComplete == TRUE);
    TEST_CHECK(readiness.offsetsInstalled == FALSE);
    TEST_CHECK(MotorControlCdd_StartSampling() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CheckStaticConfiguration() == E_OK);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_RequestCalibrationAbort(
                   (MotorControlCdd_CalibrationAbortModeType)0U) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_RequestCalibrationAbort(
                   MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP) ==
               E_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationResult(&result) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_NOT_OK);
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK((GPIOB->ODR & MOTOR_CONTROL_CDD_SAFE_PIN_MASK) == 0U);
    return 0;
}

static int Test_Sc1aCalibrationFlagsAndCleanupFailClosed(void)
{
    MotorControlCdd_CalibrationEnvironmentType environment;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);
    TEST_CHECK(MotorControlCdd_IsCalibrationCleanupPending() == TRUE);
    TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_IsCalibrationCleanupPending() == FALSE);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    Test_HoldCalibrationStop = TRUE;
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CalibrationState ==
               MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    ADC1->DR = 2000U;
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    ADC1->CR &= ~ADC_CR_ADSTART;
    ADC1->DR = 2000U;
    ADC1->ISR = ADC_ISR_OVR;
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    MotorControlCdd_CalibrationBaseline.valid = FALSE;
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CalibrationState ==
               MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_OK);
    ADC1->CFGR ^= ADC_CFGR_CONT;
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CalibrationState ==
               MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    {
        ADC_TypeDef adcBefore = MotorHost_Adc1;

        TEST_CHECK(MotorControlCdd_RequestCalibrationAbort(
                       MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC) ==
                   E_OK);
        TEST_CHECK(memcmp(&adcBefore, &MotorHost_Adc1,
                          sizeof(adcBefore)) == 0);
    }
    TEST_CHECK(MotorControlCdd_IsCalibrationActive() == FALSE);
    TEST_CHECK(MotorControlCdd_IsCalibrationCleanupPending() == FALSE);
    TEST_CHECK(MotorControlCdd_CalibrationCleanupStep() == E_NOT_OK);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    ADC1->CR &= ~ADC_CR_ADSTART;
    ADC1->DR = 2000U;
    ADC1->ISR = ADC_ISR_EOC; /* Adjacent negative: OVR missing. */
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    ADC1->CR &= ~ADC_CR_ADSTART;
    ADC1->DR = 2000U;
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR | ADC_ISR_EOS;
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, UINT_MAX, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    Test_CompleteRegularPair(0U, 2000U);
    environment.safetyAcquisitionSequence = 0U;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_OK);
    TEST_CHECK(MotorControlCdd_CalibrationAcceptedSamples == 1U);
    return 0;
}

static int Test_Sc1CalibrationStartPreconditions(void)
{
    MotorControlCdd_CalibrationEnvironmentType environment;
    uint32 testCase;

    for (testCase = 0U; testCase < 15U; testCase++)
    {
        Test_ResetFixture();
        TEST_CHECK(Test_RunSuccessfulInit() == 0);
        Test_InitCalibrationEnvironment(&environment, 1U, 1U);
        switch (testCase)
        {
            case 0U:
                MotorControlCdd_State = MOTOR_CONTROL_CDD_SAMPLING;
                break;
            case 1U:
                MotorControlCdd_CalibrationState =
                    MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE;
                break;
            case 2U:
                MotorControlCdd_PreparedGenerationValid = TRUE;
                break;
            case 3U:
                MotorControlCdd_QueuedGenerationValid = TRUE;
                break;
            case 4U:
                MotorControlCdd_BreakEvent = 1U;
                break;
            case 5U:
                MotorControlCdd_UpdateEvent = 1U;
                break;
            case 6U:
                MotorControlCdd_AdcEvent = 1U;
                break;
            case 7U:
                MotorControlCdd_CalibrationEvent = 1U;
                break;
            case 8U:
                ADC1->CR |= ADC_CR_ADSTART;
                break;
            case 9U:
                ADC1->CR |= ADC_CR_JADSTART;
                break;
            case 10U:
                ADC1->CR |= ADC_CR_ADSTP;
                break;
            case 11U:
                ADC1->JSQR = 1U;
                break;
            case 12U:
                GPIOB->ODR |= GPIO_ODR_OD5;
                break;
            case 13U:
                TIM1->CR1 |= TIM_CR1_CEN;
                break;
            default:
                ADC1->CR |= ADC_CR_JADSTP;
                break;
        }
        TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_NOT_OK);
        TEST_CHECK(MotorControlCdd_CalibrationAcceptedSamples == 0U);
        TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    }
    return 0;
}

static int Test_Sc1CalibrationPostConfigurationPreconditions(void)
{
    MotorControlCdd_CalibrationEnvironmentType environment;
    uint32 testCase;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    Test_MotorConfigCorrupt = TRUE;
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_CONFIG);

    for (testCase = 1U; testCase <= 6U; testCase++)
    {
        Std_ReturnType startResult;

        Test_ResetFixture();
        TEST_CHECK(Test_RunSuccessfulInit() == 0);
        Test_InitCalibrationEnvironment(&environment, 1U, 1U);
        Test_StartPostConfigFault = testCase;
        startResult = MotorControlCdd_StartCalibration(&environment);
        if (startResult != E_NOT_OK)
        {
            (void)printf("FAIL post-configuration case %lu\n",
                         (unsigned long)testCase);
        }
        TEST_CHECK(startResult == E_NOT_OK);
        TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
                   MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_PRECONDITION);
        TEST_CHECK(MotorControlCdd_IsCalibrationCleanupPending() == TRUE);
        TEST_CHECK((TIM1->BDTR & TIM_BDTR_MOE) == 0U);
    }
    return 0;
}

static int Test_Sc1CalibrationMetadataRegisterAndSubstateFailures(void)
{
    static const MotorControlCdd_CalibrationStateType abortState[] =
    {
        MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64,
        MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST,
        MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY
    };
    MotorControlCdd_CalibrationEnvironmentType environment;
    uint32 stateIndex;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_StartCalibration(NULL_PTR) == E_NOT_OK);
    Test_InitCalibrationEnvironment(&environment, 0U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    environment.identity.boardProfileCode++;
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    Test_CompleteRegularPair(0U, 2000U);
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    Test_CompleteRegularPair(0U, 2000U);
    environment.safetyAcquisitionSequence = 3U;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 2U, 2U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    Test_CompleteRegularPair(0U, 2000U);
    environment.safetyAcquisitionSequence = 1U;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    ADC1->CFGR ^= ADC_CFGR_CONT;
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_REGISTER);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    Test_CompleteRegularPair(0U, 2000U);
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_OK);
    Test_CompleteRegularPair(1U, 2100U);
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_OK);
    ADC1->CR &= ~ADC_CR_ADSTART;
    ADC1->DR = 2200U;
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    MotorControlCdd_CalibrationState =
        MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST;
    ADC1->CR |= ADC_CR_ADSTART;
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_RESTORE);
    TEST_CHECK(MotorControlCdd_CalibrationEvent != 0U);

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    Test_InitCalibrationEnvironment(&environment, 1U, 1U);
    TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
    MotorControlCdd_CalibrationState =
        MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY;
    MotorControlCdd_CalibrationBaseline.valid = FALSE;
    environment.safetyAcquisitionSequence++;
    environment.safetyTick++;
    TEST_CHECK(MotorControlCdd_CalibrationStep(&environment) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetCalibrationFailureCause() ==
               MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_RESTORE);
    TEST_CHECK(MotorControlCdd_CalibrationEvent != 0U);

    for (stateIndex = 0U;
         stateIndex < (sizeof(abortState) / sizeof(abortState[0]));
         stateIndex++)
    {
        Test_ResetFixture();
        TEST_CHECK(Test_RunSuccessfulInit() == 0);
        Test_InitCalibrationEnvironment(&environment, 1U, 1U);
        TEST_CHECK(MotorControlCdd_StartCalibration(&environment) == E_OK);
        MotorControlCdd_CalibrationState = abortState[stateIndex];
        TEST_CHECK(MotorControlCdd_RequestCalibrationAbort(
                       MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP) ==
                   E_OK);
        TEST_CHECK(MotorControlCdd_IsCalibrationCleanupPending() == TRUE);
    }
    return 0;
}

static int Test_Sc1UnapprovedAccumulatorMaximum(void)
{
    uint32 phase;
    uint32 sampleIndex;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_CfgGetCalibration(
                   &MotorControlCdd_ActiveCalibrationConfig) == E_OK);
    MotorControlCdd_ResetCalibrationData();
    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        for (sampleIndex = 0U;
             sampleIndex < MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT;
             sampleIndex++)
        {
            TEST_CHECK(MotorControlCdd_RecordCalibrationSample(
                           phase, 4095U) == TRUE);
        }
    }
    TEST_CHECK(MotorControlCdd_CalibrationSum[0] == 262080U);
    TEST_CHECK(MotorControlCdd_ValidateCalibrationStatistics() == TRUE);
    TEST_CHECK(MotorControlCdd_CalibrationResult.phase[0].mean == 4095U);
    TEST_CHECK(MotorControlCdd_CurrentOffsetsValid == FALSE);

    MotorControlCdd_ResetCalibrationData();
    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        for (sampleIndex = 0U;
             sampleIndex < MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT;
             sampleIndex++)
        {
            uint16 sample = (sampleIndex < 32U) ? 1000U : 1001U;

            TEST_CHECK(MotorControlCdd_RecordCalibrationSample(
                           phase, sample) == TRUE);
        }
    }
    TEST_CHECK(MotorControlCdd_ValidateCalibrationStatistics() == TRUE);
    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        TEST_CHECK(MotorControlCdd_CalibrationResult.phase[phase].minimum ==
                   1000U);
        TEST_CHECK(MotorControlCdd_CalibrationResult.phase[phase].maximum ==
                   1001U);
        TEST_CHECK(MotorControlCdd_CalibrationResult.phase[phase].mean ==
                   1001U);
        TEST_CHECK(MotorControlCdd_CalibrationResult.phase[phase].peakToPeak ==
                   1U);
    }
    return 0;
}

static boolean Test_ValidateApprovedStatistics(
    uint32 selectedPhase,
    uint16 selectedLow,
    uint16 selectedHigh)
{
    uint32 phase;
    uint32 sampleIndex;

    MotorControlCdd_ResetCalibrationData();
    MotorControlCdd_CurrentOffsetsValid = FALSE;
    MotorControlCdd_CalibrationFailure =
        MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE;
    if (MotorControlCdd_CfgGetCalibration(
            &MotorControlCdd_ActiveCalibrationConfig) != E_OK)
    {
        return FALSE;
    }

    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        for (sampleIndex = 0U;
             sampleIndex < MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT;
             sampleIndex++)
        {
            uint16 sample = 2100U;

            if (phase == selectedPhase)
            {
                sample = selectedLow;
                if (sampleIndex == 1U)
                {
                    sample = selectedHigh;
                }
                else if (sampleIndex > 1U)
                {
                    sample = (uint16)(((uint32)selectedLow +
                                       (uint32)selectedHigh) / 2U);
                }
            }
            if (MotorControlCdd_RecordCalibrationSample(phase, sample) != TRUE)
            {
                return FALSE;
            }
        }
    }
    return MotorControlCdd_ValidateCalibrationStatistics();
}

static int Test_Sc1ApprovedBoundsAndAdjacentFailures(void)
{
    uint32 phase;

    Test_ResetFixture();
    Test_MotorConfigApproved = TRUE;
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_CfgGetCalibration(
                   &MotorControlCdd_ActiveCalibrationConfig) == E_OK);

    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        MotorControlCdd_ResetCalibrationData();
        TEST_CHECK(MotorControlCdd_RecordCalibrationSample(
                       phase, 99U) == FALSE);
        MotorControlCdd_ResetCalibrationData();
        TEST_CHECK(MotorControlCdd_RecordCalibrationSample(
                       phase, 100U) == TRUE);
        MotorControlCdd_ResetCalibrationData();
        TEST_CHECK(MotorControlCdd_RecordCalibrationSample(
                       phase, 4000U) == TRUE);
        MotorControlCdd_ResetCalibrationData();
        TEST_CHECK(MotorControlCdd_RecordCalibrationSample(
                       phase, 4001U) == FALSE);

        TEST_CHECK(Test_ValidateApprovedStatistics(
                       phase, 1999U, 1999U) == FALSE);
        TEST_CHECK(Test_ValidateApprovedStatistics(
                       phase, 2000U, 2000U) == TRUE);
        TEST_CHECK(Test_ValidateApprovedStatistics(
                       phase, 2200U, 2200U) == TRUE);
        TEST_CHECK(Test_ValidateApprovedStatistics(
                       phase, 2201U, 2201U) == FALSE);
        TEST_CHECK(Test_ValidateApprovedStatistics(
                       phase, 2000U, 2010U) == TRUE);
        TEST_CHECK(Test_ValidateApprovedStatistics(
                       phase, 2000U, 2011U) == FALSE);
    }
    return 0;
}

static int Test_EveryMotorEventPublishesCriticalInterlock(void)
{
    Test_ResetFixture();

    MotorControlCdd_PublishEvent(
        &MotorControlCdd_BreakEvent,
        MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK);
    TEST_CHECK((MotorControlCdd_BreakEvent & 0xFFFFU) ==
               MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK);
    TEST_CHECK(Test_CriticalFaultMask ==
               STATE_MANAGER_CRITICAL_CLASS_MOTOR);

    Test_CriticalFaultMask = 0U;
    MotorControlCdd_PublishEvent(
        &MotorControlCdd_UpdateEvent,
        MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_UPDATE);
    TEST_CHECK((MotorControlCdd_UpdateEvent & 0xFFFFU) ==
               MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_UPDATE);
    TEST_CHECK(Test_CriticalFaultMask ==
               STATE_MANAGER_CRITICAL_CLASS_MOTOR);

    Test_CriticalFaultMask = 0U;
    MotorControlCdd_PublishEvent(
        &MotorControlCdd_AdcEvent,
        MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC);
    TEST_CHECK((MotorControlCdd_AdcEvent & 0xFFFFU) ==
               MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC);
    TEST_CHECK(Test_CriticalFaultMask ==
               STATE_MANAGER_CRITICAL_CLASS_MOTOR);

    Test_CriticalFaultMask = 0U;
    MotorControlCdd_PublishEvent(
        &MotorControlCdd_CalibrationEvent,
        MOTOR_CONTROL_CDD_EVENT_CAUSE_CALIBRATION);
    TEST_CHECK((MotorControlCdd_CalibrationEvent & 0xFFFFU) ==
               MOTOR_CONTROL_CDD_EVENT_CAUSE_CALIBRATION);
    TEST_CHECK(Test_CriticalFaultMask ==
               STATE_MANAGER_CRITICAL_CLASS_MOTOR);
    return 0;
}

static MotorControlCdd_FastSafetySnapshotType Test_MakeFastSafety(
    uint32 acquisitionSequence,
    uint32 generationId)
{
    MotorControlCdd_FastSafetySnapshotType snapshot = { 0 };

    snapshot.vbusRaw = 2000U;
    snapshot.temperatureRaw = 1000U;
    snapshot.acquisitionSequence = acquisitionSequence;
    snapshot.osTick = acquisitionSequence;
    snapshot.pwmGenerationId = generationId;
    snapshot.acquisitionValid = TRUE;
    snapshot.limitsValid = TRUE;
    snapshot.structuralValid = TRUE;
    snapshot.measurementApproved = TRUE;
    snapshot.identity.schemaVersion = SC1_MEASUREMENT_SCHEMA_VERSION;
    snapshot.identity.boardProfileCode =
        SC1_MEASUREMENT_BOARD_PROFILE_CODE;
    snapshot.identity.evidenceSha256[0] = 1U;
    return snapshot;
}

static int Test_PrepareSc2ApprovedSampling(void)
{
    MotorControlCdd_CurrentOffsetsType offsets = {2000U, 2100U, 2200U};

    Test_ResetFixture();
    Test_MotorConfigApproved = TRUE;
    Test_ControlApprovalMask = 7U;
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_InstallCurrentOffsets(&offsets) == E_OK);
    MotorControlCdd_CalibrationState = MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE;
    TEST_CHECK(MotorControlCdd_StartSampling() == E_OK);
    Test_TriggerUpdateAt(0U);
    TIM1->CNT = 1000U;
    Test_CompleteInjectedPair(2010U, 2110U);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING);
    TEST_CHECK(MotorControlCdd_ConsumedGenerationId == 1U);
    return 0;
}

static int Test_Sc2aUnapprovedLifecycleAndPartialApprovals(void)
{
    MotorControlCdd_ControlStatusType status;
    MotorControlCdd_ControlRequestType request = { 0 };
    MotorControlCdd_FastSafetySnapshotType snapshot = { 0 };
    uint32 mask;

    Test_ResetFixture();
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    TEST_CHECK(MotorControlCdd_GetControlStatus(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_UNAPPROVED);
    TEST_CHECK(status.epoch == 0U);
    TEST_CHECK(status.acceptedRequestQ15 == 0);
    TEST_CHECK(status.commissioningCapQ15 == 0);
    TEST_CHECK(MotorControlCdd_PublishSafetySnapshot(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_PublishSafetySnapshot(&snapshot) == E_OK);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_NOT_OK);
    TEST_CHECK(Test_FastLoopInitCount == 0U);

    for (mask = 1U; mask < 7U; mask++)
    {
        Test_ResetFixture();
        Test_ControlApprovalMask = mask;
        TEST_CHECK(Test_RunSuccessfulInit() == 0);
        TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
        TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_UNAPPROVED);
        TEST_CHECK(MotorControlCdd_StageControl(&request) == E_NOT_OK);
        TEST_CHECK(Test_FastLoopInitCount == 0U);
    }
    return 0;
}

static int Test_Sc2aCurrentConversionAndNormativeMapping(void)
{
    MotorControlFastLoop_ResultType result = { 0 };
    MotorControlCdd_GenerationType generation;
    MotorControlCdd_PhaseType rank1;
    MotorControlCdd_PhaseType rank2;
    MotorControlCdd_PhaseType reconstructed;
    MotorControlCdd_TriggerEdgeType edge;
    uint16 ccr4;
    sint32 raw;

    Test_ResetFixture();
    Test_ControlApprovalMask = 7U;
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    for (raw = -4095; raw <= 4095; raw++)
    {
        sint32 normalized = 0;

        TEST_CHECK(MotorControlCdd_NormalizeRawCurrent(
                       raw, &normalized) == E_OK);
        TEST_CHECK(normalized == (raw * 8));
    }
    TEST_CHECK(MotorControlCdd_NormalizeRawCurrent(4096, &raw) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_NormalizeRawCurrent(0, NULL_PTR) == E_NOT_OK);

    result.controlValid = TRUE;
    result.nextGenerationId = 10U;
    result.requestedSvpwm.dutyAQ15 = 16384U;
    result.requestedSvpwm.dutyBQ15 = 16384U;
    result.requestedSvpwm.dutyCQ15 = 16384U;
    result.requestedSvpwm.sector = 1U;
    TEST_CHECK(MotorControlCdd_BuildControlGeneration(
                   &result, &generation) == E_OK);
    TEST_CHECK(generation.generationId == 10U);
    TEST_CHECK(generation.ccr1 == 2125U);
    TEST_CHECK(generation.ccr2 == 2125U);
    TEST_CHECK(generation.ccr3 == 2125U);
    TEST_CHECK(generation.rank1Phase == MOTOR_CONTROL_CDD_PHASE_A);
    TEST_CHECK(generation.rank2Phase == MOTOR_CONTROL_CDD_PHASE_B);
    TEST_CHECK(generation.reconstructedPhase == MOTOR_CONTROL_CDD_PHASE_C);
    TEST_CHECK(generation.triggerEdge == MOTOR_CONTROL_CDD_TRIGGER_RISING);
    TEST_CHECK(generation.appliedVoltageAlphaQ15 == 0);
    TEST_CHECK(generation.appliedVoltageBetaQ15 == 0);

    TEST_CHECK(MotorControlCdd_SelectSamplingWindow(
                   4250U, 2125U, 2125U, 1U,
                   &rank1, &rank2, &reconstructed, &ccr4, &edge) == TRUE);
    TEST_CHECK(rank1 == MOTOR_CONTROL_CDD_PHASE_B);
    TEST_CHECK(rank2 == MOTOR_CONTROL_CDD_PHASE_C);
    TEST_CHECK(reconstructed == MOTOR_CONTROL_CDD_PHASE_A);
    TEST_CHECK(MotorControlCdd_SelectSamplingWindow(
                   4250U, 2125U, 2125U, 4U,
                   &rank1, &rank2, &reconstructed, &ccr4, &edge) == FALSE);

    result.requestedSvpwm.limited = TRUE;
    TEST_CHECK(MotorControlCdd_BuildControlGeneration(
                   &result, &generation) == E_NOT_OK);
    result.requestedSvpwm.limited = FALSE;
    result.requestedVoltage.alphaQ15 = 25;
    TEST_CHECK(MotorControlCdd_BuildControlGeneration(
                   &result, &generation) == E_NOT_OK);
    return 0;
}

static int Test_Sc2aStageClaimAndActiveComposition(void)
{
    MotorControlCdd_ControlRequestType request = { 0 };
    MotorControlCdd_ControlStatusType status;
    MotorControlCdd_FastSafetySnapshotType snapshot;
    uint32 index;

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_IDLE);

    request.requestCurrentMa = 200;
    request.alreadyConsumedStartingCycles = 2U;
    request.commandContinueValid = TRUE;
    request.stopRequested = FALSE;
    request.safetySnapshot = Test_MakeFastSafety(1U, 1U);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_NOT_OK);
    TEST_CHECK(Test_FastLoopInitCount == 1U);
    TEST_CHECK(Test_FastLoopBeginCount == 1U);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_PRIMING);
    TEST_CHECK(status.epoch == 1U);
    TEST_CHECK(status.acceptedRequestQ15 == 1600);
    TEST_CHECK(status.commissioningCapQ15 == 2400);
    TEST_CHECK(MotorControlCdd_ControlStaging.payload
                   .fastLoop.config.startup.alignQRefQ15 == 800);
    TEST_CHECK(MotorControlCdd_ControlStaging.payload
                   .fastLoop.config.startup.finalQRefQ15 == 1600);

    Test_TriggerUpdateAt(0U);
    TIM1->CNT = 1000U;
    Test_CompleteInjectedPair(2010U, 2110U);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_ACTIVE);
    TEST_CHECK(Test_FastLoopStepCount == 0U);

    for (index = 0U; index < MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT; index++)
    {
        snapshot = Test_MakeFastSafety(index + 2U, 3U);
        TEST_CHECK(MotorControlCdd_PublishSafetySnapshot(&snapshot) == E_OK);
    }
    snapshot = Test_MakeFastSafety(5U, 3U);
    TEST_CHECK(MotorControlCdd_PublishSafetySnapshot(&snapshot) == E_NOT_OK);

    Test_FastLoopStepResult = (MotorControlFastLoop_ResultType){ 0 };
    Test_FastLoopStepResult.controlValid = TRUE;
    Test_FastLoopStepResult.nextGenerationId = 4U;
    Test_FastLoopStepResult.requestedSvpwm.dutyAQ15 = 16384U;
    Test_FastLoopStepResult.requestedSvpwm.dutyBQ15 = 16384U;
    Test_FastLoopStepResult.requestedSvpwm.dutyCQ15 = 16384U;
    Test_FastLoopStepResult.requestedSvpwm.sector = 1U;
    Test_TriggerUpdateAt(0U);
    TIM1->CNT = 1000U;
    Test_CompleteInjectedPair(2010U, 2110U);
    TEST_CHECK(Test_FastLoopStepCount == 1U);
    TEST_CHECK(Test_LastFastLoopInput.sampleGenerationId == 3U);
    TEST_CHECK(Test_LastFastLoopInput.appliedVoltageGenerationId == 3U);
    TEST_CHECK(Test_LastFastLoopInput.phaseAQ15 == 80);
    TEST_CHECK(Test_LastFastLoopInput.phaseBQ15 == 80);
    TEST_CHECK(Test_LastFastLoopInput.phaseCQ15 == -160);
    TEST_CHECK(MotorControlCdd_PreparedGeneration.generation.generationId ==
               4U);
    TEST_CHECK(MotorControlCdd_PreparedGeneration.generation
                   .appliedVoltageAlphaQ15 == 0);
    TEST_CHECK(MotorControlCdd_StopSampling() == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    request = (MotorControlCdd_ControlRequestType){ 0 };
    request.requestCurrentMa = 99;
    request.alreadyConsumedStartingCycles = 2U;
    request.commandContinueValid = TRUE;
    request.safetySnapshot = Test_MakeFastSafety(1U, 1U);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_NOT_OK);
    TEST_CHECK(Test_FastLoopInitCount == 0U);
    request.requestCurrentMa = -200;
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    TEST_CHECK(MotorControlCdd_ControlStaging.payload
                   .fastLoop.config.startup.alignQRefQ15 == -800);
    TEST_CHECK(MotorControlCdd_ControlStaging.payload
                   .fastLoop.config.startup.finalQRefQ15 == -1600);
    TEST_CHECK(MotorControlCdd_ControlStaging.payload
                   .fastLoop.expectedDirection == -1);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    request.requestCurrentMa = 200;
    request.safetySnapshot = Test_MakeFastSafety(1U, 1U);
    Test_FastLoopInitReturn = E_NOT_OK;
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_ControlStaging.state ==
               MOTOR_CONTROL_CDD_MAILBOX_FREE);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_IDLE);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    Test_FastLoopBeginReturn = E_NOT_OK;
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_ControlStaging.state ==
               MOTOR_CONTROL_CDD_MAILBOX_FREE);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_IDLE);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    MotorControlCdd_StoreControlLifecycle(
        MOTOR_CONTROL_CDD_CONTROL_IDLE, 0x00FFFFFFUL);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_PRIMING);
    TEST_CHECK(status.epoch == 0U);
    return 0;
}

static int Test_Sc2aCommandAndStagingBoundaries(void)
{
    static const sint32 requests[] = {-301, -300, -100, 0, 100, 300, 301};
    MotorControlCdd_ControlRequestType request;
    MotorControlCdd_ControlStatusType status;
    uint32 index;

    for (index = 0U; index < (sizeof(requests) / sizeof(requests[0]));
         index++)
    {
        TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
        request = (MotorControlCdd_ControlRequestType){ 0 };
        request.requestCurrentMa = requests[index];
        request.alreadyConsumedStartingCycles = 2U;
        request.commandContinueValid = TRUE;
        request.safetySnapshot = Test_MakeFastSafety(1U, 1U);
        if ((requests[index] >= -300) && (requests[index] <= -100))
        {
            TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
            TEST_CHECK(MotorControlCdd_ControlStaging.payload
                           .fastLoop.config.startup.finalQRefQ15 ==
                       (requests[index] * 8));
            TEST_CHECK(MotorControlCdd_ControlStaging.payload
                           .fastLoop.expectedDirection == -1);
        }
        else if ((requests[index] >= 100) && (requests[index] <= 300))
        {
            TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
            TEST_CHECK(MotorControlCdd_ControlStaging.payload
                           .fastLoop.config.startup.finalQRefQ15 ==
                       (requests[index] * 8));
            TEST_CHECK(MotorControlCdd_ControlStaging.payload
                           .fastLoop.expectedDirection == 1);
        }
        else
        {
            TEST_CHECK(MotorControlCdd_StageControl(&request) == E_NOT_OK);
            TEST_CHECK(Test_FastLoopInitCount == 0U);
        }
    }

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    request = (MotorControlCdd_ControlRequestType){ 0 };
    request.requestCurrentMa = 200;
    request.alreadyConsumedStartingCycles = 2U;
    request.commandContinueValid = TRUE;
    request.safetySnapshot = Test_MakeFastSafety(1U, 1U);
    Test_ControlStagingInterleaveStage =
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_WRITING;
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    TEST_CHECK(Test_ControlStagingInterleaveStage == 0U);
    TEST_CHECK(Test_ControlStagingClaimResult == FALSE);
    TEST_CHECK(MotorControlCdd_ClaimStagedControl(1U) == TRUE);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    Test_ControlStagingInterleaveStage =
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READY;
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    TEST_CHECK(Test_ControlStagingInterleaveStage == 0U);
    TEST_CHECK(Test_ControlStagingClaimResult == FALSE);
    TEST_CHECK(MotorControlCdd_ClaimStagedControl(1U) == TRUE);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    Test_ControlStagingInterleaveStage =
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_PUBLISHED;
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    TEST_CHECK(Test_ControlStagingInterleaveStage == 0U);
    TEST_CHECK(Test_ControlStagingClaimResult == TRUE);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_ACTIVE);
    TEST_CHECK(MotorControlCdd_ControlStaging.state ==
               MOTOR_CONTROL_CDD_MAILBOX_FREE);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    Test_InterleaveControlRequest = request;
    Test_ControlStagingInterleaveStage =
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READING;
    TEST_CHECK(MotorControlCdd_ClaimStagedControl(1U) == TRUE);
    TEST_CHECK(Test_ControlStagingInterleaveStage == 0U);
    TEST_CHECK(Test_ControlStagingDuplicateResult == FALSE);
    TEST_CHECK(MotorControlCdd_StopSampling() == E_NOT_OK);
    return 0;
}

static int Test_Sc2aFastSafetyBoundariesAndFailure(void)
{
    MotorControlCdd_ControlRequestType request = { 0 };
    MotorControlCdd_ControlStatusType status;
    MotorControlCdd_FastSafetySnapshotType snapshot;
    Sc1_MeasurementIdentityType identity;

    snapshot = Test_MakeFastSafety(1U, 100U);
    identity = snapshot.identity;
    TEST_CHECK(MotorControlCdd_FastSafetyIsUsable(
                   &snapshot, &identity, 140U) == TRUE);
    TEST_CHECK(MotorControlCdd_FastSafetyIsUsable(
                   &snapshot, &identity, 141U) == FALSE);
    TEST_CHECK(MotorControlCdd_FastSafetyIsUsable(
                   &snapshot, &identity, 99U) == FALSE);
    snapshot.pwmGenerationId = UINT_MAX - 19U;
    TEST_CHECK(MotorControlCdd_FastSafetyIsUsable(
                   &snapshot, &identity, 20U) == TRUE);
    snapshot.pwmGenerationId = 5U;
    TEST_CHECK(MotorControlCdd_FastSafetyIsUsable(
                   &snapshot, &identity, UINT_MAX - 5U) == FALSE);
    snapshot = Test_MakeFastSafety(1U, 1U);
    snapshot.acquisitionValid = (boolean)2U;
    TEST_CHECK(MotorControlCdd_FastSafetyIsUsable(
                   &snapshot, &identity, 1U) == FALSE);
    snapshot = Test_MakeFastSafety(1U, 1U);
    snapshot.identity.evidenceSha256[0] = 2U;
    TEST_CHECK(MotorControlCdd_FastSafetyIsUsable(
                   &snapshot, &identity, 1U) == FALSE);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    request.requestCurrentMa = 200;
    request.alreadyConsumedStartingCycles = 2U;
    request.commandContinueValid = TRUE;
    request.safetySnapshot = Test_MakeFastSafety(1U, 1U);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    TEST_CHECK(MotorControlCdd_ClaimStagedControl(1U) == TRUE);
    MotorControlCdd_RetainedFastSafetyValid = FALSE;
    snapshot = Test_MakeFastSafety(UINT_MAX, 1U);
    Test_FastSafetyConsumeWhileWriting = TRUE;
    TEST_CHECK(MotorControlCdd_PublishSafetySnapshot(&snapshot) == E_OK);
    TEST_CHECK(Test_FastSafetyConsumeWhileWriting == FALSE);
    TEST_CHECK(MotorControlCdd_RetainedFastSafetyValid == FALSE);
    MotorControlCdd_ConsumeNewestFastSafety();
    TEST_CHECK(MotorControlCdd_RetainedFastSafetyValid == TRUE);
    TEST_CHECK(MotorControlCdd_RetainedFastSafety.acquisitionSequence ==
               UINT_MAX);

    snapshot = Test_MakeFastSafety(0U, 1U);
    TEST_CHECK(MotorControlCdd_PublishSafetySnapshot(&snapshot) == E_OK);
    MotorControlCdd_ConsumeNewestFastSafety();
    TEST_CHECK(MotorControlCdd_RetainedFastSafety.acquisitionSequence == 0U);

    snapshot = Test_MakeFastSafety(1U, 1U);
    TEST_CHECK(MotorControlCdd_PublishSafetySnapshot(&snapshot) == E_OK);
    Test_InterleaveFastSafety = Test_MakeFastSafety(2U, 1U);
    Test_FastSafetyPublishWhileReading = TRUE;
    MotorControlCdd_ConsumeNewestFastSafety();
    TEST_CHECK(Test_FastSafetyPublishWhileReading == FALSE);
    TEST_CHECK(Test_FastSafetyPublishWhileReadingResult == E_OK);
    MotorControlCdd_ConsumeNewestFastSafety();
    TEST_CHECK(MotorControlCdd_RetainedFastSafety.acquisitionSequence == 2U);

    TEST_CHECK(Test_PrepareSc2ApprovedSampling() == 0);
    request.safetySnapshot = Test_MakeFastSafety(1U, 1U);
    TEST_CHECK(MotorControlCdd_StageControl(&request) == E_OK);
    Test_TriggerUpdateAt(0U);
    TIM1->CNT = 1000U;
    Test_CompleteInjectedPair(2010U, 2110U);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_ACTIVE);
    MotorControlCdd_RetainedFastSafety =
        Test_MakeFastSafety(2U, MotorControlCdd_ConsumedGenerationId - 41U);
    MotorControlCdd_RetainedFastSafetyValid = TRUE;
    Test_TriggerUpdateAt(0U);
    TIM1->CNT = 1000U;
    Test_CompleteInjectedPair(2010U, 2110U);
    TEST_CHECK(MotorControlCdd_GetControlStatus(&status) == E_OK);
    TEST_CHECK(status.lifecycle == MOTOR_CONTROL_CDD_CONTROL_FAILED);
    TEST_CHECK(status.failure == MOTOR_CONTROL_CDD_CONTROL_FAILURE_SAFETY);
    TEST_CHECK(MotorControlCdd_State == MOTOR_CONTROL_CDD_FAILED);
    TEST_CHECK((MotorControlCdd_ControlEvent & 0xFFFFU) ==
               MOTOR_CONTROL_CDD_EVENT_CAUSE_CONTROL);
    TEST_CHECK(Test_CriticalFaultMask ==
               STATE_MANAGER_CRITICAL_CLASS_MOTOR);
    return 0;
}

static int Test_Sc2aNormativeNumericBoundaries(void)
{
    MotorControlFastLoop_ResultType result = { 0 };
    MotorControlCdd_GenerationType generation;
    MotorControlCdd_PhaseType rank1;
    MotorControlCdd_PhaseType rank2;
    MotorControlCdd_PhaseType reconstructed;
    MotorControlCdd_TriggerEdgeType edge;
    uint16 compares[3] = { 0U, 0U, 0U };
    uint16 ccr4;
    uint32 value;

    Test_ResetFixture();
    Test_ControlApprovalMask = 7U;
    TEST_CHECK(Test_RunSuccessfulInit() == 0);
    for (value = 0U; value <= 32767U; value++)
    {
        uint16 expected = (uint16)((((uint64)value * 4250ULL) +
                                    (32767ULL / 2ULL)) / 32767ULL);

        TEST_CHECK(MotorControlCdd_DutyQ15ToCompare((uint16)value) ==
                   expected);
    }
    for (value = 0U; value <= 4250U; value++)
    {
        uint16 expected = (uint16)((((uint64)value * 32767ULL) +
                                    (4250ULL / 2ULL)) / 4250ULL);

        TEST_CHECK(MotorControlCdd_CompareToDutyQ15((uint16)value) ==
                   expected);
    }

    MotorControlCdd_ActiveControlConfig.triggerArmMinimum = 4300U;
    TEST_CHECK(MotorControlCdd_TrySamplingPair(
                   compares, MOTOR_CONTROL_CDD_PHASE_A,
                   MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_PHASE_C,
                   &rank1, &rank2, &reconstructed, &ccr4, &edge) == TRUE);
    TEST_CHECK(edge == MOTOR_CONTROL_CDD_TRIGGER_FALLING);
    TEST_CHECK((ccr4 > 0U) && (ccr4 < 4250U));
    MotorControlCdd_ActiveControlConfig.triggerArmMinimum = 32U;
    compares[0] = 4250U;
    compares[1] = 4250U;
    TEST_CHECK(MotorControlCdd_TrySamplingPair(
                   compares, MOTOR_CONTROL_CDD_PHASE_A,
                   MOTOR_CONTROL_CDD_PHASE_B, MOTOR_CONTROL_CDD_PHASE_C,
                   &rank1, &rank2, &reconstructed, &ccr4, &edge) == FALSE);

    result.controlValid = TRUE;
    result.nextGenerationId = 1U;
    result.requestedSvpwm.sector = 1U;
    for (value = 0U; value <= 32767U; value++)
    {
        result.requestedSvpwm.dutyAQ15 = (uint16)value;
        result.requestedSvpwm.dutyBQ15 = (uint16)value;
        result.requestedSvpwm.dutyCQ15 = (uint16)value;
        if (MotorControlCdd_BuildControlGeneration(
                &result, &generation) == E_OK)
        {
            TEST_CHECK(generation.appliedVoltageAlphaQ15 == 0);
            TEST_CHECK(generation.appliedVoltageBetaQ15 == 0);
            TEST_CHECK((generation.ccr4 > 0U) &&
                       (generation.ccr4 < 4250U));
        }
    }
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_StaticInitIsSafeAndComplete();
    failures += Test_InitIsOneShotAndChecksPreconditions();
    failures += Test_AdcWaitsAreBounded();
    failures += Test_StaticReadbackDetectsCorruption();
    failures += Test_InterruptsFailSafeAndPublishOwnedEvents();
    failures += Test_GenerationPipelineAllContexts();
    failures += Test_PipelineRejectsUnsafeOrLateWork();
    failures += Test_ThreeSlotMailboxInterleavingsAndExpiry();
    failures += Test_Sc1aCalibrationCompletesWithoutReadiness();
    failures += Test_Sc1aCalibrationFlagsAndCleanupFailClosed();
    failures += Test_Sc1CalibrationStartPreconditions();
    failures += Test_Sc1CalibrationPostConfigurationPreconditions();
    failures += Test_Sc1CalibrationMetadataRegisterAndSubstateFailures();
    failures += Test_Sc1UnapprovedAccumulatorMaximum();
    failures += Test_Sc1ApprovedBoundsAndAdjacentFailures();
    failures += Test_EveryMotorEventPublishesCriticalInterlock();
    failures += Test_Sc2aUnapprovedLifecycleAndPartialApprovals();
    failures += Test_Sc2aCurrentConversionAndNormativeMapping();
    failures += Test_Sc2aStageClaimAndActiveComposition();
    failures += Test_Sc2aCommandAndStagingBoundaries();
    failures += Test_Sc2aFastSafetyBoundariesAndFailure();
    failures += Test_Sc2aNormativeNumericBoundaries();

    if (failures == 0)
    {
        (void)printf("PASS: MotorControl CDD Step-5 host matrix\n");
        return 0;
    }

    (void)printf("FAIL: %d MotorControl CDD host test group(s)\n", failures);
    return 1;
}
