#include "MotorControlCdd.h"
#include "MotorControlCdd_Cfg.h"

#include "Adc.h"
#include "StateManager.h"
#include "stm32g431xx.h"

#define MOTOR_CONTROL_CDD_REQUIRED_AHB2_GATES \
    (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_ADC12EN)

#define MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS   4250U
#define MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS 2125U
#define MOTOR_CONTROL_CDD_INITIAL_SAMPLE_COUNT \
    (MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS - 5U)
#define MOTOR_CONTROL_CDD_ADC_REGULATOR_DELAY_HCLK_CYCLES 3400U
#define MOTOR_CONTROL_CDD_ADC_POST_CALIBRATION_HCLK_CYCLES 16U
#define MOTOR_CONTROL_CDD_ADC_TIMEOUT_OBSERVATIONS 100000U
#define MOTOR_CONTROL_CDD_UPDATE_ARM_LIMIT_COUNTS 64U
#define MOTOR_CONTROL_CDD_COMMIT_CUTOFF_COUNTS    512U
#define MOTOR_CONTROL_CDD_ADC_RAW_MASK             0x0FFFU
#define MOTOR_CONTROL_CDD_ADC_CHANNEL_A            2U
#define MOTOR_CONTROL_CDD_ADC_CHANNEL_B            12U
#define MOTOR_CONTROL_CDD_ADC_CHANNEL_C            15U
#define MOTOR_CONTROL_CDD_CALIBRATION_TOTAL_SAMPLES \
    (MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT * \
     MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT)
#define MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT        3U
#define MOTOR_CONTROL_CDD_SAMPLE_MAX_AGE_GENERATIONS 40U
#define MOTOR_CONTROL_CDD_FAST_SAFETY_MAX_AGE_GENERATIONS 40U
#define MOTOR_CONTROL_CDD_COMMISSIONING_CURRENT_CAP_MA 300
#define MOTOR_CONTROL_CDD_Q15_SCALE                  32768LL
#define MOTOR_CONTROL_CDD_Q15_MAX                    32767
#define MOTOR_CONTROL_CDD_Q15_MIN                    (-32768)
#define MOTOR_CONTROL_CDD_SVPWM_Q15_MAX              32767U
#define MOTOR_CONTROL_CDD_CONTROL_LIFECYCLE_MASK     0x000000FFUL
#define MOTOR_CONTROL_CDD_CONTROL_EPOCH_SHIFT        8U
#define MOTOR_CONTROL_CDD_TRGO_OC4REF_VALUE \
    (7UL << TIM_CR2_MMS_Pos)

#define MOTOR_CONTROL_CDD_TEST_WAIT_CALIBRATION 1U
#define MOTOR_CONTROL_CDD_TEST_WAIT_READY       2U

#ifndef MOTOR_CONTROL_CDD_TEST_WAIT_HOOK
#define MOTOR_CONTROL_CDD_TEST_WAIT_HOOK(stage, observation) \
    do                                                        \
    {                                                         \
        (void)(stage);                                        \
        (void)(observation);                                  \
    } while (0)
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_ADC_STATUS_CLEAR_HOOK
#define MOTOR_CONTROL_CDD_TEST_ADC_STATUS_CLEAR_HOOK(mask) ((void)(mask))
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_CALIBRATION_DR_READ_HOOK
#define MOTOR_CONTROL_CDD_TEST_CALIBRATION_DR_READ_HOOK() ((void)0)
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_CALIBRATION_CONTROL_HOOK
#define MOTOR_CONTROL_CDD_TEST_CALIBRATION_CONTROL_HOOK() ((void)0)
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_TIM_STATUS_CLEAR_HOOK
#define MOTOR_CONTROL_CDD_TEST_TIM_STATUS_CLEAR_HOOK(mask) ((void)(mask))
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_NEXT_GENERATION_HOOK
#define MOTOR_CONTROL_CDD_TEST_NEXT_GENERATION_HOOK(generation) \
    ((void)(generation))
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_COMMIT_HOOK
#define MOTOR_CONTROL_CDD_TEST_COMMIT_HOOK() ((void)0)
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_FAULT_READ_HOOK
#define MOTOR_CONTROL_CDD_TEST_FAULT_READ_HOOK(stage) ((void)(stage))
#endif

#define MOTOR_CONTROL_CDD_TEST_FAULT_READ_BREAK       1U
#define MOTOR_CONTROL_CDD_TEST_FAULT_READ_UPDATE      2U
#define MOTOR_CONTROL_CDD_TEST_FAULT_READ_ADC         3U
#define MOTOR_CONTROL_CDD_TEST_FAULT_READ_CALIBRATION 4U
#define MOTOR_CONTROL_CDD_TEST_FAULT_READ_CONTROL     5U

#ifndef MOTOR_CONTROL_CDD_TEST_MAILBOX_HOOK
#define MOTOR_CONTROL_CDD_TEST_MAILBOX_HOOK(stage, slot) \
    do                                                       \
    {                                                        \
        (void)(stage);                                       \
        (void)(slot);                                        \
    } while (0)
#endif

#define MOTOR_CONTROL_CDD_TEST_MAILBOX_WRITING 1U
#define MOTOR_CONTROL_CDD_TEST_MAILBOX_READY   2U
#define MOTOR_CONTROL_CDD_TEST_MAILBOX_READING 3U

#ifndef MOTOR_CONTROL_CDD_TEST_FAST_SAFETY_HOOK
#define MOTOR_CONTROL_CDD_TEST_FAST_SAFETY_HOOK(stage, slot) \
    do                                                        \
    {                                                         \
        (void)(stage);                                        \
        (void)(slot);                                         \
    } while (0)
#endif

#ifndef MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK
#define MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK(stage) ((void)(stage))
#endif

#define MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_WRITING   1U
#define MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READY     2U
#define MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_PUBLISHED 3U
#define MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READING   4U
#define MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_FREE      5U

typedef enum
{
    MOTOR_CONTROL_CDD_UNINITIALIZED = 0,
    MOTOR_CONTROL_CDD_INITIALIZING,
    MOTOR_CONTROL_CDD_STATIC_SAFE,
    MOTOR_CONTROL_CDD_SAMPLING,
    MOTOR_CONTROL_CDD_FAILED
} MotorControlCdd_InternalStateType;

typedef enum
{
    MOTOR_CONTROL_CDD_CALIBRATION_IDLE = 0,
    MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64,
    MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST,
    MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY,
    MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE,
    MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_REQUEST,
    MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_VERIFY,
    MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_RESTORE,
    MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_VERIFY,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILED_SAFE,
    MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED
} MotorControlCdd_CalibrationStateType;

typedef struct
{
    uint32 cfgr;
    uint32 smpr1;
    uint32 smpr2;
    uint32 sqr1;
    uint32 sqr2;
    uint32 sqr3;
    uint32 sqr4;
    boolean valid;
} MotorControlCdd_CalibrationBaselineType;

typedef struct
{
    MotorControlCdd_GenerationType generation;
    uint32 jsqr;
} MotorControlCdd_RuntimeGenerationType;

typedef enum
{
    MOTOR_CONTROL_CDD_MAILBOX_FREE = 0,
    MOTOR_CONTROL_CDD_MAILBOX_WRITING,
    MOTOR_CONTROL_CDD_MAILBOX_READY,
    MOTOR_CONTROL_CDD_MAILBOX_READING
} MotorControlCdd_MailboxStateType;

typedef struct __attribute__((aligned(4)))
{
    uint32 state;
    MotorControlCdd_CurrentSampleType payload;
} MotorControlCdd_CurrentSampleSlotType;

typedef struct __attribute__((aligned(4)))
{
    uint32 state;
    MotorControlCdd_FastSafetySnapshotType payload;
} MotorControlCdd_FastSafetySlotType;

typedef struct
{
    MotorControlFastLoop_Type fastLoop;
    MotorControlCdd_FastSafetySnapshotType initialSafety;
    Sc1_MeasurementIdentityType sc1Identity;
    MotorControlCdd_ControlIdentityType sc2Identity;
    sint32 acceptedRequestQ15;
    sint32 commissioningCapQ15;
    boolean commandContinueValid;
    boolean stopRequested;
    uint32 epoch;
} MotorControlCdd_ControlActivationType;

typedef struct __attribute__((aligned(4)))
{
    uint32 state;
    MotorControlCdd_ControlActivationType payload;
} MotorControlCdd_ControlStagingSlotType;

#define MOTOR_CONTROL_CDD_SAFE_PIN_MASK       \
    (GPIO_ODR_OD5 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15)

#define MOTOR_CONTROL_CDD_SAFE_PIN_RESET_MASK \
    (GPIO_BSRR_BR5 | GPIO_BSRR_BR13 | GPIO_BSRR_BR14 | GPIO_BSRR_BR15)

#define MOTOR_CONTROL_CDD_ENABLE_PIN_RESET_MASK \
    (GPIO_BSRR_BR13 | GPIO_BSRR_BR14 | GPIO_BSRR_BR15)

#define MOTOR_CONTROL_CDD_SAFE_MODE_MASK       \
    (GPIO_MODER_MODE5 | GPIO_MODER_MODE13 |    \
     GPIO_MODER_MODE14 | GPIO_MODER_MODE15)

#define MOTOR_CONTROL_CDD_SAFE_OUTPUT_MODE     \
    (GPIO_MODER_MODE5_0 | GPIO_MODER_MODE13_0 | \
     GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0)

#define MOTOR_CONTROL_CDD_SAFE_OUTPUT_TYPE_MASK \
    (GPIO_OTYPER_OT5 | GPIO_OTYPER_OT13 |       \
     GPIO_OTYPER_OT14 | GPIO_OTYPER_OT15)

#define MOTOR_CONTROL_CDD_SAFE_SPEED_MASK       \
    (GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED13 | \
     GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15)

#define MOTOR_CONTROL_CDD_SAFE_PULL_MASK         \
    (GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD13 |     \
     GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15)

#define MOTOR_CONTROL_CDD_SAFE_PULL_DOWN         \
    (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD13_1 | \
     GPIO_PUPDR_PUPD14_1 | GPIO_PUPDR_PUPD15_1)

#define MOTOR_CONTROL_CDD_GPIOA_MODE_MASK \
    (GPIO_MODER_MODE1 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | \
     GPIO_MODER_MODE10)
#define MOTOR_CONTROL_CDD_GPIOA_MODE_VALUE \
    (GPIO_MODER_MODE1 | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 | \
     GPIO_MODER_MODE10_1)
#define MOTOR_CONTROL_CDD_GPIOA_PULL_MASK \
    (GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9 | \
     GPIO_PUPDR_PUPD10)
#define MOTOR_CONTROL_CDD_GPIOA_SPEED_MASK \
    (GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9 | \
     GPIO_OSPEEDR_OSPEED10)
#define MOTOR_CONTROL_CDD_GPIOA_PWM_OUTPUT_TYPE_MASK \
    (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10)
#define MOTOR_CONTROL_CDD_GPIOA_AF_MASK \
    (GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10)
#define MOTOR_CONTROL_CDD_GPIOA_AF6_VALUE \
    ((6UL << GPIO_AFRH_AFSEL8_Pos) | (6UL << GPIO_AFRH_AFSEL9_Pos) | \
     (6UL << GPIO_AFRH_AFSEL10_Pos))

#define MOTOR_CONTROL_CDD_GPIOB_FULL_MODE_MASK \
    (GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE5 | \
     GPIO_MODER_MODE12 | GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | \
     GPIO_MODER_MODE15)
#define MOTOR_CONTROL_CDD_GPIOB_FULL_MODE_VALUE \
    (GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE5_0 | \
     GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_0 | \
     GPIO_MODER_MODE14_0 | GPIO_MODER_MODE15_0)
#define MOTOR_CONTROL_CDD_GPIOB_ANALOG_BREAK_PULL_MASK \
    (GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD12)
#define MOTOR_CONTROL_CDD_GPIOB_BREAK_PULL_UP GPIO_PUPDR_PUPD12_0
#define MOTOR_CONTROL_CDD_GPIOB_BREAK_OUTPUT_TYPE_MASK GPIO_OTYPER_OT12
#define MOTOR_CONTROL_CDD_GPIOB_BREAK_AF_MASK GPIO_AFRH_AFSEL12
#define MOTOR_CONTROL_CDD_GPIOB_BREAK_AF6_VALUE \
    (6UL << GPIO_AFRH_AFSEL12_Pos)

#define MOTOR_CONTROL_CDD_TIM1_CR1_VALUE \
    (TIM_CR1_CMS_0 | TIM_CR1_CKD_0)
#define MOTOR_CONTROL_CDD_TIM1_CCMR1_VALUE \
    ((6UL << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | \
     (6UL << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE)
#define MOTOR_CONTROL_CDD_TIM1_CCMR2_VALUE \
    ((6UL << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | \
     (7UL << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE)
#define MOTOR_CONTROL_CDD_TIM1_CCER_VALUE \
    (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E)
#define MOTOR_CONTROL_CDD_TIM1_BDTR_VALUE \
    (TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_BKE | \
     (4UL << TIM_BDTR_BKF_Pos))

#define MOTOR_CONTROL_CDD_ADC_ACTIVITY_MASK \
    (ADC_CR_ADDIS | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP | \
     ADC_CR_JADSTP | ADC_CR_ADCAL)
#define MOTOR_CONTROL_CDD_ADC_CHANNEL_DIFSEL_MASK \
    (ADC_DIFSEL_DIFSEL_2 | ADC_DIFSEL_DIFSEL_12 | \
     ADC_DIFSEL_DIFSEL_15)
#define MOTOR_CONTROL_CDD_ADC_SMPR1_MASK ADC_SMPR1_SMP2
#define MOTOR_CONTROL_CDD_ADC_SMPR1_VALUE \
    (1UL << ADC_SMPR1_SMP2_Pos)
#define MOTOR_CONTROL_CDD_ADC_SMPR2_MASK \
    (ADC_SMPR2_SMP12 | ADC_SMPR2_SMP15)
#define MOTOR_CONTROL_CDD_ADC_SMPR2_VALUE \
    ((1UL << ADC_SMPR2_SMP12_Pos) | (1UL << ADC_SMPR2_SMP15_Pos))
#define MOTOR_CONTROL_CDD_ADC_STATUS_CLEAR_MASK \
    (ADC_ISR_ADRDY | ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS | \
     ADC_ISR_OVR | ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_AWD1 | \
     ADC_ISR_AWD2 | ADC_ISR_AWD3 | ADC_ISR_JQOVF)

#define MOTOR_CONTROL_CDD_ADC_REGULAR_STATUS_MASK \
    (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR)
#define MOTOR_CONTROL_CDD_ADC_REGULAR_W1C_MASK \
    (ADC_ISR_EOS | ADC_ISR_OVR)
#define MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_MASK \
    (ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN | \
     ADC_CFGR_OVRMOD | ADC_CFGR_CONT | ADC_CFGR_DISCEN | \
     ADC_CFGR_DISCNUM)
#define MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_VALUE \
    (ADC_CFGR_OVRMOD | ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_0)
#define MOTOR_CONTROL_CDD_ADC_REGULAR_SQR1_VALUE \
    ((5UL << ADC_SQR1_L_Pos) | \
     ((uint32)MOTOR_CONTROL_CDD_ADC_CHANNEL_A << ADC_SQR1_SQ1_Pos) | \
     ((uint32)MOTOR_CONTROL_CDD_ADC_CHANNEL_A << ADC_SQR1_SQ2_Pos) | \
     ((uint32)MOTOR_CONTROL_CDD_ADC_CHANNEL_B << ADC_SQR1_SQ3_Pos) | \
     ((uint32)MOTOR_CONTROL_CDD_ADC_CHANNEL_B << ADC_SQR1_SQ4_Pos))
#define MOTOR_CONTROL_CDD_ADC_REGULAR_SQR2_VALUE \
    (((uint32)MOTOR_CONTROL_CDD_ADC_CHANNEL_C << ADC_SQR2_SQ5_Pos) | \
     ((uint32)MOTOR_CONTROL_CDD_ADC_CHANNEL_C << ADC_SQR2_SQ6_Pos))

#define MOTOR_CONTROL_CDD_TIM1_STATUS_CLEAR_MASK \
    (TIM_SR_UIF | TIM_SR_BIF)

#define MOTOR_CONTROL_CDD_EVENT_SEQUENCE_SHIFT 16U
#define MOTOR_CONTROL_CDD_EVENT_CAUSE_MASK     0x0000FFFFU

STATIC volatile boolean MotorControlCdd_EarlySafeInitialized = FALSE;
STATIC MotorControlCdd_InternalStateType MotorControlCdd_State =
    MOTOR_CONTROL_CDD_UNINITIALIZED;
STATIC uint32 MotorControlCdd_BreakEvent;
STATIC uint32 MotorControlCdd_UpdateEvent;
STATIC uint32 MotorControlCdd_AdcEvent;
STATIC uint32 MotorControlCdd_CalibrationEvent;
STATIC uint32 MotorControlCdd_ControlEvent;
STATIC MotorControlCdd_CurrentOffsetsType MotorControlCdd_CurrentOffsets;
STATIC boolean MotorControlCdd_CurrentOffsetsValid;
STATIC MotorControlCdd_RuntimeGenerationType MotorControlCdd_PreparedGeneration;
STATIC MotorControlCdd_RuntimeGenerationType MotorControlCdd_QueuedGeneration;
STATIC boolean MotorControlCdd_PreparedGenerationValid;
STATIC boolean MotorControlCdd_QueuedGenerationValid;
STATIC boolean MotorControlCdd_DiscardNextSample;
STATIC uint32 MotorControlCdd_ConsumedGenerationId;
STATIC MotorControlCdd_CurrentSampleSlotType
    MotorControlCdd_CurrentSampleMailbox[MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT];
STATIC uint32 MotorControlCdd_SampleMailboxWriteCursor;
STATIC uint32 MotorControlCdd_SampleDropCount;
STATIC MotorControlCdd_FastSafetySlotType
    MotorControlCdd_FastSafetyMailbox[MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT];
STATIC uint32 MotorControlCdd_FastSafetyWriteCursor;
STATIC MotorControlCdd_FastSafetySnapshotType MotorControlCdd_RetainedFastSafety;
STATIC boolean MotorControlCdd_RetainedFastSafetyValid;
STATIC MotorControlCdd_ControlConfigType MotorControlCdd_ActiveControlConfig;
STATIC MotorControlCdd_ControlStagingSlotType MotorControlCdd_ControlStaging;
STATIC MotorControlCdd_ControlActivationType MotorControlCdd_LiveControl;
STATIC uint32 MotorControlCdd_ControlLifecycleEpoch;
STATIC MotorControlCdd_ControlFailureType MotorControlCdd_ControlFailure;
STATIC sint32 MotorControlCdd_AcceptedRequestQ15;
STATIC sint32 MotorControlCdd_CommissioningCapQ15;
STATIC MotorControlCdd_CalibrationStateType MotorControlCdd_CalibrationState;
STATIC MotorControlCdd_CalibrationFailureType
    MotorControlCdd_CalibrationFailure;
STATIC MotorControlCdd_CalibrationConfigType
    MotorControlCdd_ActiveCalibrationConfig;
STATIC MotorControlCdd_CalibrationBaselineType
    MotorControlCdd_CalibrationBaseline;
STATIC MotorControlCdd_CalibrationResultType MotorControlCdd_CalibrationResult;
STATIC uint32 MotorControlCdd_CalibrationSum[
    MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT];
STATIC uint16 MotorControlCdd_CalibrationMinimum[
    MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT];
STATIC uint16 MotorControlCdd_CalibrationMaximum[
    MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT];
STATIC uint16 MotorControlCdd_CalibrationCount[
    MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT];
STATIC uint32 MotorControlCdd_CalibrationAcceptedSamples;
STATIC boolean MotorControlCdd_CalibrationEnvironmentEstablished;

LOCAL_INLINE boolean MotorControlCdd_IsEarlySafeStateValid(void);
STATIC Std_ReturnType MotorControlCdd_InstallCurrentOffsets(
    const MotorControlCdd_CurrentOffsetsType *Offsets);

_Static_assert(__atomic_always_lock_free(sizeof(uint32), 0),
               "32-bit motor-control atomics must be lock-free");
_Static_assert(__alignof__(MotorControlCdd_CurrentSampleSlotType) >= 4U,
               "current-sample mailbox slots must be 32-bit aligned");
_Static_assert(__alignof__(MotorControlCdd_FastSafetySlotType) >= 4U,
               "fast-safety mailbox slots must be 32-bit aligned");
_Static_assert(__alignof__(MotorControlCdd_ControlStagingSlotType) >= 4U,
               "control staging must be 32-bit aligned");
_Static_assert(MOTOR_CONTROL_CDD_CALIBRATION_TOTAL_SAMPLES == 192U,
               "SC-1 calibration must accept exactly 192 phase samples");

STATIC boolean MotorControlCdd_IsCanonicalBoolean(boolean value)
{
    return ((value == FALSE) || (value == TRUE)) ? TRUE : FALSE;
}

STATIC MotorControlCdd_ControlLifecycleType
MotorControlCdd_LoadControlLifecycle(void)
{
    uint32 value = __atomic_load_n(&MotorControlCdd_ControlLifecycleEpoch,
                                   __ATOMIC_ACQUIRE);

    return (MotorControlCdd_ControlLifecycleType)
        (value & MOTOR_CONTROL_CDD_CONTROL_LIFECYCLE_MASK);
}

STATIC uint32 MotorControlCdd_LoadControlEpoch(void)
{
    return __atomic_load_n(&MotorControlCdd_ControlLifecycleEpoch,
                           __ATOMIC_ACQUIRE) >>
           MOTOR_CONTROL_CDD_CONTROL_EPOCH_SHIFT;
}

STATIC void MotorControlCdd_StoreControlLifecycle(
    MotorControlCdd_ControlLifecycleType lifecycle,
    uint32 epoch)
{
    uint32 value = (epoch << MOTOR_CONTROL_CDD_CONTROL_EPOCH_SHIFT) |
                   (uint32)lifecycle;

    __atomic_store_n(&MotorControlCdd_ControlLifecycleEpoch, value,
                     __ATOMIC_RELEASE);
}

STATIC uint32 MotorControlCdd_AbsS32(sint32 value)
{
    return (value >= 0) ? (uint32)value : (uint32)(-(sint64)value);
}

STATIC Std_ReturnType MotorControlCdd_RoundPhysicalMaToQ15(
    sint32 valueMa,
    uint32 currentBaseMa,
    sint32 *valueQ15)
{
    sint64 numerator;
    sint64 rounded;

    if ((valueQ15 == NULL_PTR) || (currentBaseMa <= 300U))
    {
        return E_NOT_OK;
    }
    numerator = (sint64)valueMa * MOTOR_CONTROL_CDD_Q15_SCALE;
    if (numerator >= 0)
    {
        rounded = (numerator + ((sint64)currentBaseMa / 2LL)) /
                  (sint64)currentBaseMa;
    }
    else
    {
        rounded = -(((-numerator) + ((sint64)currentBaseMa / 2LL)) /
                    (sint64)currentBaseMa);
    }
    if ((rounded < MOTOR_CONTROL_CDD_Q15_MIN) ||
        (rounded > MOTOR_CONTROL_CDD_Q15_MAX))
    {
        return E_NOT_OK;
    }
    *valueQ15 = (sint32)rounded;
    return E_OK;
}

STATIC boolean MotorControlCdd_ControlApprovalsComplete(void)
{
    return ((MotorControlCdd_ActiveControlConfig.currentScaleApproved == TRUE) &&
            (MotorControlCdd_ActiveControlConfig.controlParametersApproved ==
             TRUE) &&
            (MotorControlCdd_ActiveControlConfig.samplingTimingApproved ==
             TRUE)) ? TRUE : FALSE;
}

LOCAL_INLINE void MotorControlCdd_PreloadSafeOutputsLow(void)
{
    GPIOB->BSRR = MOTOR_CONTROL_CDD_SAFE_PIN_RESET_MASK;
}

LOCAL_INLINE void MotorControlCdd_ClearMainOutputEnable(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

LOCAL_INLINE void MotorControlCdd_GateAdcTrigger(void)
{
    TIM1->CR2 &= ~TIM_CR2_MMS;
}

STATIC void MotorControlCdd_ClearAdcStatus(uint32 mask)
{
    ADC1->ISR = mask;
    MOTOR_CONTROL_CDD_TEST_ADC_STATUS_CLEAR_HOOK(mask);
}

STATIC void MotorControlCdd_ClearTimStatus(uint32 mask)
{
    TIM1->SR = ~mask;
    MOTOR_CONTROL_CDD_TEST_TIM_STATUS_CLEAR_HOOK(mask);
}

STATIC void MotorControlCdd_DelayAtLeastHclkCycles(uint32 cycleCount)
{
    volatile uint32 cyclesRemaining = cycleCount;

    while (cyclesRemaining > 0U)
    {
        __NOP();
        cyclesRemaining--;
    }
}

STATIC Std_ReturnType MotorControlCdd_WaitForAdcTransition(uint32 testStage)
{
    uint32 observation;

    for (observation = 0U;
         observation < MOTOR_CONTROL_CDD_ADC_TIMEOUT_OBSERVATIONS;
         observation++)
    {
        MOTOR_CONTROL_CDD_TEST_WAIT_HOOK(testStage, observation);
        if ((testStage == MOTOR_CONTROL_CDD_TEST_WAIT_CALIBRATION) &&
            ((ADC1->CR & ADC_CR_ADCAL) == 0U))
        {
            return E_OK;
        }
        if ((testStage == MOTOR_CONTROL_CDD_TEST_WAIT_READY) &&
            ((ADC1->ISR & ADC_ISR_ADRDY) != 0U))
        {
            return E_OK;
        }
    }

    return E_NOT_OK;
}

STATIC boolean MotorControlCdd_AreClockPreconditionsValid(void)
{
    if (((RCC->AHB2ENR & MOTOR_CONTROL_CDD_REQUIRED_AHB2_GATES) ==
         MOTOR_CONTROL_CDD_REQUIRED_AHB2_GATES) &&
        ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U) &&
        ((RCC->CCIPR & RCC_CCIPR_ADC12SEL) == RCC_CCIPR_ADC12SEL_0) &&
        ((RCC->AHB2RSTR & RCC_AHB2RSTR_ADC12RST) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC void MotorControlCdd_ConfigureOwnedPins(void)
{
    GPIOA->OTYPER &= ~MOTOR_CONTROL_CDD_GPIOA_PWM_OUTPUT_TYPE_MASK;
    GPIOA->OSPEEDR =
        (GPIOA->OSPEEDR & ~MOTOR_CONTROL_CDD_GPIOA_SPEED_MASK) |
        MOTOR_CONTROL_CDD_GPIOA_SPEED_MASK;
    GPIOA->PUPDR &= ~MOTOR_CONTROL_CDD_GPIOA_PULL_MASK;
    GPIOA->AFR[1] =
        (GPIOA->AFR[1] & ~MOTOR_CONTROL_CDD_GPIOA_AF_MASK) |
        MOTOR_CONTROL_CDD_GPIOA_AF6_VALUE;
    GPIOA->MODER =
        (GPIOA->MODER & ~MOTOR_CONTROL_CDD_GPIOA_MODE_MASK) |
        MOTOR_CONTROL_CDD_GPIOA_MODE_VALUE;

    GPIOB->OTYPER &= ~MOTOR_CONTROL_CDD_GPIOB_BREAK_OUTPUT_TYPE_MASK;
    GPIOB->PUPDR =
        (GPIOB->PUPDR & ~MOTOR_CONTROL_CDD_GPIOB_ANALOG_BREAK_PULL_MASK) |
        MOTOR_CONTROL_CDD_GPIOB_BREAK_PULL_UP;
    GPIOB->AFR[1] =
        (GPIOB->AFR[1] & ~MOTOR_CONTROL_CDD_GPIOB_BREAK_AF_MASK) |
        MOTOR_CONTROL_CDD_GPIOB_BREAK_AF6_VALUE;
    GPIOB->MODER =
        (GPIOB->MODER & ~MOTOR_CONTROL_CDD_GPIOB_FULL_MODE_MASK) |
        MOTOR_CONTROL_CDD_GPIOB_FULL_MODE_VALUE;
}

STATIC void MotorControlCdd_ConfigureTimerSafeBaseline(void)
{
    MotorControlCdd_ClearMainOutputEnable();
    MotorControlCdd_GateAdcTrigger();

    TIM1->CR1 = MOTOR_CONTROL_CDD_TIM1_CR1_VALUE;
    TIM1->CR2 = 0U;
    TIM1->SMCR = 0U;
    TIM1->DIER = 0U;
    TIM1->CCMR1 = MOTOR_CONTROL_CDD_TIM1_CCMR1_VALUE;
    TIM1->CCMR2 = MOTOR_CONTROL_CDD_TIM1_CCMR2_VALUE;
    TIM1->CCER = MOTOR_CONTROL_CDD_TIM1_CCER_VALUE;
    TIM1->PSC = 0U;
    TIM1->ARR = MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS;
    TIM1->RCR = 1U;
    TIM1->CCR1 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    TIM1->CCR2 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    TIM1->CCR3 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    TIM1->CCR4 = MOTOR_CONTROL_CDD_INITIAL_SAMPLE_COUNT;
    TIM1->BDTR = MOTOR_CONTROL_CDD_TIM1_BDTR_VALUE;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    MotorControlCdd_ClearTimStatus(MOTOR_CONTROL_CDD_TIM1_STATUS_CLEAR_MASK);
}

STATIC Std_ReturnType MotorControlCdd_ConfigureAdc1(void)
{
    if ((ADC1->CR & (ADC_CR_ADEN | MOTOR_CONTROL_CDD_ADC_ACTIVITY_MASK)) != 0U)
    {
        return E_NOT_OK;
    }

    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    MotorControlCdd_DelayAtLeastHclkCycles(
        MOTOR_CONTROL_CDD_ADC_REGULATOR_DELAY_HCLK_CYCLES);

    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    if (MotorControlCdd_WaitForAdcTransition(
            MOTOR_CONTROL_CDD_TEST_WAIT_CALIBRATION) != E_OK)
    {
        return E_NOT_OK;
    }

    MotorControlCdd_DelayAtLeastHclkCycles(
        MOTOR_CONTROL_CDD_ADC_POST_CALIBRATION_HCLK_CYCLES);
    ADC1->IER = 0U;
    ADC1->CFGR = ADC_CFGR_JQM;
    ADC1->CFGR2 = 0U;
    ADC1->SMPR1 =
        (ADC1->SMPR1 & ~MOTOR_CONTROL_CDD_ADC_SMPR1_MASK) |
        MOTOR_CONTROL_CDD_ADC_SMPR1_VALUE;
    ADC1->SMPR2 =
        (ADC1->SMPR2 & ~MOTOR_CONTROL_CDD_ADC_SMPR2_MASK) |
        MOTOR_CONTROL_CDD_ADC_SMPR2_VALUE;
    ADC1->SQR1 = 0U;
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = 0U;
    ADC1->SQR4 = 0U;
    ADC1->DIFSEL &= ~MOTOR_CONTROL_CDD_ADC_CHANNEL_DIFSEL_MASK;
    MotorControlCdd_ClearAdcStatus(MOTOR_CONTROL_CDD_ADC_STATUS_CLEAR_MASK);

    ADC1->CR |= ADC_CR_ADEN;
    if (MotorControlCdd_WaitForAdcTransition(
            MOTOR_CONTROL_CDD_TEST_WAIT_READY) != E_OK)
    {
        return E_NOT_OK;
    }
    MotorControlCdd_ClearAdcStatus(ADC_ISR_ADRDY);

    return E_OK;
}

STATIC boolean MotorControlCdd_IsOwnedPinConfigurationValid(void)
{
    if (((GPIOA->MODER & MOTOR_CONTROL_CDD_GPIOA_MODE_MASK) ==
         MOTOR_CONTROL_CDD_GPIOA_MODE_VALUE) &&
        ((GPIOA->OTYPER & MOTOR_CONTROL_CDD_GPIOA_PWM_OUTPUT_TYPE_MASK) == 0U) &&
        ((GPIOA->OSPEEDR & MOTOR_CONTROL_CDD_GPIOA_SPEED_MASK) ==
         MOTOR_CONTROL_CDD_GPIOA_SPEED_MASK) &&
        ((GPIOA->PUPDR & MOTOR_CONTROL_CDD_GPIOA_PULL_MASK) == 0U) &&
        ((GPIOA->AFR[1] & MOTOR_CONTROL_CDD_GPIOA_AF_MASK) ==
         MOTOR_CONTROL_CDD_GPIOA_AF6_VALUE) &&
        ((GPIOB->MODER & MOTOR_CONTROL_CDD_GPIOB_FULL_MODE_MASK) ==
         MOTOR_CONTROL_CDD_GPIOB_FULL_MODE_VALUE) &&
        ((GPIOB->OTYPER & MOTOR_CONTROL_CDD_GPIOB_BREAK_OUTPUT_TYPE_MASK) == 0U) &&
        ((GPIOB->PUPDR & MOTOR_CONTROL_CDD_GPIOB_ANALOG_BREAK_PULL_MASK) ==
         MOTOR_CONTROL_CDD_GPIOB_BREAK_PULL_UP) &&
        ((GPIOB->AFR[1] & MOTOR_CONTROL_CDD_GPIOB_BREAK_AF_MASK) ==
         MOTOR_CONTROL_CDD_GPIOB_BREAK_AF6_VALUE))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean MotorControlCdd_IsTimerStaticConfigurationValid(void)
{
    boolean compareValuesValid = FALSE;

    if (((MotorControlCdd_PreparedGenerationValid == FALSE) &&
         (TIM1->CCR1 == MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS) &&
         (TIM1->CCR2 == MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS) &&
         (TIM1->CCR3 == MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS) &&
         (TIM1->CCR4 == MOTOR_CONTROL_CDD_INITIAL_SAMPLE_COUNT)) ||
        ((MotorControlCdd_PreparedGenerationValid == TRUE) &&
         (TIM1->CCR1 == MotorControlCdd_PreparedGeneration.generation.ccr1) &&
         (TIM1->CCR2 == MotorControlCdd_PreparedGeneration.generation.ccr2) &&
         (TIM1->CCR3 == MotorControlCdd_PreparedGeneration.generation.ccr3) &&
         (TIM1->CCR4 == MotorControlCdd_PreparedGeneration.generation.ccr4)))
    {
        compareValuesValid = TRUE;
    }

    if ((TIM1->CR1 == MOTOR_CONTROL_CDD_TIM1_CR1_VALUE) &&
        ((TIM1->CR2 & TIM_CR2_MMS) == 0U) &&
        (TIM1->SMCR == 0U) &&
        (TIM1->DIER == TIM_DIER_BIE) &&
        (TIM1->CCMR1 == MOTOR_CONTROL_CDD_TIM1_CCMR1_VALUE) &&
        (TIM1->CCMR2 == MOTOR_CONTROL_CDD_TIM1_CCMR2_VALUE) &&
        (TIM1->CCER == MOTOR_CONTROL_CDD_TIM1_CCER_VALUE) &&
        (TIM1->PSC == 0U) &&
        (TIM1->ARR == MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS) &&
        (TIM1->RCR == 1U) &&
        (compareValuesValid == TRUE) &&
        (TIM1->BDTR == MOTOR_CONTROL_CDD_TIM1_BDTR_VALUE) &&
        ((TIM1->SR & MOTOR_CONTROL_CDD_TIM1_STATUS_CLEAR_MASK) == 0U) &&
        (TIM1->CNT == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean MotorControlCdd_IsAdc1StaticConfigurationValid(void)
{
    if ((ADC1->IER == 0U) &&
        (ADC1->CFGR == ADC_CFGR_JQM) &&
        (ADC1->CFGR2 == 0U) &&
        ((ADC1->SMPR1 & MOTOR_CONTROL_CDD_ADC_SMPR1_MASK) ==
         MOTOR_CONTROL_CDD_ADC_SMPR1_VALUE) &&
        ((ADC1->SMPR2 & MOTOR_CONTROL_CDD_ADC_SMPR2_MASK) ==
         MOTOR_CONTROL_CDD_ADC_SMPR2_VALUE) &&
        ((ADC1->DIFSEL & MOTOR_CONTROL_CDD_ADC_CHANNEL_DIFSEL_MASK) == 0U) &&
        (ADC1->SQR1 == 0U) &&
        (ADC1->SQR2 == 0U) &&
        (ADC1->SQR3 == 0U) &&
        (ADC1->SQR4 == 0U) &&
        (ADC1->JSQR == 0U) &&
        ((ADC1->CR & ADC_CR_ADEN) != 0U) &&
        ((ADC1->CR & ADC_CR_ADVREGEN) != 0U) &&
        ((ADC1->CR & (ADC_CR_DEEPPWD | MOTOR_CONTROL_CDD_ADC_ACTIVITY_MASK)) ==
         0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean MotorControlCdd_IsCompleteStaticConfigurationValid(void)
{
    if ((MotorControlCdd_AreClockPreconditionsValid() == TRUE) &&
        (Adc_CheckCommonConfiguration() == E_OK) &&
        (MotorControlCdd_IsEarlySafeStateValid() == TRUE) &&
        (MotorControlCdd_IsOwnedPinConfigurationValid() == TRUE) &&
        (MotorControlCdd_IsTimerStaticConfigurationValid() == TRUE) &&
        (MotorControlCdd_IsAdc1StaticConfigurationValid() == TRUE))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC void MotorControlCdd_PublishEvent(uint32 *eventWord, uint16 cause)
{
    uint32 previous = __atomic_load_n(eventWord, __ATOMIC_RELAXED);
    uint32 sequence = (previous >> MOTOR_CONTROL_CDD_EVENT_SEQUENCE_SHIFT) + 1U;
    uint32 event = (sequence << MOTOR_CONTROL_CDD_EVENT_SEQUENCE_SHIFT) |
                   ((uint32)cause & MOTOR_CONTROL_CDD_EVENT_CAUSE_MASK);

    __atomic_store_n(eventWord, event, __ATOMIC_RELEASE);
    StateManager_NotifyCriticalFault(
        STATE_MANAGER_CRITICAL_CLASS_MOTOR);
}

LOCAL_INLINE void MotorControlCdd_ConfigureEarlySafePins(void)
{
    GPIOB->OTYPER &= ~MOTOR_CONTROL_CDD_SAFE_OUTPUT_TYPE_MASK;
    GPIOB->OSPEEDR &= ~MOTOR_CONTROL_CDD_SAFE_SPEED_MASK;
    GPIOB->PUPDR =
        (GPIOB->PUPDR & ~MOTOR_CONTROL_CDD_SAFE_PULL_MASK) |
        MOTOR_CONTROL_CDD_SAFE_PULL_DOWN;

    /* MODER is written last so the already-low latch becomes externally active. */
    GPIOB->MODER =
        (GPIOB->MODER & ~MOTOR_CONTROL_CDD_SAFE_MODE_MASK) |
        MOTOR_CONTROL_CDD_SAFE_OUTPUT_MODE;
}

LOCAL_INLINE boolean MotorControlCdd_IsEarlySafeStateValid(void)
{
    uint32 modeState;
    uint32 outputTypeState;
    uint32 pullState;
    uint32 speedState;
    uint32 outputState;
    uint32 mainOutputState;

    modeState = GPIOB->MODER;
    outputTypeState = GPIOB->OTYPER;
    pullState = GPIOB->PUPDR;
    speedState = GPIOB->OSPEEDR;
    outputState = GPIOB->ODR;
    mainOutputState = TIM1->BDTR;

    if (((modeState & MOTOR_CONTROL_CDD_SAFE_MODE_MASK) ==
         MOTOR_CONTROL_CDD_SAFE_OUTPUT_MODE) &&
        ((outputTypeState & MOTOR_CONTROL_CDD_SAFE_OUTPUT_TYPE_MASK) == 0U) &&
        ((pullState & MOTOR_CONTROL_CDD_SAFE_PULL_MASK) ==
         MOTOR_CONTROL_CDD_SAFE_PULL_DOWN) &&
        ((speedState & MOTOR_CONTROL_CDD_SAFE_SPEED_MASK) == 0U) &&
        ((outputState & MOTOR_CONTROL_CDD_SAFE_PIN_MASK) == 0U) &&
        ((mainOutputState & TIM_BDTR_MOE) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean MotorControlCdd_AreFaultEventsClear(void)
{
    if ((__atomic_load_n(&MotorControlCdd_BreakEvent, __ATOMIC_ACQUIRE) == 0U) &&
        (__atomic_load_n(&MotorControlCdd_UpdateEvent, __ATOMIC_ACQUIRE) == 0U) &&
        (__atomic_load_n(&MotorControlCdd_AdcEvent, __ATOMIC_ACQUIRE) == 0U) &&
        (__atomic_load_n(&MotorControlCdd_CalibrationEvent,
                         __ATOMIC_ACQUIRE) == 0U) &&
        (__atomic_load_n(&MotorControlCdd_ControlEvent,
                         __ATOMIC_ACQUIRE) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean MotorControlCdd_IdentityIsEqual(
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

STATIC void MotorControlCdd_ResetCalibrationData(void)
{
    uint32 phase;

    MotorControlCdd_CalibrationResult =
        (MotorControlCdd_CalibrationResultType){ 0 };
    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        MotorControlCdd_CalibrationSum[phase] = 0U;
        MotorControlCdd_CalibrationMinimum[phase] =
            MOTOR_CONTROL_CDD_ADC_RAW_MASK;
        MotorControlCdd_CalibrationMaximum[phase] = 0U;
        MotorControlCdd_CalibrationCount[phase] = 0U;
    }
    MotorControlCdd_CalibrationAcceptedSamples = 0U;
    MotorControlCdd_CalibrationEnvironmentEstablished = FALSE;
}

STATIC void MotorControlCdd_FailCalibration(
    MotorControlCdd_CalibrationFailureType failure,
    boolean adcCleanupRequired)
{
    if (MotorControlCdd_CalibrationFailure ==
        MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE)
    {
        MotorControlCdd_CalibrationFailure = failure;
    }

    MotorControlCdd_EmergencyOff();
    MotorControlCdd_GateAdcTrigger();
    MotorControlCdd_CurrentOffsetsValid = FALSE;
    MotorControlCdd_CalibrationResult.offsetsInstalled = FALSE;
    MotorControlCdd_PublishEvent(
        &MotorControlCdd_CalibrationEvent,
        MOTOR_CONTROL_CDD_EVENT_CAUSE_CALIBRATION);
    if (adcCleanupRequired == TRUE)
    {
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_REQUEST;
    }
    else
    {
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_FAILED_SAFE;
    }
}

STATIC boolean MotorControlCdd_UpdateCalibrationEnvironment(
    const MotorControlCdd_CalibrationEnvironmentType *environment)
{
    if ((environment == NULL_PTR) ||
        (environment->vbusRaw > MOTOR_CONTROL_CDD_ADC_RAW_MASK) ||
        (environment->temperatureRaw > MOTOR_CONTROL_CDD_ADC_RAW_MASK) ||
        (MotorControlCdd_IdentityIsEqual(
             &environment->identity,
             &MotorControlCdd_ActiveCalibrationConfig.identity) != TRUE))
    {
        return FALSE;
    }

    if (MotorControlCdd_CalibrationEnvironmentEstablished != TRUE)
    {
        if (environment->safetyAcquisitionSequence == 0U)
        {
            return FALSE;
        }
        MotorControlCdd_CalibrationResult.firstSafetySequence =
            environment->safetyAcquisitionSequence;
        MotorControlCdd_CalibrationResult.firstSafetyTick =
            environment->safetyTick;
        MotorControlCdd_CalibrationResult.vbusRawMinimum =
            environment->vbusRaw;
        MotorControlCdd_CalibrationResult.vbusRawMaximum =
            environment->vbusRaw;
        MotorControlCdd_CalibrationResult.temperatureRawMinimum =
            environment->temperatureRaw;
        MotorControlCdd_CalibrationResult.temperatureRawMaximum =
            environment->temperatureRaw;
        MotorControlCdd_CalibrationEnvironmentEstablished = TRUE;
    }
    else if (environment->safetyAcquisitionSequence !=
             (MotorControlCdd_CalibrationResult.lastSafetySequence + 1U))
    {
        return FALSE;
    }

    MotorControlCdd_CalibrationResult.lastSafetySequence =
        environment->safetyAcquisitionSequence;
    MotorControlCdd_CalibrationResult.lastSafetyTick = environment->safetyTick;
    if (environment->vbusRaw <
        MotorControlCdd_CalibrationResult.vbusRawMinimum)
    {
        MotorControlCdd_CalibrationResult.vbusRawMinimum = environment->vbusRaw;
    }
    if (environment->vbusRaw >
        MotorControlCdd_CalibrationResult.vbusRawMaximum)
    {
        MotorControlCdd_CalibrationResult.vbusRawMaximum = environment->vbusRaw;
    }
    if (environment->temperatureRaw <
        MotorControlCdd_CalibrationResult.temperatureRawMinimum)
    {
        MotorControlCdd_CalibrationResult.temperatureRawMinimum =
            environment->temperatureRaw;
    }
    if (environment->temperatureRaw >
        MotorControlCdd_CalibrationResult.temperatureRawMaximum)
    {
        MotorControlCdd_CalibrationResult.temperatureRawMaximum =
            environment->temperatureRaw;
    }
    return TRUE;
}

STATIC boolean MotorControlCdd_RecordCalibrationSample(
    uint32 phase,
    uint16 sample)
{
    const MotorControlCdd_CalibrationPhaseBoundsType *bounds;

    if ((phase >= MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT) ||
        (MotorControlCdd_CalibrationCount[phase] >=
         MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT))
    {
        return FALSE;
    }

    bounds = &MotorControlCdd_ActiveCalibrationConfig.phase[phase];
    if ((sample < bounds->absoluteRailMinimum) ||
        (sample > bounds->absoluteRailMaximum))
    {
        return FALSE;
    }
    MotorControlCdd_CalibrationSum[phase] += (uint32)sample;
    if (sample < MotorControlCdd_CalibrationMinimum[phase])
    {
        MotorControlCdd_CalibrationMinimum[phase] = sample;
    }
    if (sample > MotorControlCdd_CalibrationMaximum[phase])
    {
        MotorControlCdd_CalibrationMaximum[phase] = sample;
    }
    MotorControlCdd_CalibrationCount[phase]++;
    MotorControlCdd_CalibrationAcceptedSamples++;
    return TRUE;
}

STATIC boolean MotorControlCdd_RestoreCalibrationBaseline(void)
{
    if (MotorControlCdd_CalibrationBaseline.valid != TRUE)
    {
        return FALSE;
    }

    ADC1->CFGR =
        (ADC1->CFGR & ~MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_MASK) |
        (MotorControlCdd_CalibrationBaseline.cfgr &
         MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_MASK);
    ADC1->SMPR1 =
        (ADC1->SMPR1 & ~MOTOR_CONTROL_CDD_ADC_SMPR1_MASK) |
        (MotorControlCdd_CalibrationBaseline.smpr1 &
         MOTOR_CONTROL_CDD_ADC_SMPR1_MASK);
    ADC1->SMPR2 =
        (ADC1->SMPR2 & ~MOTOR_CONTROL_CDD_ADC_SMPR2_MASK) |
        (MotorControlCdd_CalibrationBaseline.smpr2 &
         MOTOR_CONTROL_CDD_ADC_SMPR2_MASK);
    ADC1->SQR1 = MotorControlCdd_CalibrationBaseline.sqr1;
    ADC1->SQR2 = MotorControlCdd_CalibrationBaseline.sqr2;
    ADC1->SQR3 = MotorControlCdd_CalibrationBaseline.sqr3;
    ADC1->SQR4 = MotorControlCdd_CalibrationBaseline.sqr4;
    return TRUE;
}

STATIC boolean MotorControlCdd_IsCalibrationConfigurationValid(void)
{
    if ((MotorControlCdd_CalibrationBaseline.valid == TRUE) &&
        (ADC1->CFGR ==
         ((MotorControlCdd_CalibrationBaseline.cfgr &
           ~MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_MASK) |
          MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_VALUE)) &&
        (ADC1->SMPR1 ==
         ((MotorControlCdd_CalibrationBaseline.smpr1 &
           ~MOTOR_CONTROL_CDD_ADC_SMPR1_MASK) |
          MOTOR_CONTROL_CDD_ADC_SMPR1_VALUE)) &&
        (ADC1->SMPR2 ==
         ((MotorControlCdd_CalibrationBaseline.smpr2 &
           ~MOTOR_CONTROL_CDD_ADC_SMPR2_MASK) |
          MOTOR_CONTROL_CDD_ADC_SMPR2_VALUE)) &&
        (ADC1->SQR1 == MOTOR_CONTROL_CDD_ADC_REGULAR_SQR1_VALUE) &&
        (ADC1->SQR2 == MOTOR_CONTROL_CDD_ADC_REGULAR_SQR2_VALUE) &&
        (ADC1->SQR3 == 0U) &&
        (ADC1->SQR4 == 0U) &&
        (ADC1->IER == 0U) &&
        (ADC1->JSQR == 0U))
    {
        return TRUE;
    }
    return FALSE;
}

STATIC boolean MotorControlCdd_IsCalibrationBaselineRestored(void)
{
    if ((MotorControlCdd_CalibrationBaseline.valid == TRUE) &&
        (ADC1->CFGR == MotorControlCdd_CalibrationBaseline.cfgr) &&
        (ADC1->SMPR1 == MotorControlCdd_CalibrationBaseline.smpr1) &&
        (ADC1->SMPR2 == MotorControlCdd_CalibrationBaseline.smpr2) &&
        (ADC1->SQR1 == MotorControlCdd_CalibrationBaseline.sqr1) &&
        (ADC1->SQR2 == MotorControlCdd_CalibrationBaseline.sqr2) &&
        (ADC1->SQR3 == MotorControlCdd_CalibrationBaseline.sqr3) &&
        (ADC1->SQR4 == MotorControlCdd_CalibrationBaseline.sqr4) &&
        ((ADC1->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP)) == 0U) &&
        ((ADC1->ISR & MOTOR_CONTROL_CDD_ADC_REGULAR_STATUS_MASK) == 0U) &&
        (ADC1->JSQR == 0U) &&
        ((TIM1->CR1 & TIM_CR1_CEN) == 0U) &&
        ((TIM1->CR2 & TIM_CR2_MMS) == 0U))
    {
        return TRUE;
    }
    return FALSE;
}

STATIC boolean MotorControlCdd_ValidateCalibrationStatistics(void)
{
    MotorControlCdd_CurrentOffsetsType offsets;
    uint32 phase;

    if (MotorControlCdd_CalibrationAcceptedSamples !=
        MOTOR_CONTROL_CDD_CALIBRATION_TOTAL_SAMPLES)
    {
        return FALSE;
    }
    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        uint32 roundedMean;
        uint16 peakToPeak;
        const MotorControlCdd_CalibrationPhaseBoundsType *bounds =
            &MotorControlCdd_ActiveCalibrationConfig.phase[phase];

        if (MotorControlCdd_CalibrationCount[phase] !=
            MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT)
        {
            return FALSE;
        }
        roundedMean =
            (MotorControlCdd_CalibrationSum[phase] +
             (MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT / 2U)) /
            MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT;
        peakToPeak = (uint16)(MotorControlCdd_CalibrationMaximum[phase] -
                              MotorControlCdd_CalibrationMinimum[phase]);
        MotorControlCdd_CalibrationResult.phase[phase].minimum =
            MotorControlCdd_CalibrationMinimum[phase];
        MotorControlCdd_CalibrationResult.phase[phase].maximum =
            MotorControlCdd_CalibrationMaximum[phase];
        MotorControlCdd_CalibrationResult.phase[phase].mean =
            (uint16)roundedMean;
        MotorControlCdd_CalibrationResult.phase[phase].peakToPeak = peakToPeak;

        if ((MotorControlCdd_ActiveCalibrationConfig.measurementApproved == TRUE) &&
            ((roundedMean < bounds->meanMinimum) ||
             (roundedMean > bounds->meanMaximum) ||
             (peakToPeak > bounds->maximumPeakToPeak)))
        {
            return FALSE;
        }
    }

    MotorControlCdd_CalibrationResult.identity =
        MotorControlCdd_ActiveCalibrationConfig.identity;
    MotorControlCdd_CalibrationResult.measurementApproved =
        MotorControlCdd_ActiveCalibrationConfig.measurementApproved;
    MotorControlCdd_CalibrationResult.offsetsInstalled = FALSE;
    if (MotorControlCdd_ActiveCalibrationConfig.measurementApproved == TRUE)
    {
        offsets.phaseA = MotorControlCdd_CalibrationResult.phase[0].mean;
        offsets.phaseB = MotorControlCdd_CalibrationResult.phase[1].mean;
        offsets.phaseC = MotorControlCdd_CalibrationResult.phase[2].mean;
        if (MotorControlCdd_InstallCurrentOffsets(&offsets) != E_OK)
        {
            return FALSE;
        }
        MotorControlCdd_CalibrationResult.offsetsInstalled = TRUE;
    }
    MotorControlCdd_CalibrationResult.valid = TRUE;
    return TRUE;
}

STATIC boolean MotorControlCdd_IsGenerationInputValid(
    const MotorControlCdd_GenerationType *generation)
{
    boolean phaseSelectionValid = FALSE;

    if (generation != NULL_PTR)
    {
        if ((generation->rank1Phase == MOTOR_CONTROL_CDD_PHASE_A) &&
            (generation->rank2Phase == MOTOR_CONTROL_CDD_PHASE_B) &&
            (generation->reconstructedPhase == MOTOR_CONTROL_CDD_PHASE_C))
        {
            /* The common A/B selection is valid independently of sector. */
            phaseSelectionValid = TRUE;
        }
        else if ((generation->rank1Phase == MOTOR_CONTROL_CDD_PHASE_B) &&
                 (generation->rank2Phase == MOTOR_CONTROL_CDD_PHASE_C) &&
                 (generation->reconstructedPhase == MOTOR_CONTROL_CDD_PHASE_A) &&
                 ((generation->sector == 1U) || (generation->sector == 6U)))
        {
            phaseSelectionValid = TRUE;
        }
        else if ((generation->rank1Phase == MOTOR_CONTROL_CDD_PHASE_A) &&
                 (generation->rank2Phase == MOTOR_CONTROL_CDD_PHASE_C) &&
                 (generation->reconstructedPhase == MOTOR_CONTROL_CDD_PHASE_B) &&
                 ((generation->sector == 2U) || (generation->sector == 3U)))
        {
            phaseSelectionValid = TRUE;
        }
        else
        {
            /* The A/B fallback for sectors 4/5 is covered by common A/B. */
        }
    }

    if ((generation != NULL_PTR) &&
        (generation->generationId ==
         (__atomic_load_n(&MotorControlCdd_ConsumedGenerationId,
                          __ATOMIC_ACQUIRE) + 1U)) &&
        (generation->ccr1 <= MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS) &&
        (generation->ccr2 <= MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS) &&
        (generation->ccr3 <= MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS) &&
        (generation->ccr4 > 0U) &&
        (generation->ccr4 < MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS) &&
        (generation->sector >= 1U) &&
        (generation->sector <= 6U) &&
        ((generation->triggerEdge == MOTOR_CONTROL_CDD_TRIGGER_RISING) ||
         (generation->triggerEdge == MOTOR_CONTROL_CDD_TRIGGER_FALLING)) &&
        (generation->appliedVoltageAlphaQ15 >= MOTOR_CONTROL_CDD_Q15_MIN) &&
        (generation->appliedVoltageAlphaQ15 <= MOTOR_CONTROL_CDD_Q15_MAX) &&
        (generation->appliedVoltageBetaQ15 >= MOTOR_CONTROL_CDD_Q15_MIN) &&
        (generation->appliedVoltageBetaQ15 <= MOTOR_CONTROL_CDD_Q15_MAX) &&
        (phaseSelectionValid == TRUE) &&
        (generation->numericValid == TRUE) &&
        (generation->windowValid == TRUE) &&
        (generation->safetyValid == TRUE))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC void MotorControlCdd_BuildRuntimeGeneration(
    const MotorControlCdd_GenerationType *generation,
    MotorControlCdd_RuntimeGenerationType *runtime)
{
    static const uint32 channelByPhase[3] =
    {
        MOTOR_CONTROL_CDD_ADC_CHANNEL_A,
        MOTOR_CONTROL_CDD_ADC_CHANNEL_B,
        MOTOR_CONTROL_CDD_ADC_CHANNEL_C
    };

    runtime->generation = *generation;
    runtime->jsqr =
        (1UL << ADC_JSQR_JL_Pos) |
        ((uint32)generation->triggerEdge << ADC_JSQR_JEXTEN_Pos) |
        (channelByPhase[generation->rank1Phase] << ADC_JSQR_JSQ1_Pos) |
        (channelByPhase[generation->rank2Phase] << ADC_JSQR_JSQ2_Pos);
}

STATIC boolean MotorControlCdd_IsCommitPositionSafe(void)
{
    uint32 counterState;
    uint32 controlState;
    uint32 cutoffCounts = MOTOR_CONTROL_CDD_COMMIT_CUTOFF_COUNTS;

    if (MotorControlCdd_State != MOTOR_CONTROL_CDD_SAMPLING)
    {
        return TRUE;
    }

    counterState = TIM1->CNT;
    controlState = TIM1->CR1;
    if (MotorControlCdd_ControlApprovalsComplete() == TRUE)
    {
        cutoffCounts =
            MotorControlCdd_ActiveControlConfig.commitCutoffCounts;
    }
    if (((controlState & TIM_CR1_CEN) == 0U) ||
        ((TIM1->SR & TIM_SR_UIF) != 0U) ||
        (((controlState & TIM_CR1_DIR) != 0U) &&
         (counterState <= cutoffCounts)))
    {
        return FALSE;
    }

    return TRUE;
}

STATIC void MotorControlCdd_PublishCurrentSample(
    sint16 phaseA,
    sint16 phaseB,
    sint16 phaseC,
    uint32 generationId,
    boolean valid)
{
    uint32 attempt;

    for (attempt = 0U; attempt < MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
         attempt++)
    {
        uint32 slotIndex =
            (MotorControlCdd_SampleMailboxWriteCursor + attempt) %
            MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
        MotorControlCdd_CurrentSampleSlotType *slot =
            &MotorControlCdd_CurrentSampleMailbox[slotIndex];
        uint32 expected = MOTOR_CONTROL_CDD_MAILBOX_FREE;

        if (__atomic_compare_exchange_n(
                &slot->state, &expected, MOTOR_CONTROL_CDD_MAILBOX_WRITING,
                FALSE, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
        {
            MOTOR_CONTROL_CDD_TEST_MAILBOX_HOOK(
                MOTOR_CONTROL_CDD_TEST_MAILBOX_WRITING, slotIndex);
            slot->payload.phaseA = phaseA;
            slot->payload.phaseB = phaseB;
            slot->payload.phaseC = phaseC;
            slot->payload.generationId = generationId;
            slot->payload.valid = valid;
            __atomic_store_n(&slot->state, MOTOR_CONTROL_CDD_MAILBOX_READY,
                             __ATOMIC_RELEASE);
            MOTOR_CONTROL_CDD_TEST_MAILBOX_HOOK(
                MOTOR_CONTROL_CDD_TEST_MAILBOX_READY, slotIndex);
            MotorControlCdd_SampleMailboxWriteCursor =
                (slotIndex + 1U) % MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
            return;
        }
    }

    (void)__atomic_fetch_add(&MotorControlCdd_SampleDropCount, 1U,
                             __ATOMIC_RELAXED);
}

STATIC void MotorControlCdd_EnterPipelineFault(uint32 *eventWord, uint16 cause)
{
    MotorControlCdd_ControlLifecycleType controlLifecycle =
        MotorControlCdd_LoadControlLifecycle();

    MotorControlCdd_EmergencyOff();
    MotorControlCdd_GateAdcTrigger();
    TIM1->DIER &= ~(TIM_DIER_BIE | TIM_DIER_UIE);
    ADC1->IER &= ~ADC_IER_JEOSIE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    MotorControlCdd_PreparedGenerationValid = FALSE;
    MotorControlCdd_QueuedGenerationValid = FALSE;
    MotorControlCdd_State = MOTOR_CONTROL_CDD_FAILED;
    if ((controlLifecycle == MOTOR_CONTROL_CDD_CONTROL_PRIMING) ||
        (controlLifecycle == MOTOR_CONTROL_CDD_CONTROL_ACTIVE) ||
        (controlLifecycle == MOTOR_CONTROL_CDD_CONTROL_STOP_REQUIRED))
    {
        MotorControlCdd_ControlFailureType expected =
            MOTOR_CONTROL_CDD_CONTROL_FAILURE_NONE;

        (void)__atomic_compare_exchange_n(
            &MotorControlCdd_ControlFailure, &expected,
            MOTOR_CONTROL_CDD_CONTROL_FAILURE_GENERATION, FALSE,
            __ATOMIC_RELEASE, __ATOMIC_RELAXED);
        MotorControlCdd_StoreControlLifecycle(
            MOTOR_CONTROL_CDD_CONTROL_FAILED,
            MotorControlCdd_LoadControlEpoch());
    }
    MotorControlCdd_PublishEvent(eventWord, cause);
}

Std_ReturnType MotorControlCdd_EarlySafeInit(void)
{
    Std_ReturnType result = E_NOT_OK;

    /* Preload every safety output low before enabling its output driver. */
    MotorControlCdd_PreloadSafeOutputsLow();

    /* Keep the timer output path disabled even if reset state was disturbed. */
    MotorControlCdd_ClearMainOutputEnable();

    MotorControlCdd_ConfigureEarlySafePins();

    if (MotorControlCdd_IsEarlySafeStateValid() == TRUE)
    {
        MotorControlCdd_EarlySafeInitialized = TRUE;
        result = E_OK;
    }

    return result;
}

Std_ReturnType MotorControlCdd_CheckEarlySafeState(void)
{
    if (MotorControlCdd_IsEarlySafeStateValid() == TRUE)
    {
        return E_OK;
    }

    return E_NOT_OK;
}

Std_ReturnType MotorControlCdd_Init(void)
{
    uint32 mailboxIndex;

    if (MotorControlCdd_State != MOTOR_CONTROL_CDD_UNINITIALIZED)
    {
        return E_NOT_OK;
    }
    MotorControlCdd_State = MOTOR_CONTROL_CDD_INITIALIZING;

    if ((MotorControlCdd_EarlySafeInitialized != TRUE) ||
        (MotorControlCdd_AreClockPreconditionsValid() != TRUE) ||
        (Adc_CheckCommonConfiguration() != E_OK))
    {
        MotorControlCdd_State = MOTOR_CONTROL_CDD_FAILED;
        MotorControlCdd_EmergencyOff();
        return E_NOT_OK;
    }

    MotorControlCdd_PreloadSafeOutputsLow();
    MotorControlCdd_ClearMainOutputEnable();
    MotorControlCdd_GateAdcTrigger();
    MotorControlCdd_ConfigureTimerSafeBaseline();
    MotorControlCdd_ConfigureOwnedPins();

    if (MotorControlCdd_ConfigureAdc1() != E_OK)
    {
        MotorControlCdd_State = MOTOR_CONTROL_CDD_FAILED;
        MotorControlCdd_EmergencyOff();
        return E_NOT_OK;
    }

    if (MotorControlCdd_CfgGetControl(
            &MotorControlCdd_ActiveControlConfig) != E_OK)
    {
        MotorControlCdd_State = MOTOR_CONTROL_CDD_FAILED;
        MotorControlCdd_EmergencyOff();
        return E_NOT_OK;
    }

    MotorControlCdd_CurrentOffsetsValid = FALSE;
    MotorControlCdd_CalibrationEvent = 0U;
    MotorControlCdd_ControlEvent = 0U;
    __atomic_store_n(&MotorControlCdd_ControlFailure,
                     MOTOR_CONTROL_CDD_CONTROL_FAILURE_NONE,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&MotorControlCdd_AcceptedRequestQ15, 0,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&MotorControlCdd_CommissioningCapQ15, 0,
                     __ATOMIC_RELAXED);
    MotorControlCdd_ControlStaging.payload =
        (MotorControlCdd_ControlActivationType){ 0 };
    MotorControlCdd_LiveControl =
        (MotorControlCdd_ControlActivationType){ 0 };
    __atomic_store_n(&MotorControlCdd_ControlStaging.state,
                     MOTOR_CONTROL_CDD_MAILBOX_FREE, __ATOMIC_RELAXED);
    MotorControlCdd_RetainedFastSafety =
        (MotorControlCdd_FastSafetySnapshotType){ 0 };
    MotorControlCdd_RetainedFastSafetyValid = FALSE;
    MotorControlCdd_CalibrationState = MOTOR_CONTROL_CDD_CALIBRATION_IDLE;
    MotorControlCdd_CalibrationFailure =
        MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE;
    MotorControlCdd_ActiveCalibrationConfig =
        (MotorControlCdd_CalibrationConfigType){ 0 };
    MotorControlCdd_CalibrationBaseline =
        (MotorControlCdd_CalibrationBaselineType){ 0 };
    MotorControlCdd_ResetCalibrationData();
    MotorControlCdd_PreparedGenerationValid = FALSE;
    MotorControlCdd_QueuedGenerationValid = FALSE;
    MotorControlCdd_DiscardNextSample = TRUE;
    __atomic_store_n(&MotorControlCdd_ConsumedGenerationId, 0U,
                     __ATOMIC_RELAXED);
    for (mailboxIndex = 0U;
         mailboxIndex < MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
         mailboxIndex++)
    {
        MotorControlCdd_CurrentSampleMailbox[mailboxIndex].payload.phaseA = 0;
        MotorControlCdd_CurrentSampleMailbox[mailboxIndex].payload.phaseB = 0;
        MotorControlCdd_CurrentSampleMailbox[mailboxIndex].payload.phaseC = 0;
        MotorControlCdd_CurrentSampleMailbox[mailboxIndex].payload.generationId =
            0U;
        MotorControlCdd_CurrentSampleMailbox[mailboxIndex].payload.valid = FALSE;
        __atomic_store_n(
            &MotorControlCdd_CurrentSampleMailbox[mailboxIndex].state,
            MOTOR_CONTROL_CDD_MAILBOX_FREE, __ATOMIC_RELAXED);
        MotorControlCdd_FastSafetyMailbox[mailboxIndex].payload =
            (MotorControlCdd_FastSafetySnapshotType){ 0 };
        __atomic_store_n(
            &MotorControlCdd_FastSafetyMailbox[mailboxIndex].state,
            MOTOR_CONTROL_CDD_MAILBOX_FREE, __ATOMIC_RELAXED);
    }
    MotorControlCdd_SampleMailboxWriteCursor = 0U;
    MotorControlCdd_FastSafetyWriteCursor = 0U;
    __atomic_store_n(&MotorControlCdd_SampleDropCount, 0U, __ATOMIC_RELAXED);
    MotorControlCdd_StoreControlLifecycle(
        (MotorControlCdd_ControlApprovalsComplete() == TRUE) ?
            MOTOR_CONTROL_CDD_CONTROL_IDLE :
            MOTOR_CONTROL_CDD_CONTROL_UNAPPROVED,
        0U);
    MotorControlCdd_State = MOTOR_CONTROL_CDD_STATIC_SAFE;
    TIM1->DIER = TIM_DIER_BIE;
    if (MotorControlCdd_IsCompleteStaticConfigurationValid() != TRUE)
    {
        TIM1->DIER = 0U;
        MotorControlCdd_State = MOTOR_CONTROL_CDD_FAILED;
        MotorControlCdd_EmergencyOff();
        return E_NOT_OK;
    }

    NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0U);
    NVIC_SetPriority(ADC1_2_IRQn, 1U);
    NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 2U);
    NVIC_ClearPendingIRQ(TIM1_BRK_TIM15_IRQn);
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

    return E_OK;
}

Std_ReturnType MotorControlCdd_CheckStaticConfiguration(void)
{
    if ((MotorControlCdd_State == MOTOR_CONTROL_CDD_STATIC_SAFE) &&
        (MotorControlCdd_IsCompleteStaticConfigurationValid() == TRUE))
    {
        return E_OK;
    }

    return E_NOT_OK;
}

Std_ReturnType MotorControlCdd_GetFaultEvents(
    MotorControlCdd_FaultEventsType *Events)
{
    if (Events == NULL_PTR)
    {
        return E_NOT_OK;
    }

    Events->breakEvent =
        __atomic_load_n(&MotorControlCdd_BreakEvent, __ATOMIC_ACQUIRE);
    MOTOR_CONTROL_CDD_TEST_FAULT_READ_HOOK(
        MOTOR_CONTROL_CDD_TEST_FAULT_READ_BREAK);
    Events->updateEvent =
        __atomic_load_n(&MotorControlCdd_UpdateEvent, __ATOMIC_ACQUIRE);
    MOTOR_CONTROL_CDD_TEST_FAULT_READ_HOOK(
        MOTOR_CONTROL_CDD_TEST_FAULT_READ_UPDATE);
    Events->adcEvent =
        __atomic_load_n(&MotorControlCdd_AdcEvent, __ATOMIC_ACQUIRE);
    MOTOR_CONTROL_CDD_TEST_FAULT_READ_HOOK(
        MOTOR_CONTROL_CDD_TEST_FAULT_READ_ADC);
    Events->calibrationEvent =
        __atomic_load_n(&MotorControlCdd_CalibrationEvent, __ATOMIC_ACQUIRE);
    MOTOR_CONTROL_CDD_TEST_FAULT_READ_HOOK(
        MOTOR_CONTROL_CDD_TEST_FAULT_READ_CALIBRATION);
    Events->controlEvent =
        __atomic_load_n(&MotorControlCdd_ControlEvent, __ATOMIC_ACQUIRE);
    MOTOR_CONTROL_CDD_TEST_FAULT_READ_HOOK(
        MOTOR_CONTROL_CDD_TEST_FAULT_READ_CONTROL);
    return E_OK;
}

Std_ReturnType MotorControlCdd_StartCalibration(
    const MotorControlCdd_CalibrationEnvironmentType *Environment)
{
    if ((Environment == NULL_PTR) ||
        (MotorControlCdd_State != MOTOR_CONTROL_CDD_STATIC_SAFE) ||
        (MotorControlCdd_CalibrationState !=
         MOTOR_CONTROL_CDD_CALIBRATION_IDLE) ||
        (MotorControlCdd_PreparedGenerationValid == TRUE) ||
        (MotorControlCdd_QueuedGenerationValid == TRUE) ||
        (MotorControlCdd_AreFaultEventsClear() != TRUE) ||
        (MotorControlCdd_IsCompleteStaticConfigurationValid() != TRUE) ||
        ((ADC1->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP)) !=
         0U) ||
        (ADC1->JSQR != 0U))
    {
        return E_NOT_OK;
    }

    if (MotorControlCdd_CfgGetCalibration(
            &MotorControlCdd_ActiveCalibrationConfig) != E_OK)
    {
        MotorControlCdd_FailCalibration(
            MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_CONFIG, FALSE);
        return E_NOT_OK;
    }

    MotorControlCdd_ResetCalibrationData();
    MotorControlCdd_CalibrationFailure =
        MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE;
    if (MotorControlCdd_UpdateCalibrationEnvironment(Environment) != TRUE)
    {
        MotorControlCdd_FailCalibration(
            MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT, FALSE);
        return E_NOT_OK;
    }

    MotorControlCdd_CalibrationBaseline.cfgr = ADC1->CFGR;
    MotorControlCdd_CalibrationBaseline.smpr1 = ADC1->SMPR1;
    MotorControlCdd_CalibrationBaseline.smpr2 = ADC1->SMPR2;
    MotorControlCdd_CalibrationBaseline.sqr1 = ADC1->SQR1;
    MotorControlCdd_CalibrationBaseline.sqr2 = ADC1->SQR2;
    MotorControlCdd_CalibrationBaseline.sqr3 = ADC1->SQR3;
    MotorControlCdd_CalibrationBaseline.sqr4 = ADC1->SQR4;
    MotorControlCdd_CalibrationBaseline.valid = TRUE;

    ADC1->CFGR =
        (ADC1->CFGR & ~MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_MASK) |
        MOTOR_CONTROL_CDD_ADC_REGULAR_CFGR_VALUE;
    ADC1->SMPR1 =
        (ADC1->SMPR1 & ~MOTOR_CONTROL_CDD_ADC_SMPR1_MASK) |
        MOTOR_CONTROL_CDD_ADC_SMPR1_VALUE;
    ADC1->SMPR2 =
        (ADC1->SMPR2 & ~MOTOR_CONTROL_CDD_ADC_SMPR2_MASK) |
        MOTOR_CONTROL_CDD_ADC_SMPR2_VALUE;
    ADC1->SQR1 = MOTOR_CONTROL_CDD_ADC_REGULAR_SQR1_VALUE;
    ADC1->SQR2 = MOTOR_CONTROL_CDD_ADC_REGULAR_SQR2_VALUE;
    ADC1->SQR3 = 0U;
    ADC1->SQR4 = 0U;
    MotorControlCdd_ClearAdcStatus(
        MOTOR_CONTROL_CDD_ADC_REGULAR_W1C_MASK);

    if ((MotorControlCdd_IsCalibrationConfigurationValid() != TRUE) ||
        ((ADC1->ISR & MOTOR_CONTROL_CDD_ADC_REGULAR_STATUS_MASK) != 0U) ||
        ((TIM1->CR1 & TIM_CR1_CEN) != 0U) ||
        ((TIM1->CR2 & TIM_CR2_MMS) != 0U) ||
        (MotorControlCdd_IsEarlySafeStateValid() != TRUE))
    {
        MotorControlCdd_FailCalibration(
            MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_PRECONDITION, TRUE);
        return E_NOT_OK;
    }

    MotorControlCdd_CalibrationState =
        MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64;
    ADC1->CR |= ADC_CR_ADSTART;
    MOTOR_CONTROL_CDD_TEST_CALIBRATION_CONTROL_HOOK();
    return E_OK;
}

Std_ReturnType MotorControlCdd_CalibrationStep(
    const MotorControlCdd_CalibrationEnvironmentType *Environment)
{
    if ((MotorControlCdd_CalibrationState !=
         MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64) &&
        (MotorControlCdd_CalibrationState !=
         MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST) &&
        (MotorControlCdd_CalibrationState !=
         MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY))
    {
        return E_NOT_OK;
    }

    if (MotorControlCdd_UpdateCalibrationEnvironment(Environment) != TRUE)
    {
        MotorControlCdd_FailCalibration(
            MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT, TRUE);
        return E_NOT_OK;
    }

    if (((MotorControlCdd_CalibrationState ==
          MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64) ||
         (MotorControlCdd_CalibrationState ==
          MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST)) &&
        (MotorControlCdd_IsCalibrationConfigurationValid() != TRUE))
    {
        MotorControlCdd_FailCalibration(
            MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_REGISTER, TRUE);
        return E_NOT_OK;
    }

    if (MotorControlCdd_CalibrationState ==
        MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64)
    {
        uint32 controlSnapshot = ADC1->CR;
        uint32 statusSnapshot = ADC1->ISR;
        uint32 phase = MotorControlCdd_CalibrationAcceptedSamples %
                       MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
        uint32 expectedStatus = ADC_ISR_EOC | ADC_ISR_OVR;
        uint32 rawSample;

        if (phase == 2U)
        {
            expectedStatus |= ADC_ISR_EOS;
        }
        if (((controlSnapshot & ADC_CR_ADSTART) != 0U) ||
            ((statusSnapshot & MOTOR_CONTROL_CDD_ADC_REGULAR_STATUS_MASK) !=
             expectedStatus))
        {
            MotorControlCdd_FailCalibration(
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS, TRUE);
            return E_NOT_OK;
        }

        rawSample = ADC1->DR;
        MOTOR_CONTROL_CDD_TEST_CALIBRATION_DR_READ_HOOK();
        if (rawSample > MOTOR_CONTROL_CDD_ADC_RAW_MASK)
        {
            MotorControlCdd_FailCalibration(
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_SAMPLE, TRUE);
            return E_NOT_OK;
        }
        MotorControlCdd_ClearAdcStatus(
            ADC_ISR_OVR | ((phase == 2U) ? ADC_ISR_EOS : 0U));
        if ((ADC1->ISR & MOTOR_CONTROL_CDD_ADC_REGULAR_STATUS_MASK) != 0U)
        {
            MotorControlCdd_FailCalibration(
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS, TRUE);
            return E_NOT_OK;
        }
        if (MotorControlCdd_RecordCalibrationSample(
                phase, (uint16)rawSample) != TRUE)
        {
            MotorControlCdd_FailCalibration(
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_SAMPLE, TRUE);
            return E_NOT_OK;
        }

        if (MotorControlCdd_CalibrationAcceptedSamples <
            MOTOR_CONTROL_CDD_CALIBRATION_TOTAL_SAMPLES)
        {
            ADC1->CR |= ADC_CR_ADSTART;
            MOTOR_CONTROL_CDD_TEST_CALIBRATION_CONTROL_HOOK();
        }
        else
        {
            ADC1->CR |= ADC_CR_ADSTP;
            MOTOR_CONTROL_CDD_TEST_CALIBRATION_CONTROL_HOOK();
            MotorControlCdd_CalibrationState =
                MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST;
        }
        return E_OK;
    }

    if (MotorControlCdd_CalibrationState ==
        MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST)
    {
        if ((ADC1->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP)) != 0U)
        {
            MotorControlCdd_FailCalibration(
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_RESTORE, TRUE);
            return E_NOT_OK;
        }
        MotorControlCdd_ClearAdcStatus(
            MOTOR_CONTROL_CDD_ADC_REGULAR_W1C_MASK);
        if (((ADC1->ISR & MOTOR_CONTROL_CDD_ADC_REGULAR_STATUS_MASK) != 0U) ||
            (MotorControlCdd_RestoreCalibrationBaseline() != TRUE))
        {
            MotorControlCdd_FailCalibration(
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_RESTORE, TRUE);
            return E_NOT_OK;
        }
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY;
        return E_OK;
    }

    if ((MotorControlCdd_IsCalibrationBaselineRestored() != TRUE) ||
        (MotorControlCdd_ValidateCalibrationStatistics() != TRUE))
    {
        MotorControlCdd_FailCalibration(
            MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_RESTORE, TRUE);
        return E_NOT_OK;
    }
    MotorControlCdd_CalibrationState = MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE;
    return E_OK;
}

Std_ReturnType MotorControlCdd_RequestCalibrationAbort(
    MotorControlCdd_CalibrationAbortModeType Mode)
{
    if ((Mode != MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP) &&
        (Mode != MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC))
    {
        return E_NOT_OK;
    }

    if (Mode ==
        MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC)
    {
        MotorControlCdd_CurrentOffsetsValid = FALSE;
        MotorControlCdd_CalibrationResult.offsetsInstalled = FALSE;
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED;
        return E_OK;
    }

    if ((MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_REQUEST) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_VERIFY) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_RESTORE) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_VERIFY))
    {
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_REQUEST;
        return E_OK;
    }

    if ((MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_IDLE) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE))
    {
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_FAILED_SAFE;
        return E_OK;
    }
    return E_NOT_OK;
}

Std_ReturnType MotorControlCdd_CalibrationCleanupStep(void)
{
    if (MotorControlCdd_CalibrationState ==
        MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_REQUEST)
    {
        ADC1->CR |= ADC_CR_ADSTP;
        MOTOR_CONTROL_CDD_TEST_CALIBRATION_CONTROL_HOOK();
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_VERIFY;
        return E_OK;
    }
    if (MotorControlCdd_CalibrationState ==
        MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_VERIFY)
    {
        if ((ADC1->CR & (ADC_CR_ADSTART | ADC_CR_ADSTP)) != 0U)
        {
            if (MotorControlCdd_CalibrationFailure ==
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE)
            {
                MotorControlCdd_CalibrationFailure =
                    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_CLEANUP;
            }
            MotorControlCdd_CalibrationState =
                MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED;
            return E_NOT_OK;
        }
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_RESTORE;
        return E_OK;
    }
    if (MotorControlCdd_CalibrationState ==
        MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_RESTORE)
    {
        MotorControlCdd_ClearAdcStatus(
            MOTOR_CONTROL_CDD_ADC_REGULAR_W1C_MASK);
        if (MotorControlCdd_RestoreCalibrationBaseline() != TRUE)
        {
            if (MotorControlCdd_CalibrationFailure ==
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE)
            {
                MotorControlCdd_CalibrationFailure =
                    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_CLEANUP;
            }
            MotorControlCdd_CalibrationState =
                MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED;
            return E_NOT_OK;
        }
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_VERIFY;
        return E_OK;
    }
    if (MotorControlCdd_CalibrationState ==
        MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_VERIFY)
    {
        if (MotorControlCdd_IsCalibrationBaselineRestored() != TRUE)
        {
            if (MotorControlCdd_CalibrationFailure ==
                MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE)
            {
                MotorControlCdd_CalibrationFailure =
                    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_CLEANUP;
            }
            MotorControlCdd_CalibrationState =
                MOTOR_CONTROL_CDD_CALIBRATION_RESET_REQUIRED;
            return E_NOT_OK;
        }
        MotorControlCdd_CalibrationState =
            MOTOR_CONTROL_CDD_CALIBRATION_FAILED_SAFE;
        return E_OK;
    }
    return E_NOT_OK;
}

Std_ReturnType MotorControlCdd_GetCalibrationResult(
    MotorControlCdd_CalibrationResultType *Result)
{
    if ((Result == NULL_PTR) ||
        (MotorControlCdd_CalibrationState !=
         MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE) ||
        (MotorControlCdd_CalibrationResult.valid != TRUE))
    {
        return E_NOT_OK;
    }
    *Result = MotorControlCdd_CalibrationResult;
    return E_OK;
}

Std_ReturnType MotorControlCdd_GetCalibrationReadiness(
    MotorControlCdd_CalibrationReadinessType *Readiness)
{
    MotorControlCdd_CalibrationConfigType config;

    if (Readiness == NULL_PTR)
    {
        return E_NOT_OK;
    }
    *Readiness = (MotorControlCdd_CalibrationReadinessType){ 0 };
    if (MotorControlCdd_CfgGetCalibration(&config) != E_OK)
    {
        return E_OK;
    }

    Readiness->identity = config.identity;
    Readiness->structuralValid = TRUE;
    Readiness->measurementApproved = config.measurementApproved;
    Readiness->calibrationComplete =
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE) ? TRUE : FALSE;
    Readiness->offsetsInstalled = MotorControlCdd_CurrentOffsetsValid;
    return E_OK;
}

boolean MotorControlCdd_IsCalibrationActive(void)
{
    if ((MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_ACQUIRE_64) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_REQUEST) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_RESTORE_VERIFY))
    {
        return TRUE;
    }
    return FALSE;
}

boolean MotorControlCdd_IsCalibrationCleanupPending(void)
{
    if ((MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_REQUEST) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_ABORT_VERIFY) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_RESTORE) ||
        (MotorControlCdd_CalibrationState ==
         MOTOR_CONTROL_CDD_CALIBRATION_CLEANUP_VERIFY))
    {
        return TRUE;
    }
    return FALSE;
}

MotorControlCdd_CalibrationFailureType
MotorControlCdd_GetCalibrationFailureCause(void)
{
    return MotorControlCdd_CalibrationFailure;
}

STATIC Std_ReturnType MotorControlCdd_InstallCurrentOffsets(
    const MotorControlCdd_CurrentOffsetsType *Offsets)
{
    if ((Offsets == NULL_PTR) ||
        (MotorControlCdd_State != MOTOR_CONTROL_CDD_STATIC_SAFE) ||
        (Offsets->phaseA > MOTOR_CONTROL_CDD_ADC_RAW_MASK) ||
        (Offsets->phaseB > MOTOR_CONTROL_CDD_ADC_RAW_MASK) ||
        (Offsets->phaseC > MOTOR_CONTROL_CDD_ADC_RAW_MASK) ||
        (MotorControlCdd_AreFaultEventsClear() != TRUE))
    {
        return E_NOT_OK;
    }

    MotorControlCdd_CurrentOffsets = *Offsets;
    MotorControlCdd_CurrentOffsetsValid = TRUE;
    return E_OK;
}

STATIC void MotorControlCdd_BuildBootstrapGeneration(
    MotorControlCdd_GenerationType *generation)
{
    generation->generationId = 1U;
    generation->ccr1 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    generation->ccr2 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    generation->ccr3 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    generation->ccr4 = MOTOR_CONTROL_CDD_INITIAL_SAMPLE_COUNT;
    generation->sector = 1U;
    generation->rank1Phase = MOTOR_CONTROL_CDD_PHASE_A;
    generation->rank2Phase = MOTOR_CONTROL_CDD_PHASE_B;
    generation->reconstructedPhase = MOTOR_CONTROL_CDD_PHASE_C;
    generation->triggerEdge = MOTOR_CONTROL_CDD_TRIGGER_RISING;
    generation->appliedVoltageAlphaQ15 = 0;
    generation->appliedVoltageBetaQ15 = 0;
    generation->numericValid = TRUE;
    generation->windowValid = TRUE;
    generation->safetyValid = TRUE;
}

STATIC Std_ReturnType MotorControlCdd_NormalizeRawCurrent(
    sint32 rawDelta,
    sint32 *normalizedQ15)
{
    uint64 product;
    uint64 magnitude;
    uint64 rounding;
    uint32 shift = MotorControlCdd_ActiveControlConfig.rawToQ15Shift;

    if ((normalizedQ15 == NULL_PTR) ||
        (MotorControlCdd_AbsS32(rawDelta) > MOTOR_CONTROL_CDD_ADC_RAW_MASK) ||
        (MotorControlCdd_ActiveControlConfig.rawToQ15Multiplier == 0U) ||
        (shift == 0U) || (shift > 31U))
    {
        return E_NOT_OK;
    }
    product = (uint64)MotorControlCdd_AbsS32(rawDelta) *
              (uint64)MotorControlCdd_ActiveControlConfig.rawToQ15Multiplier;
    rounding = (uint64)1U << (shift - 1U);
    magnitude = (product + rounding) >> shift;
    if ((magnitude > 32768U) ||
        ((rawDelta >= 0) && (magnitude > 32767U)))
    {
        return E_NOT_OK;
    }
    *normalizedQ15 = (rawDelta < 0) ?
        -(sint32)magnitude : (sint32)magnitude;
    return E_OK;
}

STATIC uint16 MotorControlCdd_DutyQ15ToCompare(uint16 dutyQ15)
{
    uint64 numerator =
        ((uint64)dutyQ15 * MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS) +
        (MOTOR_CONTROL_CDD_SVPWM_Q15_MAX / 2U);

    return (uint16)(numerator / MOTOR_CONTROL_CDD_SVPWM_Q15_MAX);
}

STATIC sint32 MotorControlCdd_RoundAwayDivideBy3(sint64 value)
{
    return (value >= 0) ?
        (sint32)((value + 1LL) / 3LL) :
        -(sint32)(((-value) + 1LL) / 3LL);
}

STATIC sint32 MotorControlCdd_RoundAwayDivideBy32768(sint64 value)
{
    uint64 magnitude = (value >= 0) ?
        (uint64)value : (uint64)(-value);
    sint32 rounded = (sint32)((magnitude + 16384ULL) >> 15U);

    return (value < 0) ? -rounded : rounded;
}

STATIC uint16 MotorControlCdd_CompareToDutyQ15(uint16 compare)
{
    uint64 numerator =
        ((uint64)compare * MOTOR_CONTROL_CDD_SVPWM_Q15_MAX) +
        (MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS / 2U);

    return (uint16)(numerator / MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS);
}

STATIC boolean MotorControlCdd_TrySamplingPair(
    const uint16 *compares,
    MotorControlCdd_PhaseType candidateRank1,
    MotorControlCdd_PhaseType candidateRank2,
    MotorControlCdd_PhaseType candidateReconstructed,
    MotorControlCdd_PhaseType *rank1,
    MotorControlCdd_PhaseType *rank2,
    MotorControlCdd_PhaseType *reconstructed,
    uint16 *ccr4,
    MotorControlCdd_TriggerEdgeType *edge)
{
    sint64 fullPeriod =
        (sint64)2 * MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS;
    sint64 blanking =
        MotorControlCdd_ActiveControlConfig.switchingBlankingCounts;
    sint64 latency =
        MotorControlCdd_ActiveControlConfig.triggerLatencyCounts;
    sint64 aperture =
        MotorControlCdd_ActiveControlConfig.apertureCounts;
    sint64 separation =
        MotorControlCdd_ActiveControlConfig.rankSeparationCounts;
    sint64 low1Lo = (sint64)compares[candidateRank1] + blanking;
    sint64 low1Hi = fullPeriod -
                    (sint64)compares[candidateRank1] - blanking;
    sint64 low2Lo = (sint64)compares[candidateRank2] + blanking;
    sint64 low2Hi = fullPeriod -
                    (sint64)compares[candidateRank2] - blanking;
    sint64 pairLo = (low1Lo > low2Lo) ? low1Lo : low2Lo;
    sint64 pairHi = (low1Hi < low2Hi) ? low1Hi : low2Hi;
    sint64 feasibleLo = pairLo - latency;
    sint64 feasibleHi = pairHi -
                        (latency + separation + aperture - 1LL);
    sint64 armMinimum =
        MotorControlCdd_ActiveControlConfig.triggerArmMinimum;
    sint64 commitMaximum = fullPeriod -
        MotorControlCdd_ActiveControlConfig.commitMarginCounts;
    sint64 trigger;
    sint64 reconstructedTrigger;
    sint64 firstApertureLo;
    sint64 secondApertureHi;

    if ((low1Lo > low1Hi) || (low2Lo > low2Hi))
    {
        return FALSE;
    }
    if (feasibleLo < armMinimum)
    {
        feasibleLo = armMinimum;
    }
    if (feasibleHi > commitMaximum)
    {
        feasibleHi = commitMaximum;
    }
    if ((feasibleLo > feasibleHi) || (feasibleHi < 1LL) ||
        (feasibleLo > (fullPeriod - 1LL)))
    {
        return FALSE;
    }
    if (feasibleLo < 1LL)
    {
        feasibleLo = 1LL;
    }
    if (feasibleHi > (fullPeriod - 1LL))
    {
        feasibleHi = fullPeriod - 1LL;
    }
    trigger = (feasibleLo + feasibleHi) / 2LL;
    if (trigger == MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS)
    {
        if ((trigger - 1LL) >= feasibleLo)
        {
            trigger--;
        }
        else if ((trigger + 1LL) <= feasibleHi)
        {
            trigger++;
        }
        else
        {
            return FALSE;
        }
    }
    if ((trigger <= 0LL) || (trigger >= fullPeriod))
    {
        return FALSE;
    }
    if (trigger < MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS)
    {
        *ccr4 = (uint16)trigger;
        *edge = MOTOR_CONTROL_CDD_TRIGGER_RISING;
        reconstructedTrigger = *ccr4;
    }
    else
    {
        *ccr4 = (uint16)(fullPeriod - trigger);
        *edge = MOTOR_CONTROL_CDD_TRIGGER_FALLING;
        reconstructedTrigger = fullPeriod - *ccr4;
    }
    if ((*ccr4 == 0U) ||
        (*ccr4 >= MOTOR_CONTROL_CDD_PWM_PERIOD_COUNTS))
    {
        return FALSE;
    }
    firstApertureLo = reconstructedTrigger + latency;
    secondApertureHi = reconstructedTrigger + latency + separation +
                       aperture - 1LL;
    if ((firstApertureLo < low1Lo) || (firstApertureLo < low2Lo) ||
        (secondApertureHi > low1Hi) ||
        (secondApertureHi > low2Hi) ||
        (reconstructedTrigger < armMinimum) ||
        (reconstructedTrigger > commitMaximum))
    {
        return FALSE;
    }
    *rank1 = candidateRank1;
    *rank2 = candidateRank2;
    *reconstructed = candidateReconstructed;
    return TRUE;
}

STATIC boolean MotorControlCdd_SelectSamplingWindow(
    uint16 ccr1,
    uint16 ccr2,
    uint16 ccr3,
    uint8 sector,
    MotorControlCdd_PhaseType *rank1,
    MotorControlCdd_PhaseType *rank2,
    MotorControlCdd_PhaseType *reconstructed,
    uint16 *ccr4,
    MotorControlCdd_TriggerEdgeType *edge)
{
    uint16 compares[3] = { ccr1, ccr2, ccr3 };

    if (MotorControlCdd_TrySamplingPair(
            compares,
            MOTOR_CONTROL_CDD_PHASE_A,
            MOTOR_CONTROL_CDD_PHASE_B,
            MOTOR_CONTROL_CDD_PHASE_C,
            rank1, rank2, reconstructed, ccr4, edge) == TRUE)
    {
        return TRUE;
    }
    if (((sector == 1U) || (sector == 6U)) &&
        (MotorControlCdd_TrySamplingPair(
             compares,
             MOTOR_CONTROL_CDD_PHASE_B,
             MOTOR_CONTROL_CDD_PHASE_C,
             MOTOR_CONTROL_CDD_PHASE_A,
             rank1, rank2, reconstructed, ccr4, edge) == TRUE))
    {
        return TRUE;
    }
    if (((sector == 2U) || (sector == 3U)) &&
        (MotorControlCdd_TrySamplingPair(
             compares,
             MOTOR_CONTROL_CDD_PHASE_A,
             MOTOR_CONTROL_CDD_PHASE_C,
             MOTOR_CONTROL_CDD_PHASE_B,
             rank1, rank2, reconstructed, ccr4, edge) == TRUE))
    {
        return TRUE;
    }
    return FALSE;
}

STATIC Std_ReturnType MotorControlCdd_BuildControlGeneration(
    const MotorControlFastLoop_ResultType *result,
    MotorControlCdd_GenerationType *generation)
{
    sint32 dutyA;
    sint32 dutyB;
    sint32 dutyC;
    sint32 xA;
    sint32 xB;
    sint32 xC;
    sint32 alpha;
    sint32 beta;

    if ((result == NULL_PTR) || (generation == NULL_PTR) ||
        (result->controlValid != TRUE) ||
        (result->requestedSvpwm.limited == TRUE) ||
        (result->requestedSvpwm.dutyAQ15 > MOTOR_CONTROL_CDD_SVPWM_Q15_MAX) ||
        (result->requestedSvpwm.dutyBQ15 > MOTOR_CONTROL_CDD_SVPWM_Q15_MAX) ||
        (result->requestedSvpwm.dutyCQ15 > MOTOR_CONTROL_CDD_SVPWM_Q15_MAX) ||
        (result->requestedSvpwm.sector < 1U) ||
        (result->requestedSvpwm.sector > 6U))
    {
        return E_NOT_OK;
    }

    *generation = (MotorControlCdd_GenerationType){ 0 };
    generation->generationId = result->nextGenerationId;
    generation->ccr1 = MotorControlCdd_DutyQ15ToCompare(
        result->requestedSvpwm.dutyAQ15);
    generation->ccr2 = MotorControlCdd_DutyQ15ToCompare(
        result->requestedSvpwm.dutyBQ15);
    generation->ccr3 = MotorControlCdd_DutyQ15ToCompare(
        result->requestedSvpwm.dutyCQ15);
    generation->sector = result->requestedSvpwm.sector;
    if (MotorControlCdd_SelectSamplingWindow(
            generation->ccr1, generation->ccr2, generation->ccr3,
            generation->sector,
            &generation->rank1Phase, &generation->rank2Phase,
            &generation->reconstructedPhase,
            &generation->ccr4, &generation->triggerEdge) != TRUE)
    {
        return E_NOT_OK;
    }

    dutyA = MotorControlCdd_CompareToDutyQ15(generation->ccr1);
    dutyB = MotorControlCdd_CompareToDutyQ15(generation->ccr2);
    dutyC = MotorControlCdd_CompareToDutyQ15(generation->ccr3);
    xA = (2 * dutyA) - 32768;
    xB = (2 * dutyB) - 32768;
    xC = (2 * dutyC) - 32768;
    alpha = MotorControlCdd_RoundAwayDivideBy3(
        ((sint64)2 * xA) - xB - xC);
    beta = MotorControlCdd_RoundAwayDivideBy32768(
        ((sint64)xB - xC) * 18919LL);
    if ((alpha < MOTOR_CONTROL_CDD_Q15_MIN) ||
        (alpha > MOTOR_CONTROL_CDD_Q15_MAX) ||
        (beta < MOTOR_CONTROL_CDD_Q15_MIN) ||
        (beta > MOTOR_CONTROL_CDD_Q15_MAX) ||
        (MotorControlCdd_AbsS32(
             alpha - result->requestedVoltage.alphaQ15) >
         MotorControlCdd_ActiveControlConfig
             .appliedVoltageMaximumErrorQ15) ||
        (MotorControlCdd_AbsS32(
             beta - result->requestedVoltage.betaQ15) >
         MotorControlCdd_ActiveControlConfig
             .appliedVoltageMaximumErrorQ15))
    {
        return E_NOT_OK;
    }
    generation->appliedVoltageAlphaQ15 = alpha;
    generation->appliedVoltageBetaQ15 = beta;
    generation->numericValid = TRUE;
    generation->windowValid = TRUE;
    generation->safetyValid = TRUE;
    return E_OK;
}

STATIC void MotorControlCdd_BuildNextGeneration(
    const MotorControlCdd_RuntimeGenerationType *consumed,
    MotorControlCdd_GenerationType *generation)
{
    *generation = consumed->generation;
    generation->generationId++;

    /*
     * Step 5 retains the safe zero-vector command. The private JEOS/FOC path
     * in later steps replaces these values after calculating sampling-window
     * validity. The complete explicit phase identity is already part of the
     * immutable record and may select common A/B or an approved fallback.
     */
    MOTOR_CONTROL_CDD_TEST_NEXT_GENERATION_HOOK(generation);
}

STATIC Std_ReturnType MotorControlCdd_CommitPreparedGeneration(
    const MotorControlCdd_GenerationType *Generation)
{
    MotorControlCdd_RuntimeGenerationType runtimeGeneration;
    boolean inputValid;

    inputValid = MotorControlCdd_IsGenerationInputValid(Generation);
    if ((MotorControlCdd_State != MOTOR_CONTROL_CDD_SAMPLING) ||
        (MotorControlCdd_PreparedGenerationValid == TRUE) ||
        (MotorControlCdd_QueuedGenerationValid == TRUE) ||
        (MotorControlCdd_IsEarlySafeStateValid() != TRUE) ||
        (MotorControlCdd_AreFaultEventsClear() != TRUE) ||
        (inputValid != TRUE))
    {
        if (MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING)
        {
            MotorControlCdd_EnterPipelineFault(
                &MotorControlCdd_AdcEvent,
                MOTOR_CONTROL_CDD_EVENT_CAUSE_INVALID_GENERATION);
        }
        return E_NOT_OK;
    }

    MotorControlCdd_BuildRuntimeGeneration(Generation, &runtimeGeneration);
    if (MotorControlCdd_IsCommitPositionSafe() != TRUE)
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_AdcEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_INVALID_GENERATION);
        return E_NOT_OK;
    }

    TIM1->CCR1 = runtimeGeneration.generation.ccr1;
    TIM1->CCR2 = runtimeGeneration.generation.ccr2;
    TIM1->CCR3 = runtimeGeneration.generation.ccr3;
    TIM1->CCR4 = runtimeGeneration.generation.ccr4;
    __DMB();
    MOTOR_CONTROL_CDD_TEST_COMMIT_HOOK();

    if (MotorControlCdd_IsCommitPositionSafe() != TRUE)
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_AdcEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_INVALID_GENERATION);
        return E_NOT_OK;
    }

    MotorControlCdd_PreparedGeneration = runtimeGeneration;
    __DMB();
    MotorControlCdd_PreparedGenerationValid = TRUE;
    return E_OK;
}

STATIC Std_ReturnType MotorControlCdd_SeedBootstrapGeneration(
    const MotorControlCdd_GenerationType *generation)
{
    MotorControlCdd_RuntimeGenerationType runtimeGeneration;

    if ((MotorControlCdd_State != MOTOR_CONTROL_CDD_STATIC_SAFE) ||
        (MotorControlCdd_PreparedGenerationValid == TRUE) ||
        (MotorControlCdd_QueuedGenerationValid == TRUE) ||
        (MotorControlCdd_IsGenerationInputValid(generation) != TRUE) ||
        (TIM1->CCR1 != generation->ccr1) ||
        (TIM1->CCR2 != generation->ccr2) ||
        (TIM1->CCR3 != generation->ccr3) ||
        (TIM1->CCR4 != generation->ccr4))
    {
        return E_NOT_OK;
    }

    /* One-time metadata seed while CEN/UIE/JEOSIE remain disabled. */
    MotorControlCdd_BuildRuntimeGeneration(generation, &runtimeGeneration);
    MotorControlCdd_PreparedGeneration = runtimeGeneration;
    __DMB();
    MotorControlCdd_PreparedGenerationValid = TRUE;
    return E_OK;
}

Std_ReturnType MotorControlCdd_StartSampling(void)
{
    MotorControlCdd_GenerationType bootstrapGeneration;

    if ((MotorControlCdd_State != MOTOR_CONTROL_CDD_STATIC_SAFE) ||
        (MotorControlCdd_CalibrationState !=
         MOTOR_CONTROL_CDD_CALIBRATION_COMPLETE) ||
        (MotorControlCdd_CurrentOffsetsValid != TRUE) ||
        (MotorControlCdd_PreparedGenerationValid == TRUE) ||
        (MotorControlCdd_QueuedGenerationValid == TRUE) ||
        (MotorControlCdd_AreFaultEventsClear() != TRUE) ||
        (MotorControlCdd_IsCompleteStaticConfigurationValid() != TRUE))
    {
        return E_NOT_OK;
    }

    MotorControlCdd_BuildBootstrapGeneration(&bootstrapGeneration);
    if (MotorControlCdd_SeedBootstrapGeneration(&bootstrapGeneration) != E_OK)
    {
        return E_NOT_OK;
    }

    MotorControlCdd_GateAdcTrigger();
    MotorControlCdd_ClearAdcStatus(ADC_ISR_JEOS | ADC_ISR_JQOVF);
    MotorControlCdd_ClearTimStatus(TIM_SR_UIF);
    MotorControlCdd_DiscardNextSample = TRUE;
    MotorControlCdd_State = MOTOR_CONTROL_CDD_SAMPLING;
    ADC1->IER = ADC_IER_JEOSIE;
    TIM1->DIER = TIM_DIER_BIE | TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_CEN;

    if ((MotorControlCdd_IsEarlySafeStateValid() != TRUE) ||
        ((TIM1->CR2 & TIM_CR2_MMS) != 0U))
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_UpdateEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_UPDATE);
        return E_NOT_OK;
    }

    return E_OK;
}

Std_ReturnType MotorControlCdd_StopSampling(void)
{
    MotorControlCdd_ControlLifecycleType controlLifecycle =
        MotorControlCdd_LoadControlLifecycle();

    if ((MotorControlCdd_State != MOTOR_CONTROL_CDD_SAMPLING) ||
        ((controlLifecycle != MOTOR_CONTROL_CDD_CONTROL_UNAPPROVED) &&
         (controlLifecycle != MOTOR_CONTROL_CDD_CONTROL_IDLE)))
    {
        return E_NOT_OK;
    }

    MotorControlCdd_GateAdcTrigger();
    TIM1->DIER &= ~TIM_DIER_UIE;
    ADC1->IER &= ~ADC_IER_JEOSIE;
    if (((ADC1->CR & ADC_CR_JADSTART) != 0U) ||
        ((ADC1->JSQR & ADC_JSQR_JEXTEN) != 0U) ||
        (MotorControlCdd_QueuedGenerationValid == TRUE))
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_AdcEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC);
        return E_NOT_OK;
    }

    TIM1->CR1 &= ~TIM_CR1_CEN;
    MotorControlCdd_PreparedGenerationValid = FALSE;
    MotorControlCdd_QueuedGenerationValid = FALSE;
    __atomic_store_n(&MotorControlCdd_ConsumedGenerationId, 0U,
                     __ATOMIC_RELEASE);
    MotorControlCdd_DiscardNextSample = TRUE;
    TIM1->CCR1 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    TIM1->CCR2 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    TIM1->CCR3 = MOTOR_CONTROL_CDD_INITIAL_DUTY_COUNTS;
    TIM1->CCR4 = MOTOR_CONTROL_CDD_INITIAL_SAMPLE_COUNT;
    TIM1->CNT = 0U;
    TIM1->RCR = 1U;
    TIM1->EGR = TIM_EGR_UG;
    MotorControlCdd_ClearTimStatus(TIM_SR_UIF);
    TIM1->DIER = TIM_DIER_BIE;
    MotorControlCdd_State = MOTOR_CONTROL_CDD_STATIC_SAFE;

    if (MotorControlCdd_IsCompleteStaticConfigurationValid() != TRUE)
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_UpdateEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_UPDATE);
        return E_NOT_OK;
    }

    return E_OK;
}

Std_ReturnType MotorControlCdd_GetCurrentSample(
    MotorControlCdd_CurrentSampleType *Sample)
{
    uint32 slotIndex;
    uint32 newestGeneration;
    MotorControlCdd_CurrentSampleType newestSample;
    boolean sampleFound = FALSE;

    if (Sample == NULL_PTR)
    {
        return E_NOT_OK;
    }

    for (slotIndex = 0U;
         slotIndex < MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
         slotIndex++)
    {
        MotorControlCdd_CurrentSampleSlotType *slot =
            &MotorControlCdd_CurrentSampleMailbox[slotIndex];
        uint32 expected = MOTOR_CONTROL_CDD_MAILBOX_READY;

        if (__atomic_compare_exchange_n(
                &slot->state, &expected, MOTOR_CONTROL_CDD_MAILBOX_READING,
                FALSE, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
        {
            MotorControlCdd_CurrentSampleType candidate;

            MOTOR_CONTROL_CDD_TEST_MAILBOX_HOOK(
                MOTOR_CONTROL_CDD_TEST_MAILBOX_READING, slotIndex);
            candidate = slot->payload;
            __atomic_store_n(&slot->state, MOTOR_CONTROL_CDD_MAILBOX_FREE,
                             __ATOMIC_RELEASE);

            if ((sampleFound != TRUE) ||
                (((uint32)(candidate.generationId - newestSample.generationId) !=
                  0U) &&
                 ((uint32)(candidate.generationId - newestSample.generationId) <
                  0x80000000UL)))
            {
                newestSample = candidate;
                sampleFound = TRUE;
            }
        }
    }

    if (sampleFound != TRUE)
    {
        return E_NOT_OK;
    }

    newestGeneration =
        __atomic_load_n(&MotorControlCdd_ConsumedGenerationId,
                        __ATOMIC_ACQUIRE);
    if ((uint32)(newestGeneration - newestSample.generationId) >
        MOTOR_CONTROL_CDD_SAMPLE_MAX_AGE_GENERATIONS)
    {
        return E_NOT_OK;
    }

    *Sample = newestSample;
    return E_OK;
}

Std_ReturnType MotorControlCdd_GetSampleDropCount(uint32 *DropCount)
{
    if (DropCount == NULL_PTR)
    {
        return E_NOT_OK;
    }

    *DropCount =
        __atomic_load_n(&MotorControlCdd_SampleDropCount, __ATOMIC_ACQUIRE);
    return E_OK;
}

Std_ReturnType MotorControlCdd_GetCurrentGenerationId(uint32 *GenerationId)
{
    if (GenerationId == NULL_PTR)
    {
        return E_NOT_OK;
    }

    *GenerationId =
        __atomic_load_n(&MotorControlCdd_ConsumedGenerationId,
                        __ATOMIC_ACQUIRE);
    return E_OK;
}

STATIC boolean MotorControlCdd_FastSafetyRepresentationValid(
    const MotorControlCdd_FastSafetySnapshotType *snapshot)
{
    return ((snapshot != NULL_PTR) &&
            (MotorControlCdd_IsCanonicalBoolean(
                 snapshot->acquisitionValid) == TRUE) &&
            (MotorControlCdd_IsCanonicalBoolean(
                 snapshot->limitsValid) == TRUE) &&
            (MotorControlCdd_IsCanonicalBoolean(
                 snapshot->structuralValid) == TRUE) &&
            (MotorControlCdd_IsCanonicalBoolean(
                 snapshot->measurementApproved) == TRUE) &&
            (snapshot->identity.schemaVersion ==
             SC1_MEASUREMENT_SCHEMA_VERSION) &&
            (snapshot->identity.boardProfileCode ==
             SC1_MEASUREMENT_BOARD_PROFILE_CODE)) ? TRUE : FALSE;
}

STATIC boolean MotorControlCdd_FastSafetyIsUsable(
    const MotorControlCdd_FastSafetySnapshotType *snapshot,
    const Sc1_MeasurementIdentityType *identity,
    uint32 consumedGeneration)
{
    uint32 age;

    if ((MotorControlCdd_FastSafetyRepresentationValid(snapshot) != TRUE) ||
        (snapshot->acquisitionValid != TRUE) ||
        (snapshot->limitsValid != TRUE) ||
        (snapshot->structuralValid != TRUE) ||
        (snapshot->measurementApproved != TRUE) ||
        (MotorControlCdd_IdentityIsEqual(&snapshot->identity, identity) !=
         TRUE))
    {
        return FALSE;
    }
    age = consumedGeneration - snapshot->pwmGenerationId;
    if ((snapshot->pwmGenerationId != consumedGeneration) &&
        (age >= 0x80000000UL))
    {
        return FALSE;
    }
    return (age <= MOTOR_CONTROL_CDD_FAST_SAFETY_MAX_AGE_GENERATIONS) ?
        TRUE : FALSE;
}

Std_ReturnType MotorControlCdd_PublishSafetySnapshot(
    const MotorControlCdd_FastSafetySnapshotType *Snapshot)
{
    MotorControlCdd_ControlLifecycleType lifecycle;
    uint32 offset;

    if (Snapshot == NULL_PTR)
    {
        return E_NOT_OK;
    }
    lifecycle = MotorControlCdd_LoadControlLifecycle();
    if ((lifecycle == MOTOR_CONTROL_CDD_CONTROL_UNAPPROVED) ||
        (lifecycle == MOTOR_CONTROL_CDD_CONTROL_IDLE))
    {
        return E_OK;
    }
    if ((lifecycle != MOTOR_CONTROL_CDD_CONTROL_PRIMING) &&
        (lifecycle != MOTOR_CONTROL_CDD_CONTROL_ACTIVE) &&
        (lifecycle != MOTOR_CONTROL_CDD_CONTROL_STOP_REQUIRED))
    {
        return E_NOT_OK;
    }
    if (MotorControlCdd_FastSafetyRepresentationValid(Snapshot) != TRUE)
    {
        return E_NOT_OK;
    }

    for (offset = 0U;
         offset < MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
         offset++)
    {
        uint32 slotIndex = (MotorControlCdd_FastSafetyWriteCursor + offset) %
                           MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
        MotorControlCdd_FastSafetySlotType *slot =
            &MotorControlCdd_FastSafetyMailbox[slotIndex];
        uint32 expected = MOTOR_CONTROL_CDD_MAILBOX_FREE;

        if (__atomic_compare_exchange_n(
                &slot->state, &expected, MOTOR_CONTROL_CDD_MAILBOX_WRITING,
                FALSE, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
        {
            MOTOR_CONTROL_CDD_TEST_FAST_SAFETY_HOOK(
                MOTOR_CONTROL_CDD_TEST_MAILBOX_WRITING, slotIndex);
            slot->payload = *Snapshot;
            __atomic_store_n(&slot->state, MOTOR_CONTROL_CDD_MAILBOX_READY,
                             __ATOMIC_RELEASE);
            MOTOR_CONTROL_CDD_TEST_FAST_SAFETY_HOOK(
                MOTOR_CONTROL_CDD_TEST_MAILBOX_READY, slotIndex);
            MotorControlCdd_FastSafetyWriteCursor =
                (slotIndex + 1U) % MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
            return E_OK;
        }
    }
    return E_NOT_OK;
}

STATIC void MotorControlCdd_ConsumeNewestFastSafety(void)
{
    uint32 slotIndex;
    MotorControlCdd_FastSafetySnapshotType newest;
    boolean found = FALSE;

    for (slotIndex = 0U;
         slotIndex < MOTOR_CONTROL_CDD_MAILBOX_SLOT_COUNT;
         slotIndex++)
    {
        MotorControlCdd_FastSafetySlotType *slot =
            &MotorControlCdd_FastSafetyMailbox[slotIndex];
        uint32 expected = MOTOR_CONTROL_CDD_MAILBOX_READY;

        if (__atomic_compare_exchange_n(
                &slot->state, &expected, MOTOR_CONTROL_CDD_MAILBOX_READING,
                FALSE, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
        {
            MotorControlCdd_FastSafetySnapshotType candidate = slot->payload;

            MOTOR_CONTROL_CDD_TEST_FAST_SAFETY_HOOK(
                MOTOR_CONTROL_CDD_TEST_MAILBOX_READING, slotIndex);
            __atomic_store_n(&slot->state, MOTOR_CONTROL_CDD_MAILBOX_FREE,
                             __ATOMIC_RELEASE);
            if ((found != TRUE) ||
                (((uint32)(candidate.acquisitionSequence -
                           newest.acquisitionSequence) != 0U) &&
                 ((uint32)(candidate.acquisitionSequence -
                           newest.acquisitionSequence) < 0x80000000UL)))
            {
                newest = candidate;
                found = TRUE;
            }
        }
    }
    if (found == TRUE)
    {
        MotorControlCdd_RetainedFastSafety = newest;
        MotorControlCdd_RetainedFastSafetyValid = TRUE;
    }
}

STATIC void MotorControlCdd_FailControl(
    MotorControlCdd_ControlFailureType failure)
{
    MotorControlCdd_ControlFailureType expected =
        MOTOR_CONTROL_CDD_CONTROL_FAILURE_NONE;

    (void)__atomic_compare_exchange_n(
        &MotorControlCdd_ControlFailure, &expected, failure, FALSE,
        __ATOMIC_RELEASE, __ATOMIC_RELAXED);
    if (MotorControlCdd_LoadControlLifecycle() !=
        MOTOR_CONTROL_CDD_CONTROL_FAILED)
    {
        MotorControlCdd_StoreControlLifecycle(
            MOTOR_CONTROL_CDD_CONTROL_FAILED,
            MotorControlCdd_LoadControlEpoch());
    }
    MotorControlCdd_EnterPipelineFault(
        &MotorControlCdd_ControlEvent,
        MOTOR_CONTROL_CDD_EVENT_CAUSE_CONTROL);
}

Std_ReturnType MotorControlCdd_StageControl(
    const MotorControlCdd_ControlRequestType *Request)
{
    MotorControlCdd_CalibrationReadinessType readiness;
    MotorControlCdd_ControlActivationType candidate;
    MotorControlFastLoop_ConfigType fastLoopConfig;
    sint32 requestQ15;
    sint32 capQ15;
    sint32 alignQ15;
    sint32 alignDQ15;
    sint32 finalDQ15;
    uint32 currentGeneration;
    uint32 currentEpoch;
    uint32 expected;

    if ((Request == NULL_PTR) ||
        (MotorControlCdd_IsCanonicalBoolean(
             Request->commandContinueValid) != TRUE) ||
        (MotorControlCdd_IsCanonicalBoolean(Request->stopRequested) != TRUE) ||
        (Request->commandContinueValid != TRUE) ||
        (Request->stopRequested == TRUE) ||
        (Request->requestCurrentMa == 0) ||
        (MotorControlCdd_AbsS32(Request->requestCurrentMa) >
         MOTOR_CONTROL_CDD_COMMISSIONING_CURRENT_CAP_MA) ||
        (MotorControlCdd_State != MOTOR_CONTROL_CDD_SAMPLING) ||
        (MotorControlCdd_LoadControlLifecycle() !=
         MOTOR_CONTROL_CDD_CONTROL_IDLE) ||
        (MotorControlCdd_ControlApprovalsComplete() != TRUE) ||
        (MotorControlCdd_AreFaultEventsClear() != TRUE) ||
        (MotorControlCdd_IsEarlySafeStateValid() != TRUE) ||
        (MotorControlCdd_GetCalibrationReadiness(&readiness) != E_OK) ||
        (readiness.structuralValid != TRUE) ||
        (readiness.measurementApproved != TRUE) ||
        (readiness.calibrationComplete != TRUE) ||
        (readiness.offsetsInstalled != TRUE))
    {
        return E_NOT_OK;
    }

    currentGeneration = __atomic_load_n(&MotorControlCdd_ConsumedGenerationId,
                                         __ATOMIC_ACQUIRE);
    if ((currentGeneration == 0U) ||
        (MotorControlCdd_FastSafetyIsUsable(
             &Request->safetySnapshot,
             &readiness.identity,
             currentGeneration) != TRUE) ||
        (MotorControlCdd_ActiveControlConfig.identity.boardProfileCode !=
         readiness.identity.boardProfileCode) ||
        (MotorControlCdd_ActiveControlConfig.alignQReferenceMagnitudeMa >
         MotorControlCdd_AbsS32(Request->requestCurrentMa)))
    {
        return E_NOT_OK;
    }

    if ((MotorControlCdd_RoundPhysicalMaToQ15(
             Request->requestCurrentMa,
             MotorControlCdd_ActiveControlConfig.currentBaseMa,
             &requestQ15) != E_OK) ||
        (MotorControlCdd_RoundPhysicalMaToQ15(
             MOTOR_CONTROL_CDD_COMMISSIONING_CURRENT_CAP_MA,
             MotorControlCdd_ActiveControlConfig.currentBaseMa,
             &capQ15) != E_OK) ||
        (MotorControlCdd_RoundPhysicalMaToQ15(
             (Request->requestCurrentMa > 0) ?
                 (sint32)MotorControlCdd_ActiveControlConfig
                     .alignQReferenceMagnitudeMa :
                 -(sint32)MotorControlCdd_ActiveControlConfig
                     .alignQReferenceMagnitudeMa,
             MotorControlCdd_ActiveControlConfig.currentBaseMa,
             &alignQ15) != E_OK) ||
        (MotorControlCdd_RoundPhysicalMaToQ15(
             MotorControlCdd_ActiveControlConfig.alignDReferenceMa,
             MotorControlCdd_ActiveControlConfig.currentBaseMa,
             &alignDQ15) != E_OK) ||
        (MotorControlCdd_RoundPhysicalMaToQ15(
             MotorControlCdd_ActiveControlConfig.finalDReferenceMa,
             MotorControlCdd_ActiveControlConfig.currentBaseMa,
             &finalDQ15) != E_OK) ||
        (requestQ15 == 0) ||
        (MotorControlCdd_AbsS32(alignQ15) >
         MotorControlCdd_AbsS32(requestQ15)) ||
        (MotorControlCdd_AbsS32(alignDQ15) > (uint32)capQ15) ||
        (MotorControlCdd_AbsS32(finalDQ15) > (uint32)capQ15))
    {
        return E_NOT_OK;
    }

    expected = MOTOR_CONTROL_CDD_MAILBOX_FREE;
    if (!__atomic_compare_exchange_n(
            &MotorControlCdd_ControlStaging.state, &expected,
            MOTOR_CONTROL_CDD_MAILBOX_WRITING, FALSE,
            __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
    {
        return E_NOT_OK;
    }
    MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK(
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_WRITING);

    candidate = (MotorControlCdd_ControlActivationType){ 0 };
    fastLoopConfig = MotorControlCdd_ActiveControlConfig.fastLoop;
    fastLoopConfig.startup.alignQRefQ15 = alignQ15;
    fastLoopConfig.startup.finalQRefQ15 = requestQ15;
    fastLoopConfig.startup.alignDRefQ15 = alignDQ15;
    fastLoopConfig.startup.finalDRefQ15 = finalDQ15;
    fastLoopConfig.startup.absoluteCurrentCapQ15 = capQ15;
    if ((MotorControlFastLoop_Init(&candidate.fastLoop, &fastLoopConfig) !=
         E_OK) ||
        (MotorControlFastLoop_Begin(
             &candidate.fastLoop,
             (Request->requestCurrentMa > 0) ? 1 : -1,
             Request->alreadyConsumedStartingCycles) != E_OK))
    {
        __atomic_store_n(&MotorControlCdd_ControlStaging.state,
                         MOTOR_CONTROL_CDD_MAILBOX_FREE, __ATOMIC_RELEASE);
        return E_NOT_OK;
    }

    currentEpoch = MotorControlCdd_LoadControlEpoch();
    candidate.initialSafety = Request->safetySnapshot;
    candidate.sc1Identity = readiness.identity;
    candidate.sc2Identity = MotorControlCdd_ActiveControlConfig.identity;
    candidate.acceptedRequestQ15 = requestQ15;
    candidate.commissioningCapQ15 = capQ15;
    candidate.commandContinueValid = Request->commandContinueValid;
    candidate.stopRequested = Request->stopRequested;
    candidate.epoch = currentEpoch + 1U;
    MotorControlCdd_ControlStaging.payload = candidate;
    __atomic_store_n(&MotorControlCdd_ControlStaging.state,
                     MOTOR_CONTROL_CDD_MAILBOX_READY, __ATOMIC_RELEASE);
    MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK(
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READY);
    __atomic_store_n(&MotorControlCdd_AcceptedRequestQ15, requestQ15,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&MotorControlCdd_CommissioningCapQ15, capQ15,
                     __ATOMIC_RELEASE);
    MotorControlCdd_StoreControlLifecycle(
        MOTOR_CONTROL_CDD_CONTROL_PRIMING, candidate.epoch);
    MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK(
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_PUBLISHED);
    return E_OK;
}

Std_ReturnType MotorControlCdd_GetControlStatus(
    MotorControlCdd_ControlStatusType *Status)
{
    uint32 lifecycleEpoch;

    if (Status == NULL_PTR)
    {
        return E_NOT_OK;
    }
    lifecycleEpoch = __atomic_load_n(&MotorControlCdd_ControlLifecycleEpoch,
                                     __ATOMIC_ACQUIRE);
    Status->lifecycle = (MotorControlCdd_ControlLifecycleType)
        (lifecycleEpoch & MOTOR_CONTROL_CDD_CONTROL_LIFECYCLE_MASK);
    Status->epoch = lifecycleEpoch >>
                    MOTOR_CONTROL_CDD_CONTROL_EPOCH_SHIFT;
    Status->failure = __atomic_load_n(&MotorControlCdd_ControlFailure,
                                      __ATOMIC_ACQUIRE);
    Status->acceptedRequestQ15 =
        __atomic_load_n(&MotorControlCdd_AcceptedRequestQ15,
                        __ATOMIC_ACQUIRE);
    Status->commissioningCapQ15 =
        __atomic_load_n(&MotorControlCdd_CommissioningCapQ15,
                        __ATOMIC_ACQUIRE);
    return E_OK;
}

STATIC boolean MotorControlCdd_ClaimStagedControl(uint32 consumedGeneration)
{
    uint32 expected = MOTOR_CONTROL_CDD_MAILBOX_READY;

    if (MotorControlCdd_LoadControlLifecycle() !=
        MOTOR_CONTROL_CDD_CONTROL_PRIMING)
    {
        return FALSE;
    }
    if (!__atomic_compare_exchange_n(
            &MotorControlCdd_ControlStaging.state, &expected,
            MOTOR_CONTROL_CDD_MAILBOX_READING, FALSE,
            __ATOMIC_ACQUIRE, __ATOMIC_RELAXED))
    {
        MotorControlCdd_FailControl(
            MOTOR_CONTROL_CDD_CONTROL_FAILURE_INTERNAL);
        return FALSE;
    }
    MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK(
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_READING);

    MotorControlCdd_LiveControl = MotorControlCdd_ControlStaging.payload;
    if ((MotorControlCdd_LiveControl.epoch !=
         MotorControlCdd_LoadControlEpoch()) ||
        (MotorControlCdd_FastSafetyIsUsable(
             &MotorControlCdd_LiveControl.initialSafety,
             &MotorControlCdd_LiveControl.sc1Identity,
             consumedGeneration) != TRUE))
    {
        __atomic_store_n(&MotorControlCdd_ControlStaging.state,
                         MOTOR_CONTROL_CDD_MAILBOX_FREE, __ATOMIC_RELEASE);
        MotorControlCdd_FailControl(
            MOTOR_CONTROL_CDD_CONTROL_FAILURE_SAFETY);
        return FALSE;
    }

    MotorControlCdd_RetainedFastSafety =
        MotorControlCdd_LiveControl.initialSafety;
    MotorControlCdd_RetainedFastSafetyValid = TRUE;
    __atomic_store_n(&MotorControlCdd_ControlStaging.state,
                     MOTOR_CONTROL_CDD_MAILBOX_FREE, __ATOMIC_RELEASE);
    MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_HOOK(
        MOTOR_CONTROL_CDD_TEST_CONTROL_STAGING_FREE);
    MotorControlCdd_StoreControlLifecycle(
        MOTOR_CONTROL_CDD_CONTROL_ACTIVE,
        MotorControlCdd_LiveControl.epoch);
    return TRUE;
}

void MotorControlCdd_EmergencyOff(void)
{
    uint32 gpioBClockState = RCC->AHB2ENR;
    uint32 tim1ClockState = RCC->APB2ENR;

    if ((MotorControlCdd_EarlySafeInitialized == TRUE) &&
        ((gpioBClockState & RCC_AHB2ENR_GPIOBEN) != 0U) &&
        ((tim1ClockState & RCC_APB2ENR_TIM1EN) != 0U))
    {
        /* ENU/ENV/ENW low makes all half-bridges High-Z in MODE LOW. */
        GPIOB->BSRR = MOTOR_CONTROL_CDD_ENABLE_PIN_RESET_MASK;

        /* Reset/place the complete gate driver in standby. */
        GPIOB->BSRR = GPIO_BSRR_BR5;

        /* Timer bookkeeping is last; ENx is already fail-closed. */
        MotorControlCdd_ClearMainOutputEnable();
        __DSB();
    }
}

void TIM1_BRK_TIM15_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_BIF) != 0U)
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_BreakEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK);
        MotorControlCdd_ClearTimStatus(TIM_SR_BIF);
    }
    __DSB();
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    boolean updateValid = TRUE;
    uint32 updateArmLimit = MOTOR_CONTROL_CDD_UPDATE_ARM_LIMIT_COUNTS;

    if (MotorControlCdd_ControlApprovalsComplete() == TRUE)
    {
        updateArmLimit =
            MotorControlCdd_ActiveControlConfig.updateArmLimitCounts;
    }

    if (((TIM1->SR & TIM_SR_UIF) == 0U) ||
        (MotorControlCdd_State != MOTOR_CONTROL_CDD_SAMPLING) ||
        (MotorControlCdd_PreparedGenerationValid != TRUE) ||
        (MotorControlCdd_QueuedGenerationValid == TRUE) ||
        (MotorControlCdd_PreparedGeneration.generation.generationId !=
         (__atomic_load_n(&MotorControlCdd_ConsumedGenerationId,
                          __ATOMIC_ACQUIRE) + 1U)) ||
        ((TIM1->CR1 & TIM_CR1_DIR) != 0U) ||
        (TIM1->CNT > updateArmLimit) ||
        ((ADC1->CR & ADC_CR_JADSTART) != 0U) ||
        ((ADC1->JSQR & ADC_JSQR_JEXTEN) != 0U) ||
        ((ADC1->ISR & (ADC_ISR_JEOS | ADC_ISR_JQOVF)) != 0U) ||
        (TIM1->CCR1 != MotorControlCdd_PreparedGeneration.generation.ccr1) ||
        (TIM1->CCR2 != MotorControlCdd_PreparedGeneration.generation.ccr2) ||
        (TIM1->CCR3 != MotorControlCdd_PreparedGeneration.generation.ccr3) ||
        (TIM1->CCR4 != MotorControlCdd_PreparedGeneration.generation.ccr4) ||
        (MotorControlCdd_IsEarlySafeStateValid() != TRUE))
    {
        updateValid = FALSE;
    }

    MotorControlCdd_ClearTimStatus(TIM_SR_UIF);
    if (updateValid == TRUE)
    {
        MotorControlCdd_QueuedGeneration = MotorControlCdd_PreparedGeneration;
        MotorControlCdd_PreparedGenerationValid = FALSE;
        MotorControlCdd_QueuedGenerationValid = TRUE;
        __DMB();
        ADC1->JSQR = MotorControlCdd_QueuedGeneration.jsqr;
        TIM1->CR2 =
            (TIM1->CR2 & ~TIM_CR2_MMS) |
            MOTOR_CONTROL_CDD_TRGO_OC4REF_VALUE;
    }
    else
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_UpdateEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_UPDATE);
    }
    __DSB();
}

void ADC1_2_IRQHandler(void)
{
    boolean adcResultValid = TRUE;

    MotorControlCdd_GateAdcTrigger();
    if (((ADC1->ISR & ADC_ISR_JEOS) == 0U) ||
        ((ADC1->ISR & ADC_ISR_JQOVF) != 0U) ||
        (MotorControlCdd_State != MOTOR_CONTROL_CDD_SAMPLING) ||
        (MotorControlCdd_QueuedGenerationValid != TRUE) ||
        ((ADC1->JSQR & ADC_JSQR_JEXTEN) != 0U) ||
        (MotorControlCdd_IsEarlySafeStateValid() != TRUE))
    {
        adcResultValid = FALSE;
    }

    if (adcResultValid == TRUE)
    {
        uint32 firstRaw = ADC1->JDR1 & MOTOR_CONTROL_CDD_ADC_RAW_MASK;
        uint32 secondRaw = ADC1->JDR2 & MOTOR_CONTROL_CDD_ADC_RAW_MASK;
        sint32 phases[3] = {0, 0, 0};
        uint16 offsets[3] =
        {
            MotorControlCdd_CurrentOffsets.phaseA,
            MotorControlCdd_CurrentOffsets.phaseB,
            MotorControlCdd_CurrentOffsets.phaseC
        };
        MotorControlCdd_PhaseType firstPhase =
            MotorControlCdd_QueuedGeneration.generation.rank1Phase;
        MotorControlCdd_PhaseType secondPhase =
            MotorControlCdd_QueuedGeneration.generation.rank2Phase;
        MotorControlCdd_PhaseType reconstructedPhase =
            MotorControlCdd_QueuedGeneration.generation.reconstructedPhase;
        uint32 consumedGeneration =
            MotorControlCdd_QueuedGeneration.generation.generationId;
        MotorControlCdd_RuntimeGenerationType consumedRuntimeGeneration =
            MotorControlCdd_QueuedGeneration;
        MotorControlCdd_GenerationType nextGeneration;
        boolean publishValid =
            (MotorControlCdd_DiscardNextSample == TRUE) ? FALSE : TRUE;

        phases[firstPhase] = (sint32)firstRaw - (sint32)offsets[firstPhase];
        phases[secondPhase] = (sint32)secondRaw - (sint32)offsets[secondPhase];
        phases[reconstructedPhase] =
            -(phases[firstPhase] + phases[secondPhase]);

        __atomic_store_n(&MotorControlCdd_ConsumedGenerationId,
                         consumedGeneration, __ATOMIC_RELEASE);
        MotorControlCdd_QueuedGenerationValid = FALSE;
        MotorControlCdd_DiscardNextSample = FALSE;
        MotorControlCdd_PublishCurrentSample(
            (sint16)phases[MOTOR_CONTROL_CDD_PHASE_A],
            (sint16)phases[MOTOR_CONTROL_CDD_PHASE_B],
            (sint16)phases[MOTOR_CONTROL_CDD_PHASE_C],
            consumedGeneration,
            publishValid);
        MotorControlCdd_ClearAdcStatus(ADC_ISR_JEOS);
        if (MotorControlCdd_LoadControlLifecycle() ==
            MOTOR_CONTROL_CDD_CONTROL_PRIMING)
        {
            (void)MotorControlCdd_ClaimStagedControl(consumedGeneration);
            if (MotorControlCdd_State == MOTOR_CONTROL_CDD_SAMPLING)
            {
                MotorControlCdd_BuildNextGeneration(
                    &consumedRuntimeGeneration, &nextGeneration);
                (void)MotorControlCdd_CommitPreparedGeneration(
                    &nextGeneration);
            }
        }
        else if (MotorControlCdd_LoadControlLifecycle() ==
                 MOTOR_CONTROL_CDD_CONTROL_ACTIVE)
        {
            MotorControlFastLoop_StepInputType stepInput = { 0 };
            MotorControlFastLoop_ResultType stepResult;
            sint32 normalized[3];

            MotorControlCdd_ConsumeNewestFastSafety();
            if ((MotorControlCdd_RetainedFastSafetyValid != TRUE) ||
                (MotorControlCdd_FastSafetyIsUsable(
                     &MotorControlCdd_RetainedFastSafety,
                     &MotorControlCdd_LiveControl.sc1Identity,
                     consumedGeneration) != TRUE))
            {
                MotorControlCdd_FailControl(
                    MOTOR_CONTROL_CDD_CONTROL_FAILURE_SAFETY);
            }
            else if ((MotorControlCdd_NormalizeRawCurrent(
                          phases[MOTOR_CONTROL_CDD_PHASE_A],
                          &normalized[MOTOR_CONTROL_CDD_PHASE_A]) != E_OK) ||
                     (MotorControlCdd_NormalizeRawCurrent(
                          phases[MOTOR_CONTROL_CDD_PHASE_B],
                          &normalized[MOTOR_CONTROL_CDD_PHASE_B]) != E_OK) ||
                     (MotorControlCdd_NormalizeRawCurrent(
                          phases[MOTOR_CONTROL_CDD_PHASE_C],
                          &normalized[MOTOR_CONTROL_CDD_PHASE_C]) != E_OK))
            {
                MotorControlCdd_FailControl(
                    MOTOR_CONTROL_CDD_CONTROL_FAILURE_CURRENT_SCALE);
            }
            else
            {
                stepInput.phaseAQ15 =
                    normalized[MOTOR_CONTROL_CDD_PHASE_A];
                stepInput.phaseBQ15 =
                    normalized[MOTOR_CONTROL_CDD_PHASE_B];
                stepInput.phaseCQ15 =
                    normalized[MOTOR_CONTROL_CDD_PHASE_C];
                stepInput.sampleGenerationId = consumedGeneration;
                stepInput.appliedVoltage.alphaQ15 =
                    consumedRuntimeGeneration.generation
                        .appliedVoltageAlphaQ15;
                stepInput.appliedVoltage.betaQ15 =
                    consumedRuntimeGeneration.generation
                        .appliedVoltageBetaQ15;
                stepInput.appliedVoltageGenerationId = consumedGeneration;
                stepInput.currentSampleValid = TRUE;
                stepInput.appliedVoltageValid = TRUE;
                stepInput.generationOwnerValid = TRUE;
                stepInput.safetyValid = TRUE;
                stepInput.commandContinueValid =
                    MotorControlCdd_LiveControl.commandContinueValid;
                stepInput.stopRequested =
                    MotorControlCdd_LiveControl.stopRequested;
                if (MotorControlFastLoop_Step(
                        &MotorControlCdd_LiveControl.fastLoop,
                        &stepInput,
                        &stepResult) != E_OK)
                {
                    MotorControlCdd_FailControl(
                        MOTOR_CONTROL_CDD_CONTROL_FAILURE_FAST_LOOP);
                }
                else if (stepResult.stopRequired == TRUE)
                {
                    MotorControlCdd_StoreControlLifecycle(
                        MOTOR_CONTROL_CDD_CONTROL_STOP_REQUIRED,
                        MotorControlCdd_LiveControl.epoch);
                    MotorControlCdd_BuildBootstrapGeneration(&nextGeneration);
                    nextGeneration.generationId = consumedGeneration + 1U;
                    (void)MotorControlCdd_CommitPreparedGeneration(
                        &nextGeneration);
                }
                else if (MotorControlCdd_BuildControlGeneration(
                             &stepResult, &nextGeneration) != E_OK)
                {
                    MotorControlCdd_FailControl(
                        MOTOR_CONTROL_CDD_CONTROL_FAILURE_GENERATION);
                }
                else
                {
                    (void)MotorControlCdd_CommitPreparedGeneration(
                        &nextGeneration);
                }
            }
        }
        else if (MotorControlCdd_LoadControlLifecycle() !=
                 MOTOR_CONTROL_CDD_CONTROL_FAILED)
        {
            MotorControlCdd_BuildNextGeneration(
                &consumedRuntimeGeneration, &nextGeneration);
            (void)MotorControlCdd_CommitPreparedGeneration(&nextGeneration);
        }
    }
    else
    {
        MotorControlCdd_EnterPipelineFault(
            &MotorControlCdd_AdcEvent,
            MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC);
        MotorControlCdd_ClearAdcStatus(ADC_ISR_JEOS | ADC_ISR_JQOVF);
    }
    __DSB();
}
