#ifndef MOTOR_CONTROL_CDD_H
#define MOTOR_CONTROL_CDD_H

#include "Std_Types.h"
#include "MotorControlCdd_Cfg.h"

/**
 * @brief Establish and verify the software-controlled motor power-stage safe
 *        state before full clock, interrupt, ADC, PWM, OS or RTE startup.
 *
 * @pre Mcu_EarlyClockGateInit() returned E_OK.
 * @pre GPIOB and TIM1 are otherwise still in the reviewed reset/boot state.
 *
 * @return E_OK when PB5 and PB13-PB15 are push-pull outputs with pull-down,
 *         low-speed configuration and a low output latch, and TIM1 MOE is
 *         clear.
 * @return E_NOT_OK when the safe-state readback fails; the caller shall not
 *         continue boot.
 */
Std_ReturnType MotorControlCdd_EarlySafeInit(void);

/**
 * @brief Read back the early-safe power-stage state without modifying it.
 *
 * @return E_OK while PB5, PB13-PB15 and TIM1 MOE retain the reviewed safe
 *         configuration; otherwise E_NOT_OK.
 */
Std_ReturnType MotorControlCdd_CheckEarlySafeState(void);

/**
 * @brief Configure the CDD-owned motor GPIO, TIM1/BKIN and ADC1 static
 *        baseline while retaining the non-actuating state.
 *
 * The timer counter, ADC trigger, Update/JEOS interrupt sources, TIM1 MOE,
 * PB5 and PB13-PB15 remain disabled/low. Only the asynchronous hardware-break
 * interrupt source is armed after all configuration readbacks pass.
 *
 * @pre MotorControlCdd_EarlySafeInit, the production clock baseline,
 *      Mcu_Adc12PeripheralClockInit and Adc_Init returned E_OK.
 * @pre Called exactly once before StartOS with no concurrent CDD access.
 *
 * @return E_OK when every owned field and the frozen ADC common boundary read
 *         back correctly; otherwise E_NOT_OK after emergency-off.
 */
Std_ReturnType MotorControlCdd_Init(void);

/**
 * @brief Read back the complete non-actuating Step-5 static configuration.
 *
 * @return E_OK only while the CDD is initialized, the frozen ADC common
 *         configuration is valid, motor enable outputs are low, TIM1 MOE and
 *         CEN are clear, ADC triggering is gated, and every owned static field
 *         matches the reviewed baseline.
 */
Std_ReturnType MotorControlCdd_CheckStaticConfiguration(void);

typedef enum
{
    MOTOR_CONTROL_CDD_TRIGGER_RISING = 1,
    MOTOR_CONTROL_CDD_TRIGGER_FALLING = 2
} MotorControlCdd_TriggerEdgeType;

typedef enum
{
    MOTOR_CONTROL_CDD_PHASE_A = 0,
    MOTOR_CONTROL_CDD_PHASE_B,
    MOTOR_CONTROL_CDD_PHASE_C
} MotorControlCdd_PhaseType;

typedef struct
{
    uint32 generationId;
    uint16 ccr1;
    uint16 ccr2;
    uint16 ccr3;
    uint16 ccr4;
    uint8 sector;
    MotorControlCdd_PhaseType rank1Phase;
    MotorControlCdd_PhaseType rank2Phase;
    MotorControlCdd_PhaseType reconstructedPhase;
    MotorControlCdd_TriggerEdgeType triggerEdge;
    sint32 appliedVoltageAlphaQ15;
    sint32 appliedVoltageBetaQ15;
    boolean numericValid;
    boolean windowValid;
    boolean safetyValid;
} MotorControlCdd_GenerationType;

typedef enum
{
    MOTOR_CONTROL_CDD_CONTROL_UNAPPROVED = 0,
    MOTOR_CONTROL_CDD_CONTROL_IDLE,
    MOTOR_CONTROL_CDD_CONTROL_PRIMING,
    MOTOR_CONTROL_CDD_CONTROL_ACTIVE,
    MOTOR_CONTROL_CDD_CONTROL_STOP_REQUIRED,
    MOTOR_CONTROL_CDD_CONTROL_FAILED
} MotorControlCdd_ControlLifecycleType;

typedef enum
{
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_NONE = 0,
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_PRECONDITION,
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_CONFIGURATION,
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_SAFETY,
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_CURRENT_SCALE,
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_FAST_LOOP,
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_GENERATION,
    MOTOR_CONTROL_CDD_CONTROL_FAILURE_INTERNAL
} MotorControlCdd_ControlFailureType;

typedef struct
{
    uint16 vbusRaw;
    uint16 temperatureRaw;
    uint32 acquisitionSequence;
    uint32 osTick;
    uint32 pwmGenerationId;
    boolean acquisitionValid;
    boolean limitsValid;
    boolean structuralValid;
    boolean measurementApproved;
    Sc1_MeasurementIdentityType identity;
} MotorControlCdd_FastSafetySnapshotType;

typedef struct
{
    sint32 requestCurrentMa;
    uint32 alreadyConsumedStartingCycles;
    boolean commandContinueValid;
    boolean stopRequested;
    MotorControlCdd_FastSafetySnapshotType safetySnapshot;
} MotorControlCdd_ControlRequestType;

typedef struct
{
    MotorControlCdd_ControlLifecycleType lifecycle;
    MotorControlCdd_ControlFailureType failure;
    uint32 epoch;
    sint32 acceptedRequestQ15;
    sint32 commissioningCapQ15;
} MotorControlCdd_ControlStatusType;

typedef struct
{
    uint16 phaseA;
    uint16 phaseB;
    uint16 phaseC;
} MotorControlCdd_CurrentOffsetsType;

typedef struct
{
    sint16 phaseA;
    sint16 phaseB;
    sint16 phaseC;
    uint32 generationId;
    boolean valid;
} MotorControlCdd_CurrentSampleType;

#define MOTOR_CONTROL_CDD_CALIBRATION_RESULT_PHASE_COUNT 3U

typedef struct
{
    uint16 minimum;
    uint16 maximum;
    uint16 mean;
    uint16 peakToPeak;
} MotorControlCdd_CalibrationPhaseResultType;

typedef struct
{
    MotorControlCdd_CalibrationPhaseResultType
        phase[MOTOR_CONTROL_CDD_CALIBRATION_RESULT_PHASE_COUNT];
    uint32 firstSafetySequence;
    uint32 lastSafetySequence;
    uint32 firstSafetyTick;
    uint32 lastSafetyTick;
    uint16 vbusRawMinimum;
    uint16 vbusRawMaximum;
    uint16 temperatureRawMinimum;
    uint16 temperatureRawMaximum;
    Sc1_MeasurementIdentityType identity;
    boolean measurementApproved;
    boolean offsetsInstalled;
    boolean valid;
} MotorControlCdd_CalibrationResultType;

typedef struct
{
    uint16 vbusRaw;
    uint16 temperatureRaw;
    uint32 safetyAcquisitionSequence;
    uint32 safetyTick;
    Sc1_MeasurementIdentityType identity;
} MotorControlCdd_CalibrationEnvironmentType;

typedef struct
{
    Sc1_MeasurementIdentityType identity;
    boolean structuralValid;
    boolean measurementApproved;
    boolean calibrationComplete;
    boolean offsetsInstalled;
} MotorControlCdd_CalibrationReadinessType;

typedef enum
{
    MOTOR_CONTROL_CDD_CALIBRATION_ABORT_BOUNDED_CLEANUP = 1,
    MOTOR_CONTROL_CDD_CALIBRATION_ABORT_RESET_REQUIRED_NO_ADC = 2
} MotorControlCdd_CalibrationAbortModeType;

typedef enum
{
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_NONE = 0,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_PRECONDITION,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_CONFIG,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_ENVIRONMENT,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_REGISTER,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_FLAGS,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_SAMPLE,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_RESTORE,
    MOTOR_CONTROL_CDD_CALIBRATION_FAILURE_CLEANUP
} MotorControlCdd_CalibrationFailureType;

/** Start the bounded non-actuating SC-1 current-offset calibration. */
Std_ReturnType MotorControlCdd_StartCalibration(
    const MotorControlCdd_CalibrationEnvironmentType *Environment);

/** Advance the calibration by at most one accepted ADC regular result. */
Std_ReturnType MotorControlCdd_CalibrationStep(
    const MotorControlCdd_CalibrationEnvironmentType *Environment);

/** Request fault cleanup or terminal no-register reset handling. */
Std_ReturnType MotorControlCdd_RequestCalibrationAbort(
    MotorControlCdd_CalibrationAbortModeType Mode);

/** Advance exactly one bounded fault-cleanup state. */
Std_ReturnType MotorControlCdd_CalibrationCleanupStep(void);

/** Copy the completed characterization result. */
Std_ReturnType MotorControlCdd_GetCalibrationResult(
    MotorControlCdd_CalibrationResultType *Result);

/** Copy the immutable-config and calibration readiness summary. */
Std_ReturnType MotorControlCdd_GetCalibrationReadiness(
    MotorControlCdd_CalibrationReadinessType *Readiness);

/** Report whether normal calibration progress still owns ADC1 regular fields. */
boolean MotorControlCdd_IsCalibrationActive(void);

/** Report whether FAULT_LATCHED must continue bounded cleanup. */
boolean MotorControlCdd_IsCalibrationCleanupPending(void);

/** Acquire the reset-latched first local calibration failure cause. */
MotorControlCdd_CalibrationFailureType
MotorControlCdd_GetCalibrationFailureCause(void);

/**
 * Start only the TIM1/ADC1 sampling pipeline with every actuation control
 * retained safe. The CDD seeds the first immutable generation internally;
 * every later generation and all four compare shadows are written only by
 * the private JEOS/FOC path. No current API authorizes PWM output or a power
 * stage.
 */
Std_ReturnType MotorControlCdd_StartSampling(void);

/** Stop the sampling pipeline only after its ADC injected queue is empty. */
Std_ReturnType MotorControlCdd_StopSampling(void);

/**
 * Acquire and consume the newest non-expired reconstructed-current mailbox
 * sample. Older ready slots are retired without ever exposing a partially
 * written payload.
 */
Std_ReturnType MotorControlCdd_GetCurrentSample(
    MotorControlCdd_CurrentSampleType *Sample);

/** Acquire the number of samples dropped because all three slots were busy. */
Std_ReturnType MotorControlCdd_GetSampleDropCount(uint32 *DropCount);

/** Acquire the newest ADC1/PWM generation consumed by the JEOS path. */
Std_ReturnType MotorControlCdd_GetCurrentGenerationId(uint32 *GenerationId);

/** Copy one validated SafetySupervisor sample into the CDD fast mailbox. */
Std_ReturnType MotorControlCdd_PublishSafetySnapshot(
    const MotorControlCdd_FastSafetySnapshotType *Snapshot);

/**
 * Stage one complete control object for JEOS ownership. SC-2A intentionally
 * has no production caller and all production approval flags remain false.
 */
Std_ReturnType MotorControlCdd_StageControl(
    const MotorControlCdd_ControlRequestType *Request);

/** Acquire the immutable control lifecycle/epoch summary. */
Std_ReturnType MotorControlCdd_GetControlStatus(
    MotorControlCdd_ControlStatusType *Status);

#define MOTOR_CONTROL_CDD_EVENT_CAUSE_NONE             0x0000U
#define MOTOR_CONTROL_CDD_EVENT_CAUSE_BREAK            0x0001U
#define MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_UPDATE 0x0002U
#define MOTOR_CONTROL_CDD_EVENT_CAUSE_UNEXPECTED_ADC   0x0004U
#define MOTOR_CONTROL_CDD_EVENT_CAUSE_INVALID_GENERATION 0x0008U
#define MOTOR_CONTROL_CDD_EVENT_CAUSE_CALIBRATION      0x0010U
#define MOTOR_CONTROL_CDD_EVENT_CAUSE_CONTROL          0x0020U

typedef struct
{
    uint32 breakEvent;
    uint32 updateEvent;
    uint32 adcEvent;
    uint32 calibrationEvent;
    uint32 controlEvent;
} MotorControlCdd_FaultEventsType;

/**
 * @brief Acquire the producer-owned interrupt event words.
 *
 * Bits 31:16 contain the producer's monotonic sequence and bits 15:0 the
 * event cause. Event words are reset-latched and are never cleared here.
 *
 * @param Events Destination for the atomic snapshot.
 * @return E_OK for a valid destination, otherwise E_NOT_OK.
 */
Std_ReturnType MotorControlCdd_GetFaultEvents(
    MotorControlCdd_FaultEventsType *Events);

/**
 * @brief Force the complete motor power-stage command path to its safe state.
 *
 * The operation is idempotent and may be called from an approved Cat1 ISR,
 * NMI, or task context. It atomically clears the three half-bridge enables,
 * pulls the gate-driver standby/reset output low, and clears TIM1 MOE. It does
 * not allocate, wait, call an OS service, or publish a software event.
 *
 * Before MotorControlCdd_EarlySafeInit has completed successfully, this
 * function performs no GPIOB or TIM1 write. It also verifies both peripheral
 * clock gates before accessing either safety-owned peripheral.
 */
void MotorControlCdd_EmergencyOff(void);

#endif /* MOTOR_CONTROL_CDD_H */
