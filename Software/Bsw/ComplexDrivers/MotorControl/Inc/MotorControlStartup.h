#ifndef MOTOR_CONTROL_STARTUP_H
#define MOTOR_CONTROL_STARTUP_H

#include "Std_Types.h"

#define MOTOR_CONTROL_STARTUP_MAX_CYCLES       ((uint32)100000UL)
#define MOTOR_CONTROL_STARTUP_Q15_MIN           ((sint32)-32768)
#define MOTOR_CONTROL_STARTUP_Q15_MAX           ((sint32)32767)
#define MOTOR_CONTROL_STARTUP_ANGLE_HALF_TURN   ((uint32)32768UL)

typedef enum
{
    MOTOR_CONTROL_STARTUP_PHASE_IDLE = 0,
    MOTOR_CONTROL_STARTUP_PHASE_ALIGN,
    MOTOR_CONTROL_STARTUP_PHASE_OPEN_LOOP_RAMP,
    MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY,
    MOTOR_CONTROL_STARTUP_PHASE_HANDOVER,
    MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM,
    MOTOR_CONTROL_STARTUP_PHASE_COMPLETE,
    MOTOR_CONTROL_STARTUP_PHASE_ABORTED,
    MOTOR_CONTROL_STARTUP_PHASE_FAILED
} MotorControlStartup_PhaseType;

typedef enum
{
    MOTOR_CONTROL_STARTUP_FAILURE_NONE = 0,
    MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC,
    MOTOR_CONTROL_STARTUP_FAILURE_GUARD,
    MOTOR_CONTROL_STARTUP_FAILURE_QUALIFICATION_TIMEOUT,
    MOTOR_CONTROL_STARTUP_FAILURE_HANDOVER_TIMEOUT,
    MOTOR_CONTROL_STARTUP_FAILURE_OBSERVER_LOSS,
    MOTOR_CONTROL_STARTUP_FAILURE_START_TIMEOUT
} MotorControlStartup_FailureType;

typedef struct
{
    uint32 alignCycles;
    uint32 openLoopRampCycles;
    uint32 qualificationCycles;
    uint32 handoverCycles;
    uint32 outputConfirmCycles;
    uint16 startAngleQ16;
    sint32 alignDRefQ15;
    sint32 alignQRefQ15;
    sint32 finalDRefQ15;
    sint32 finalQRefQ15;
    sint32 absoluteCurrentCapQ15;
    sint32 finalOpenLoopSpeedAngleCounts;
    uint32 qualificationRequiredCycles;
    uint32 minimumObservableSpeedAngleCounts;
    uint32 qualificationMaxAngleErrorQ16;
    uint32 qualificationMaxSpeedErrorAngleCounts;
    uint32 handoverMaxAngleCorrectionQ16;
    uint32 handoverMaxSpeedSlewAngleCounts;
    uint32 finalMaxAngleErrorQ16;
    uint32 finalMaxSpeedErrorAngleCounts;
} MotorControlStartup_ConfigType;

typedef struct
{
    uint16 angleQ16;
    sint32 speedAngleCounts;
    boolean numericValid;
    boolean angleValid;
    boolean speedValid;
    boolean varianceValid;
    boolean bemfValid;
    boolean currentValid;
    boolean directionValid;
} MotorControlStartup_ObserverSampleType;

typedef struct
{
    MotorControlStartup_ObserverSampleType observer;
    boolean generationValid;
    boolean safetyValid;
    boolean commandContinueValid;
    boolean stopRequested;
} MotorControlStartup_StepInputType;

typedef struct
{
    MotorControlStartup_PhaseType phase;
    uint16 selectedAngleQ16;
    sint32 selectedSpeedAngleCounts;
    sint32 dRefQ15;
    sint32 qRefQ15;
    boolean observerQualified;
    boolean handoverComplete;
    boolean stopRequired;
    boolean readyForRun;
    MotorControlStartup_FailureType failure;
} MotorControlStartup_ResultType;

typedef struct
{
    sint32 current;
    sint32 target;
    sint32 direction;
    uint32 baseIncrement;
    uint32 remainder;
    uint32 error;
    uint32 duration;
    uint32 elapsed;
} MotorControlStartup_RampType;

typedef struct
{
    MotorControlStartup_ConfigType config;
    MotorControlStartup_RampType dRefRamp;
    MotorControlStartup_RampType qRefRamp;
    MotorControlStartup_RampType speedRamp;
    uint16 selectedAngleQ16;
    sint32 selectedSpeedAngleCounts;
    sint32 dRefQ15;
    sint32 qRefQ15;
    uint32 phaseCycle;
    uint32 totalCycle;
    uint32 qualificationConsecutiveCycles;
    MotorControlStartup_PhaseType phase;
    MotorControlStartup_FailureType failure;
    boolean initialized;
    boolean observerQualified;
    boolean handoverComplete;
    boolean readyForRun;
} MotorControlStartup_Type;

/** Initialize one pure startup object. Call only during boot/system reset. */
Std_ReturnType MotorControlStartup_Init(
    MotorControlStartup_Type *Object,
    const MotorControlStartup_ConfigType *Config);

/** Begin ALIGN after nonzero DRIVER_RELEASE and ADC_PRIME cycle consumption. */
Std_ReturnType MotorControlStartup_Begin(
    MotorControlStartup_Type *Object,
    uint32 AlreadyConsumedStartingCycles);

/**
 * Validate the active phase's critical guards without advancing state or
 * evaluating controlled STOP, and copy its current q-axis reference.
 * MotorControlFastLoop is the sole production caller.
 */
Std_ReturnType MotorControlStartup_ValidateCycle(
    const MotorControlStartup_Type *Object,
    const MotorControlStartup_StepInputType *Input,
    sint32 *CurrentQRefQ15);

/** Execute one pure 20 kHz startup-control cycle. */
Std_ReturnType MotorControlStartup_Step(
    MotorControlStartup_Type *Object,
    const MotorControlStartup_StepInputType *Input,
    MotorControlStartup_ResultType *Result);

/**
 * Rearm only after the top-level owner completed STOPPING, verified safe
 * outputs and returned to READY. This API never clears FAILED.
 */
Std_ReturnType MotorControlStartup_RearmFromReady(
    MotorControlStartup_Type *Object);

#endif /* MOTOR_CONTROL_STARTUP_H */
