#ifndef MOTOR_CONTROL_FAST_LOOP_H
#define MOTOR_CONTROL_FAST_LOOP_H

#include "MotorControlAlgorithm.h"
#include "MotorControlObserver.h"
#include "MotorControlStartup.h"

#define MOTOR_CONTROL_FAST_LOOP_PHASE_SUM_MAX_Q15       ((uint32)98301UL)
#define MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30       ((uint32)262144UL)
#define MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MAX_Q30       ((uint32)1073741824UL)

typedef enum
{
    MOTOR_CONTROL_FAST_LOOP_UNINITIALIZED = 0,
    MOTOR_CONTROL_FAST_LOOP_IDLE,
    MOTOR_CONTROL_FAST_LOOP_ACTIVE,
    MOTOR_CONTROL_FAST_LOOP_STOP_REQUIRED,
    MOTOR_CONTROL_FAST_LOOP_FAILED
} MotorControlFastLoop_LifecycleType;

typedef enum
{
    MOTOR_CONTROL_FAST_LOOP_MODE_STARTUP = 0,
    MOTOR_CONTROL_FAST_LOOP_MODE_RUN
} MotorControlFastLoop_ModeType;

typedef enum
{
    MOTOR_CONTROL_FAST_LOOP_FAILURE_NONE = 0,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_CONFIGURATION,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_INTERNAL,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_INPUT_REPRESENTATION,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_GENERATION_PAIR,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_OBSERVER,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_STARTUP,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_GUARD,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_ALGORITHM,
    MOTOR_CONTROL_FAST_LOOP_FAILURE_NUMERIC
} MotorControlFastLoop_FailureType;

typedef struct
{
    MotorControlObserver_ConfigType observer;
    MotorControlStartup_ConfigType startup;
    MotorControlAlgorithm_PiConfigType dPi;
    MotorControlAlgorithm_PiConfigType qPi;
    uint32 phaseSumToleranceQ15;
    uint32 trigNormToleranceQ30;
} MotorControlFastLoop_ConfigType;

typedef struct
{
    sint32 phaseAQ15;
    sint32 phaseBQ15;
    sint32 phaseCQ15;
    uint32 sampleGenerationId;
    MotorControlAlgorithm_AlphaBetaType appliedVoltage;
    uint32 appliedVoltageGenerationId;
    boolean currentSampleValid;
    boolean appliedVoltageValid;
    boolean generationOwnerValid;
    boolean safetyValid;
    boolean commandContinueValid;
    boolean stopRequested;
} MotorControlFastLoop_StepInputType;

typedef struct
{
    MotorControlObserver_ResultType observer;
    MotorControlStartup_ResultType startup;
    MotorControlAlgorithm_AlphaBetaType measuredCurrent;
    MotorControlAlgorithm_DqType measuredCurrentDq;
    MotorControlAlgorithm_AlphaBetaType requestedVoltage;
    MotorControlAlgorithm_SvpwmResultType requestedSvpwm;
    uint16 selectedAngleQ16;
    sint32 selectedSpeedAngleCounts;
    sint32 dRefQ15;
    sint32 qRefQ15;
    uint32 sourceGenerationId;
    uint32 nextGenerationId;
    MotorControlFastLoop_ModeType mode;
    boolean controlValid;
    boolean stopRequired;
    boolean readyForRun;
    MotorControlFastLoop_LifecycleType lifecycle;
    MotorControlFastLoop_FailureType failure;
} MotorControlFastLoop_ResultType;

typedef struct
{
    MotorControlFastLoop_ConfigType config;
    MotorControlObserver_ObjectType observer;
    MotorControlStartup_Type startup;
    MotorControlAlgorithm_PiType dPi;
    MotorControlAlgorithm_PiType qPi;
    MotorControlStartup_ResultType retainedStartupResult;
    sint32 runDRefQ15;
    sint32 runQRefQ15;
    uint32 lastGenerationId;
    sint32 expectedDirection;
    MotorControlFastLoop_LifecycleType lifecycle;
    MotorControlFastLoop_ModeType mode;
    MotorControlFastLoop_FailureType failure;
    boolean generationEpochValid;
} MotorControlFastLoop_Type;

Std_ReturnType MotorControlFastLoop_Init(
    MotorControlFastLoop_Type *Object,
    const MotorControlFastLoop_ConfigType *Config);

Std_ReturnType MotorControlFastLoop_Begin(
    MotorControlFastLoop_Type *Object,
    sint32 ExpectedDirection,
    uint32 AlreadyConsumedStartingCycles);

Std_ReturnType MotorControlFastLoop_Step(
    MotorControlFastLoop_Type *Object,
    const MotorControlFastLoop_StepInputType *Input,
    MotorControlFastLoop_ResultType *Result);

Std_ReturnType MotorControlFastLoop_ResetFromReady(
    MotorControlFastLoop_Type *Object);

Std_ReturnType MotorControlFastLoop_SinCosQ15(
    uint16 AngleQ16,
    sint32 *SinQ15,
    sint32 *CosQ15);

#endif /* MOTOR_CONTROL_FAST_LOOP_H */
