#ifndef MOTOR_CONTROL_OBSERVER_H
#define MOTOR_CONTROL_OBSERVER_H

#include "Std_Types.h"

#define MOTOR_CONTROL_OBSERVER_MAX_SPEED_WINDOW        ((uint32)32UL)
#define MOTOR_CONTROL_OBSERVER_Q15_MIN                 ((sint32)-32768)
#define MOTOR_CONTROL_OBSERVER_Q15_MAX                 ((sint32)32767)
#define MOTOR_CONTROL_OBSERVER_ANGLE_QUARTER_TURN      ((uint16)16384U)
#define MOTOR_CONTROL_OBSERVER_ANGLE_HALF_TURN         ((uint32)32768UL)
#define MOTOR_CONTROL_OBSERVER_ANGLE_FULL_TURN         ((uint32)65536UL)

typedef enum
{
    MOTOR_CONTROL_OBSERVER_UNINITIALIZED = 0,
    MOTOR_CONTROL_OBSERVER_IDLE,
    MOTOR_CONTROL_OBSERVER_ACTIVE,
    MOTOR_CONTROL_OBSERVER_FAILED
} MotorControlObserver_LifecycleType;

typedef enum
{
    MOTOR_CONTROL_OBSERVER_FAILURE_NONE = 0,
    MOTOR_CONTROL_OBSERVER_FAILURE_CONFIGURATION,
    MOTOR_CONTROL_OBSERVER_FAILURE_INTERNAL,
    MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_REPRESENTATION,
    MOTOR_CONTROL_OBSERVER_FAILURE_INPUT_QUALITY,
    MOTOR_CONTROL_OBSERVER_FAILURE_NUMERIC,
    MOTOR_CONTROL_OBSERVER_FAILURE_STATE_RANGE
} MotorControlObserver_FailureType;

typedef struct
{
    sint32 currentRetentionQ15;
    sint32 voltageInjectionQ15;
    sint32 bemfSubtractionQ15;
    sint32 currentErrorToCurrentQ15;
    sint32 currentErrorToBemfQ15;
    sint32 phaseProportionalQ15;
    sint32 phaseIntegralQ15;
    sint32 maxAbsCurrentQ15;
    sint32 maxAbsBemfQ15;
    uint32 maxAbsSpeedAngleCounts;
    uint32 maxSpeedDeltaAngleCounts;
    uint64 minBemfMagnitudeSqQ30;
    uint64 maxBemfMagnitudeSqQ30;
    uint64 maxCurrentResidualMagnitudeSqQ30;
    uint32 speedWindowLength;
    uint64 maxSpeedVarianceNumerator;
    uint32 reliabilityWarmupSamples;
    uint32 maxConsecutiveUnreliableSamples;
} MotorControlObserver_ConfigType;

typedef struct
{
    sint32 measuredCurrentAlphaQ15;
    sint32 measuredCurrentBetaQ15;
    sint32 appliedVoltageAlphaQ15;
    sint32 appliedVoltageBetaQ15;
    boolean currentSampleValid;
    boolean voltageSampleValid;
    boolean generationValid;
} MotorControlObserver_StepInputType;

typedef struct
{
    sint32 estimatedCurrentAlphaQ15;
    sint32 estimatedCurrentBetaQ15;
    sint32 estimatedBemfAlphaQ15;
    sint32 estimatedBemfBetaQ15;
    uint16 angleQ16;
    sint32 speedAngleCounts;
    uint64 currentResidualMagnitudeSqQ30;
    uint64 bemfMagnitudeSqQ30;
    uint64 speedVarianceNumerator;
    uint32 consecutiveUnreliableSamples;
    boolean numericValid;
    boolean angleValid;
    boolean speedValid;
    boolean varianceValid;
    boolean bemfValid;
    boolean currentValid;
    boolean directionValid;
    MotorControlObserver_LifecycleType lifecycle;
    MotorControlObserver_FailureType failure;
} MotorControlObserver_ResultType;

typedef struct
{
    MotorControlObserver_ConfigType config;
    sint32 estimatedCurrentAlphaQ15;
    sint32 estimatedCurrentBetaQ15;
    sint32 estimatedBemfAlphaQ15;
    sint32 estimatedBemfBetaQ15;
    uint16 angleQ16;
    sint64 speedIntegratorQ15;
    sint32 speedAngleCounts;
    sint32 speedRing[MOTOR_CONTROL_OBSERVER_MAX_SPEED_WINDOW];
    sint64 speedSum;
    uint64 speedSumSquares;
    uint32 speedWindowFill;
    uint32 speedWindowIndex;
    uint32 warmupSamples;
    uint32 consecutiveUnreliableSamples;
    sint32 expectedDirection;
    MotorControlObserver_LifecycleType lifecycle;
    MotorControlObserver_FailureType failure;
} MotorControlObserver_ObjectType;

Std_ReturnType MotorControlObserver_Init(
    MotorControlObserver_ObjectType *Object,
    const MotorControlObserver_ConfigType *Config);

Std_ReturnType MotorControlObserver_Begin(
    MotorControlObserver_ObjectType *Object,
    uint16 InitialAngleQ16,
    sint32 ExpectedDirection);

Std_ReturnType MotorControlObserver_Step(
    MotorControlObserver_ObjectType *Object,
    const MotorControlObserver_StepInputType *Input,
    MotorControlObserver_ResultType *Result);

Std_ReturnType MotorControlObserver_ResetFromReady(
    MotorControlObserver_ObjectType *Object);

#endif /* MOTOR_CONTROL_OBSERVER_H */
