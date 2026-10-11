#ifndef MOTOR_CONTROL_CDD_CFG_H
#define MOTOR_CONTROL_CDD_CFG_H

#include "MotorControlFastLoop.h"
#include "Sc1_MeasurementIdentity.h"

#define MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT 3U
#define MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT 64U

typedef struct
{
    uint16 absoluteRailMinimum;
    uint16 absoluteRailMaximum;
    uint16 meanMinimum;
    uint16 meanMaximum;
    uint16 maximumPeakToPeak;
} MotorControlCdd_CalibrationPhaseBoundsType;

typedef struct
{
    boolean measurementApproved;
    uint16 sampleCount;
    MotorControlCdd_CalibrationPhaseBoundsType
        phase[MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT];
    Sc1_MeasurementIdentityType identity;
} MotorControlCdd_CalibrationConfigType;

#define MOTOR_CONTROL_CDD_CONTROL_IDENTITY_DIGEST_WORD_COUNT 8U
#define MOTOR_CONTROL_CDD_CONTROL_SCHEMA_VERSION             1UL

typedef struct
{
    uint32 schemaVersion;
    uint32 boardProfileCode;
    uint32 firmwareIdentity;
    uint32 evidenceSha256[
        MOTOR_CONTROL_CDD_CONTROL_IDENTITY_DIGEST_WORD_COUNT];
    uint32 configurationSha256[
        MOTOR_CONTROL_CDD_CONTROL_IDENTITY_DIGEST_WORD_COUNT];
} MotorControlCdd_ControlIdentityType;

typedef struct
{
    boolean currentScaleApproved;
    boolean controlParametersApproved;
    boolean samplingTimingApproved;
    uint32 currentBaseMa;
    uint32 rawMilliampNumerator;
    uint32 rawMilliampDenominator;
    uint32 rawToQ15Multiplier;
    uint8 rawToQ15Shift;
    uint32 rawScaleMaximumErrorMicroamp;
    uint32 alignQReferenceMagnitudeMa;
    sint32 alignDReferenceMa;
    sint32 finalDReferenceMa;
    MotorControlFastLoop_ConfigType fastLoop;
    uint16 switchingBlankingCounts;
    uint16 triggerLatencyCounts;
    uint16 apertureCounts;
    uint16 rankSeparationCounts;
    uint16 triggerArmMinimum;
    uint16 commitMarginCounts;
    uint16 updateArmLimitCounts;
    uint16 commitCutoffCounts;
    uint16 appliedVoltageMaximumErrorQ15;
    MotorControlCdd_ControlIdentityType identity;
} MotorControlCdd_ControlConfigType;

/** Copy and validate the immutable SC-1 current-offset configuration. */
Std_ReturnType MotorControlCdd_CfgGetCalibration(
    MotorControlCdd_CalibrationConfigType *Config);

/** Copy and validate the immutable SC-2 control-composition configuration. */
Std_ReturnType MotorControlCdd_CfgGetControl(
    MotorControlCdd_ControlConfigType *Config);

#endif /* MOTOR_CONTROL_CDD_CFG_H */
