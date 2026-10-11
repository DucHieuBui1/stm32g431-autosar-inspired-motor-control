#include "MotorControlCdd_Cfg.h"

#define MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM 4095U
#define MOTOR_CONTROL_CDD_CFG_MEASUREMENT_APPROVED FALSE
#define MOTOR_CONTROL_CDD_CFG_RAIL_MINIMUM 0U
#define MOTOR_CONTROL_CDD_CFG_RAIL_MAXIMUM \
    MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM
#define MOTOR_CONTROL_CDD_CFG_MEAN_MINIMUM 0U
#define MOTOR_CONTROL_CDD_CFG_MEAN_MAXIMUM \
    MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM
#define MOTOR_CONTROL_CDD_CFG_NOISE_MAXIMUM \
    MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM
#define MOTOR_CONTROL_CDD_CFG_CURRENT_SCALE_APPROVED FALSE
#define MOTOR_CONTROL_CDD_CFG_CONTROL_PARAMETERS_APPROVED FALSE
#define MOTOR_CONTROL_CDD_CFG_SAMPLING_TIMING_APPROVED FALSE

#ifndef MOTOR_CONTROL_CDD_CFG_TEST_HOOK
#define MOTOR_CONTROL_CDD_CFG_TEST_HOOK(Config) ((void)(Config))
#endif

#ifndef MOTOR_CONTROL_CDD_CONTROL_CFG_TEST_HOOK
#define MOTOR_CONTROL_CDD_CONTROL_CFG_TEST_HOOK(Config) ((void)(Config))
#endif

#define MOTOR_CONTROL_CDD_CFG_PHASE_ENVELOPE \
    { MOTOR_CONTROL_CDD_CFG_RAIL_MINIMUM, \
      MOTOR_CONTROL_CDD_CFG_RAIL_MAXIMUM, \
      MOTOR_CONTROL_CDD_CFG_MEAN_MINIMUM, \
      MOTOR_CONTROL_CDD_CFG_MEAN_MAXIMUM, \
      MOTOR_CONTROL_CDD_CFG_NOISE_MAXIMUM }

STATIC const MotorControlCdd_CalibrationConfigType
    MotorControlCdd_CalibrationConfig =
{
    MOTOR_CONTROL_CDD_CFG_MEASUREMENT_APPROVED,
    MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT,
    {
        MOTOR_CONTROL_CDD_CFG_PHASE_ENVELOPE,
        MOTOR_CONTROL_CDD_CFG_PHASE_ENVELOPE,
        MOTOR_CONTROL_CDD_CFG_PHASE_ENVELOPE
    },
    {
        SC1_MEASUREMENT_SCHEMA_VERSION,
        SC1_MEASUREMENT_BOARD_PROFILE_CODE,
        { 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL }
    }
};

STATIC const MotorControlCdd_ControlConfigType MotorControlCdd_ControlConfig =
{
    .currentScaleApproved = MOTOR_CONTROL_CDD_CFG_CURRENT_SCALE_APPROVED,
    .controlParametersApproved =
        MOTOR_CONTROL_CDD_CFG_CONTROL_PARAMETERS_APPROVED,
    .samplingTimingApproved = MOTOR_CONTROL_CDD_CFG_SAMPLING_TIMING_APPROVED,
    .identity =
    {
        .schemaVersion = MOTOR_CONTROL_CDD_CONTROL_SCHEMA_VERSION,
        .boardProfileCode = SC1_MEASUREMENT_BOARD_PROFILE_CODE
    }
};

_Static_assert(MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT == 64U,
               "SC-1 calibration sample count must remain 64");
_Static_assert(MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM == 0x0FFFU,
               "SC-1 configuration must use the 12-bit ADC envelope");
_Static_assert(MOTOR_CONTROL_CDD_CFG_MEASUREMENT_APPROVED == FALSE,
               "SC-1A measurement approval must remain false");
_Static_assert(MOTOR_CONTROL_CDD_CFG_CURRENT_SCALE_APPROVED == FALSE,
               "SC-2A current-scale approval must remain false");
_Static_assert(MOTOR_CONTROL_CDD_CFG_CONTROL_PARAMETERS_APPROVED == FALSE,
               "SC-2A control-parameter approval must remain false");
_Static_assert(MOTOR_CONTROL_CDD_CFG_SAMPLING_TIMING_APPROVED == FALSE,
               "SC-2A sampling-timing approval must remain false");
_Static_assert(MOTOR_CONTROL_CDD_CFG_RAIL_MINIMUM <=
               MOTOR_CONTROL_CDD_CFG_RAIL_MAXIMUM,
               "SC-1 rail bounds are impossible");
_Static_assert(MOTOR_CONTROL_CDD_CFG_MEAN_MINIMUM <=
               MOTOR_CONTROL_CDD_CFG_MEAN_MAXIMUM,
               "SC-1 mean bounds are impossible");

STATIC boolean MotorControlCdd_CfgDigestIsZero(
    const Sc1_MeasurementIdentityType *identity)
{
    uint32 index;

    for (index = 0U;
         index < SC1_MEASUREMENT_IDENTITY_DIGEST_WORD_COUNT;
         index++)
    {
        if (identity->evidenceSha256[index] != 0U)
        {
            return FALSE;
        }
    }
    return TRUE;
}

STATIC boolean MotorControlCdd_CfgControlDigestIsZero(
    const uint32 *digest)
{
    uint32 index;

    for (index = 0U;
         index < MOTOR_CONTROL_CDD_CONTROL_IDENTITY_DIGEST_WORD_COUNT;
         index++)
    {
        if (digest[index] != 0U)
        {
            return FALSE;
        }
    }
    return TRUE;
}

STATIC boolean MotorControlCdd_CfgCanonicalBoolean(boolean value)
{
    return ((value == FALSE) || (value == TRUE)) ? TRUE : FALSE;
}

STATIC uint32 MotorControlCdd_CfgAbsS32(sint32 value)
{
    return (value >= 0) ? (uint32)value : (uint32)(-(sint64)value);
}

STATIC boolean MotorControlCdd_CfgQ15(sint32 value)
{
    return ((value >= -32768) && (value <= 32767)) ? TRUE : FALSE;
}

STATIC boolean MotorControlCdd_CfgPiValid(
    const MotorControlAlgorithm_PiConfigType *config)
{
    return ((config->kpQ15 >= 0) && (config->kpQ15 <= 32767) &&
            (config->kiQ15 >= 0) && (config->kiQ15 <= 32767) &&
            (MotorControlCdd_CfgQ15(config->lowerQ15) == TRUE) &&
            (MotorControlCdd_CfgQ15(config->upperQ15) == TRUE) &&
            (config->lowerQ15 < config->upperQ15)) ? TRUE : FALSE;
}

STATIC boolean MotorControlCdd_CfgObserverValid(
    const MotorControlObserver_ConfigType *config)
{
    uint64 maxBemfMagnitude;
    uint64 maxVarianceNumerator;
    uint64 speedSquared;
    uint64 windowSquared;

    if ((MotorControlCdd_CfgQ15(config->currentRetentionQ15) != TRUE) ||
        (MotorControlCdd_CfgQ15(config->voltageInjectionQ15) != TRUE) ||
        (MotorControlCdd_CfgQ15(config->bemfSubtractionQ15) != TRUE) ||
        (MotorControlCdd_CfgQ15(config->currentErrorToCurrentQ15) != TRUE) ||
        (MotorControlCdd_CfgQ15(config->currentErrorToBemfQ15) != TRUE) ||
        (MotorControlCdd_CfgQ15(config->phaseProportionalQ15) != TRUE) ||
        (MotorControlCdd_CfgQ15(config->phaseIntegralQ15) != TRUE) ||
        (config->currentRetentionQ15 < 0) ||
        (config->voltageInjectionQ15 < 0) ||
        (config->bemfSubtractionQ15 < 0) ||
        (config->currentErrorToCurrentQ15 < 0) ||
        (config->phaseProportionalQ15 < 0) ||
        (config->phaseIntegralQ15 < 0) ||
        (config->maxAbsCurrentQ15 <= 0) ||
        (config->maxAbsCurrentQ15 > 32767) ||
        (config->maxAbsBemfQ15 <= 0) ||
        (config->maxAbsBemfQ15 > 32767) ||
        (config->maxAbsSpeedAngleCounts == 0U) ||
        (config->maxAbsSpeedAngleCounts > 32767U) ||
        (config->maxSpeedDeltaAngleCounts == 0U) ||
        ((uint64)config->maxSpeedDeltaAngleCounts >
         (2ULL * config->maxAbsSpeedAngleCounts)) ||
        (config->speedWindowLength < 2U) ||
        (config->speedWindowLength >
         MOTOR_CONTROL_OBSERVER_MAX_SPEED_WINDOW) ||
        ((config->speedWindowLength &
          (config->speedWindowLength - 1U)) != 0U) ||
        (config->reliabilityWarmupSamples < config->speedWindowLength) ||
        (config->maxConsecutiveUnreliableSamples == 0U))
    {
        return FALSE;
    }
    maxBemfMagnitude = 2ULL * (uint64)config->maxAbsBemfQ15 *
                       (uint64)config->maxAbsBemfQ15;
    if ((config->minBemfMagnitudeSqQ30 == 0U) ||
        (config->minBemfMagnitudeSqQ30 >
         config->maxBemfMagnitudeSqQ30) ||
        (config->maxBemfMagnitudeSqQ30 > maxBemfMagnitude) ||
        (config->maxCurrentResidualMagnitudeSqQ30 == 0U) ||
        (config->maxCurrentResidualMagnitudeSqQ30 > 8589672450ULL))
    {
        return FALSE;
    }
    speedSquared = (uint64)config->maxAbsSpeedAngleCounts *
                   config->maxAbsSpeedAngleCounts;
    windowSquared = (uint64)config->speedWindowLength *
                    config->speedWindowLength;
    maxVarianceNumerator = windowSquared * speedSquared;
    return ((config->maxSpeedVarianceNumerator > 0U) &&
            (config->maxSpeedVarianceNumerator <= maxVarianceNumerator)) ?
        TRUE : FALSE;
}

STATIC boolean MotorControlCdd_CfgStartupTemplateValid(
    const MotorControlStartup_ConfigType *config)
{
    uint64 duration = (uint64)config->alignCycles +
                      (uint64)config->openLoopRampCycles +
                      (uint64)config->qualificationCycles +
                      (uint64)config->handoverCycles +
                      (uint64)config->outputConfirmCycles;

    return ((config->alignCycles > 0U) &&
            (config->openLoopRampCycles > 0U) &&
            (config->qualificationCycles > 0U) &&
            (config->handoverCycles > 0U) &&
            (config->outputConfirmCycles > 0U) &&
            (duration < MOTOR_CONTROL_STARTUP_MAX_CYCLES) &&
            (MotorControlCdd_CfgQ15(
                 config->finalOpenLoopSpeedAngleCounts) == TRUE) &&
            (config->finalOpenLoopSpeedAngleCounts != 0) &&
            (config->qualificationRequiredCycles > 0U) &&
            (config->qualificationRequiredCycles <=
             config->qualificationCycles) &&
            (config->minimumObservableSpeedAngleCounts > 0U) &&
            (config->minimumObservableSpeedAngleCounts <= 32768U) &&
            (config->qualificationMaxAngleErrorQ16 <= 32768U) &&
            (config->qualificationMaxSpeedErrorAngleCounts <= 32768U) &&
            (config->handoverMaxAngleCorrectionQ16 > 0U) &&
            (config->handoverMaxAngleCorrectionQ16 <= 32768U) &&
            (config->handoverMaxSpeedSlewAngleCounts > 0U) &&
            (config->handoverMaxSpeedSlewAngleCounts <= 32768U) &&
            (config->finalMaxAngleErrorQ16 <=
             config->handoverMaxAngleCorrectionQ16) &&
            (config->finalMaxSpeedErrorAngleCounts <= 32768U)) ? TRUE : FALSE;
}

STATIC boolean MotorControlCdd_CfgFastLoopTemplateValid(
    const MotorControlFastLoop_ConfigType *config)
{
    return ((MotorControlCdd_CfgObserverValid(&config->observer) == TRUE) &&
            (MotorControlCdd_CfgStartupTemplateValid(&config->startup) ==
             TRUE) &&
            (MotorControlCdd_CfgPiValid(&config->dPi) == TRUE) &&
            (MotorControlCdd_CfgPiValid(&config->qPi) == TRUE) &&
            (config->phaseSumToleranceQ15 <=
             MOTOR_CONTROL_FAST_LOOP_PHASE_SUM_MAX_Q15) &&
            (config->trigNormToleranceQ30 >=
             MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MIN_Q30) &&
            (config->trigNormToleranceQ30 <=
             MOTOR_CONTROL_FAST_LOOP_TRIG_NORM_MAX_Q30) &&
            (config->startup.minimumObservableSpeedAngleCounts <=
             config->observer.maxAbsSpeedAngleCounts) &&
            (MotorControlCdd_CfgAbsS32(
                 config->startup.finalOpenLoopSpeedAngleCounts) <=
             config->observer.maxAbsSpeedAngleCounts)) ? TRUE : FALSE;
}

STATIC boolean MotorControlCdd_CfgControlRepresentationIsZero(
    const MotorControlCdd_ControlConfigType *config)
{
    const uint8 *bytes = (const uint8 *)&config->fastLoop;
    uint32 byteIndex;

    for (byteIndex = 0U;
         byteIndex < (uint32)sizeof(config->fastLoop);
         byteIndex++)
    {
        if (bytes[byteIndex] != 0U)
        {
            return FALSE;
        }
    }

    return ((config->currentBaseMa == 0U) &&
            (config->rawMilliampNumerator == 0U) &&
            (config->rawMilliampDenominator == 0U) &&
            (config->rawToQ15Multiplier == 0U) &&
            (config->rawToQ15Shift == 0U) &&
            (config->rawScaleMaximumErrorMicroamp == 0U) &&
            (config->alignQReferenceMagnitudeMa == 0U) &&
            (config->alignDReferenceMa == 0) &&
            (config->finalDReferenceMa == 0) &&
            (config->switchingBlankingCounts == 0U) &&
            (config->triggerLatencyCounts == 0U) &&
            (config->apertureCounts == 0U) &&
            (config->rankSeparationCounts == 0U) &&
            (config->triggerArmMinimum == 0U) &&
            (config->commitMarginCounts == 0U) &&
            (config->updateArmLimitCounts == 0U) &&
            (config->commitCutoffCounts == 0U) &&
            (config->appliedVoltageMaximumErrorQ15 == 0U) &&
            (config->identity.firmwareIdentity == 0U)) ? TRUE : FALSE;
}

STATIC boolean MotorControlCdd_CfgControlIsValid(
    const MotorControlCdd_ControlConfigType *config)
{
    boolean anyApproved;
    boolean evidenceZero;
    boolean configurationZero;

    if ((MotorControlCdd_CfgCanonicalBoolean(
             config->currentScaleApproved) != TRUE) ||
        (MotorControlCdd_CfgCanonicalBoolean(
             config->controlParametersApproved) != TRUE) ||
        (MotorControlCdd_CfgCanonicalBoolean(
             config->samplingTimingApproved) != TRUE) ||
        (config->identity.schemaVersion !=
         MOTOR_CONTROL_CDD_CONTROL_SCHEMA_VERSION) ||
        (config->identity.boardProfileCode !=
         SC1_MEASUREMENT_BOARD_PROFILE_CODE))
    {
        return FALSE;
    }

    anyApproved = ((config->currentScaleApproved == TRUE) ||
                   (config->controlParametersApproved == TRUE) ||
                   (config->samplingTimingApproved == TRUE)) ? TRUE : FALSE;
    evidenceZero = MotorControlCdd_CfgControlDigestIsZero(
        config->identity.evidenceSha256);
    configurationZero = MotorControlCdd_CfgControlDigestIsZero(
        config->identity.configurationSha256);

    if (anyApproved != TRUE)
    {
        return ((evidenceZero == TRUE) && (configurationZero == TRUE) &&
                (MotorControlCdd_CfgControlRepresentationIsZero(config) ==
                 TRUE)) ? TRUE : FALSE;
    }

    if ((evidenceZero == TRUE) || (configurationZero == TRUE) ||
        (config->identity.firmwareIdentity == 0U) ||
        (config->currentBaseMa <= 300U) ||
        (config->rawMilliampNumerator == 0U) ||
        (config->rawMilliampDenominator == 0U) ||
        (config->rawToQ15Multiplier == 0U) ||
        (config->rawToQ15Shift == 0U) ||
        (config->rawToQ15Shift > 31U) ||
        (config->alignQReferenceMagnitudeMa > 300U) ||
        (MotorControlCdd_CfgAbsS32(config->alignDReferenceMa) > 300U) ||
        (MotorControlCdd_CfgAbsS32(config->finalDReferenceMa) > 300U) ||
        (MotorControlCdd_CfgFastLoopTemplateValid(&config->fastLoop) != TRUE) ||
        (config->switchingBlankingCounts >= 4250U) ||
        (config->triggerLatencyCounts >= 4250U) ||
        (config->apertureCounts == 0U) ||
        (config->apertureCounts >= 4250U) ||
        (config->rankSeparationCounts < config->apertureCounts) ||
        (config->rankSeparationCounts >= 4250U) ||
        (config->triggerArmMinimum == 0U) ||
        (config->triggerArmMinimum >= 4250U) ||
        (config->commitMarginCounts == 0U) ||
        (config->commitMarginCounts >= 4250U) ||
        (config->updateArmLimitCounts == 0U) ||
        (config->updateArmLimitCounts >= config->commitCutoffCounts) ||
        (config->commitCutoffCounts >= 4250U) ||
        (config->appliedVoltageMaximumErrorQ15 > 32767U))
    {
        return FALSE;
    }
    return TRUE;
}

STATIC boolean MotorControlCdd_CfgPhaseIsValid(
    const MotorControlCdd_CalibrationPhaseBoundsType *bounds,
    boolean approved)
{
    if ((bounds->absoluteRailMinimum > bounds->absoluteRailMaximum) ||
        (bounds->absoluteRailMaximum > MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM) ||
        (bounds->meanMinimum > bounds->meanMaximum) ||
        (bounds->meanMaximum > MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM) ||
        (bounds->maximumPeakToPeak > MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM))
    {
        return FALSE;
    }

    if (approved == TRUE)
    {
        if ((bounds->absoluteRailMinimum == 0U) ||
            (bounds->absoluteRailMaximum == MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM) ||
            (bounds->meanMinimum == 0U) ||
            (bounds->meanMaximum == MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM) ||
            (bounds->maximumPeakToPeak == MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM))
        {
            return FALSE;
        }
    }
    else if ((bounds->absoluteRailMinimum != 0U) ||
             (bounds->absoluteRailMaximum != MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM) ||
             (bounds->meanMinimum != 0U) ||
             (bounds->meanMaximum != MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM) ||
             (bounds->maximumPeakToPeak != MOTOR_CONTROL_CDD_CFG_ADC_MAXIMUM))
    {
        return FALSE;
    }
    else
    {
        /* Representation envelope is the only unapproved SC-1A profile. */
    }

    return TRUE;
}

STATIC boolean MotorControlCdd_CfgIsValid(
    const MotorControlCdd_CalibrationConfigType *config)
{
    uint32 phase;
    boolean digestIsZero;

    if (((config->measurementApproved != FALSE) &&
         (config->measurementApproved != TRUE)) ||
        (config->sampleCount != MOTOR_CONTROL_CDD_CALIBRATION_SAMPLE_COUNT) ||
        (config->identity.schemaVersion != SC1_MEASUREMENT_SCHEMA_VERSION) ||
        (config->identity.boardProfileCode !=
         SC1_MEASUREMENT_BOARD_PROFILE_CODE))
    {
        return FALSE;
    }

    digestIsZero = MotorControlCdd_CfgDigestIsZero(&config->identity);
    if (((config->measurementApproved == TRUE) && (digestIsZero == TRUE)) ||
        ((config->measurementApproved == FALSE) && (digestIsZero != TRUE)))
    {
        return FALSE;
    }

    for (phase = 0U;
         phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
         phase++)
    {
        if (MotorControlCdd_CfgPhaseIsValid(
                &config->phase[phase],
                config->measurementApproved) != TRUE)
        {
            return FALSE;
        }
    }
    return TRUE;
}

Std_ReturnType MotorControlCdd_CfgGetCalibration(
    MotorControlCdd_CalibrationConfigType *Config)
{
    if (Config == NULL_PTR)
    {
        return E_NOT_OK;
    }

    *Config = MotorControlCdd_CalibrationConfig;
    MOTOR_CONTROL_CDD_CFG_TEST_HOOK(Config);
    if (MotorControlCdd_CfgIsValid(Config) != TRUE)
    {
        *Config = (MotorControlCdd_CalibrationConfigType){ 0 };
        return E_NOT_OK;
    }
    return E_OK;
}

Std_ReturnType MotorControlCdd_CfgGetControl(
    MotorControlCdd_ControlConfigType *Config)
{
    if (Config == NULL_PTR)
    {
        return E_NOT_OK;
    }

    *Config = (MotorControlCdd_ControlConfigType){ 0 };
    *Config = MotorControlCdd_ControlConfig;
    MOTOR_CONTROL_CDD_CONTROL_CFG_TEST_HOOK(Config);
    if (MotorControlCdd_CfgControlIsValid(Config) != TRUE)
    {
        *Config = (MotorControlCdd_ControlConfigType){ 0 };
        return E_NOT_OK;
    }
    return E_OK;
}
