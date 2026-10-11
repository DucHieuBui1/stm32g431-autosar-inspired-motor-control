#include <stdio.h>

#include "MotorControlCdd_Cfg.h"
#include "SafetySupervisor_Cfg.h"

static uint32 Test_MotorMutation;
static uint32 Test_ControlMutation;
static uint32 Test_ControlApprovalMask;
static uint32 Test_SafetyMutation;

static void Test_MutateMotorConfig(
    MotorControlCdd_CalibrationConfigType *config);
static void Test_MutateControlConfig(
    MotorControlCdd_ControlConfigType *config);
static void Test_MutateSafetyConfig(SafetySupervisor_ConfigType *config);

#define MOTOR_CONTROL_CDD_CFG_TEST_HOOK(Config) Test_MutateMotorConfig(Config)
#define MOTOR_CONTROL_CDD_CONTROL_CFG_TEST_HOOK(Config) \
    Test_MutateControlConfig(Config)
#define SAFETY_SUPERVISOR_CFG_TEST_HOOK(Config) Test_MutateSafetyConfig(Config)
#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlCdd_Cfg.c"
#include "../../../Software/Bsw/Services/SafetySupervisor/Src/SafetySupervisor_Cfg.c"

#define TEST_CHECK(condition)                                             \
    do                                                                    \
    {                                                                     \
        if (!(condition))                                                 \
        {                                                                 \
            (void)printf("FAIL line %d: %s\n", __LINE__, #condition);     \
            return 1;                                                     \
        }                                                                 \
    } while (0)

static void Test_MutateMotorConfig(
    MotorControlCdd_CalibrationConfigType *config)
{
    uint32 phase;

    if (Test_MotorMutation == 1U)
    {
        config->measurementApproved = (boolean)2U;
    }
    else if (Test_MotorMutation == 2U)
    {
        config->sampleCount = 63U;
    }
    else if (Test_MotorMutation == 3U)
    {
        config->identity.evidenceSha256[0] = 1U;
    }
    else if (Test_MotorMutation == 4U)
    {
        config->phase[0].absoluteRailMaximum = 4096U;
    }
    else if (Test_MotorMutation == 5U)
    {
        config->measurementApproved = TRUE;
        config->identity.evidenceSha256[0] = 1U;
        for (phase = 0U;
             phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
             phase++)
        {
            config->phase[phase].absoluteRailMinimum = 1U;
            config->phase[phase].absoluteRailMaximum = 4094U;
            config->phase[phase].meanMinimum = 1U;
            config->phase[phase].meanMaximum = 4094U;
            config->phase[phase].maximumPeakToPeak = 4094U;
        }
    }
    else if (Test_MotorMutation == 6U)
    {
        config->identity.schemaVersion++;
    }
    else if (Test_MotorMutation == 7U)
    {
        config->identity.boardProfileCode++;
    }
    else if (Test_MotorMutation == 8U)
    {
        config->phase[1].absoluteRailMinimum = 1U;
        config->phase[1].absoluteRailMaximum = 0U;
    }
    else if (Test_MotorMutation == 9U)
    {
        config->phase[2].meanMinimum = 1U;
        config->phase[2].meanMaximum = 0U;
    }
    else if (Test_MotorMutation == 10U)
    {
        config->phase[0].maximumPeakToPeak = 4096U;
    }
    else if (Test_MotorMutation == 11U)
    {
        config->measurementApproved = TRUE;
        for (phase = 0U;
             phase < MOTOR_CONTROL_CDD_CALIBRATION_PHASE_COUNT;
             phase++)
        {
            config->phase[phase].absoluteRailMinimum = 1U;
            config->phase[phase].absoluteRailMaximum = 4094U;
            config->phase[phase].meanMinimum = 1U;
            config->phase[phase].meanMaximum = 4094U;
            config->phase[phase].maximumPeakToPeak = 4094U;
        }
    }
    else if (Test_MotorMutation == 12U)
    {
        config->measurementApproved = TRUE;
        config->identity.evidenceSha256[0] = 1U;
    }
    else if (Test_MotorMutation == 13U)
    {
        config->phase[0].meanMinimum = 1U;
    }
    else
    {
        /* No mutation. */
    }
}

static void Test_FillSyntheticControlConfig(
    MotorControlCdd_ControlConfigType *config)
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
    config->alignDReferenceMa = 0;
    config->finalDReferenceMa = 0;
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
    config->identity.schemaVersion = MOTOR_CONTROL_CDD_CONTROL_SCHEMA_VERSION;
    config->identity.boardProfileCode = SC1_MEASUREMENT_BOARD_PROFILE_CODE;
    config->identity.firmwareIdentity = 1U;
    config->identity.evidenceSha256[0] = 1U;
    config->identity.configurationSha256[0] = 2U;
}

static void Test_MutateControlConfig(
    MotorControlCdd_ControlConfigType *config)
{
    if (Test_ControlMutation == 0U)
    {
        return;
    }
    if (Test_ControlMutation == 1U)
    {
        config->currentScaleApproved = (boolean)2U;
        return;
    }
    Test_FillSyntheticControlConfig(config);
    switch (Test_ControlMutation)
    {
        case 2U: config->identity.schemaVersion++; break;
        case 3U: config->identity.boardProfileCode++; break;
        case 4U: config->currentBaseMa = 300U; break;
        case 5U: config->rawMilliampDenominator = 0U; break;
        case 6U: config->rawToQ15Multiplier = 0U; break;
        case 7U: config->rawToQ15Shift = 0U; break;
        case 8U: config->rawToQ15Shift = 32U; break;
        case 9U: config->apertureCounts = 0U; break;
        case 10U: config->rankSeparationCounts = 3U; break;
        case 11U: config->updateArmLimitCounts = 512U; break;
        case 12U: config->identity.evidenceSha256[0] = 0U; break;
        case 13U: config->identity.configurationSha256[0] = 0U; break;
        case 14U: config->identity.firmwareIdentity = 0U; break;
        case 15U: config->appliedVoltageMaximumErrorQ15 = 32768U; break;
        case 16U: config->triggerArmMinimum = 4250U; break;
        case 17U: config->commitMarginCounts = 4250U; break;
        case 19U: config->fastLoop.observer.maxAbsCurrentQ15 = 0; break;
        default: break;
    }
}

static void Test_MutateSafetyConfig(SafetySupervisor_ConfigType *config)
{
    if (Test_SafetyMutation == 1U)
    {
        config->measurementApproved = (boolean)2U;
    }
    else if (Test_SafetyMutation == 2U)
    {
        config->readySnapshotMaximumAgeMs = 2U;
    }
    else if (Test_SafetyMutation == 3U)
    {
        config->identity.evidenceSha256[0] = 1U;
    }
    else if (Test_SafetyMutation == 4U)
    {
        config->vbusRawMaximum = 4096U;
    }
    else if (Test_SafetyMutation == 5U)
    {
        config->measurementApproved = TRUE;
        config->vbusRawMinimum = 1U;
        config->vbusRawMaximum = 4094U;
        config->temperatureRawMinimum = 1U;
        config->temperatureRawMaximum = 4094U;
        config->identity.evidenceSha256[0] = 1U;
    }
    else if (Test_SafetyMutation == 6U)
    {
        config->identity.schemaVersion++;
    }
    else if (Test_SafetyMutation == 7U)
    {
        config->identity.boardProfileCode++;
    }
    else if (Test_SafetyMutation == 8U)
    {
        config->vbusRawMinimum = 1U;
        config->vbusRawMaximum = 0U;
    }
    else if (Test_SafetyMutation == 9U)
    {
        config->temperatureRawMaximum = 4096U;
    }
    else if (Test_SafetyMutation == 10U)
    {
        config->vbusMillivolts.denominator = 0U;
    }
    else if (Test_SafetyMutation == 11U)
    {
        config->temperatureCentiDegrees.denominator = 0U;
    }
    else if (Test_SafetyMutation == 12U)
    {
        config->measurementApproved = TRUE;
        config->vbusRawMinimum = 1U;
        config->vbusRawMaximum = 4094U;
        config->temperatureRawMinimum = 1U;
        config->temperatureRawMaximum = 4094U;
    }
    else if (Test_SafetyMutation == 13U)
    {
        config->measurementApproved = TRUE;
        config->identity.evidenceSha256[0] = 1U;
    }
    else if (Test_SafetyMutation == 14U)
    {
        config->vbusRawMinimum = 1U;
    }
    else
    {
        /* No mutation. */
    }
}

static int Test_DefaultSc1aProfilesAreCoherentAndUnapproved(void)
{
    MotorControlCdd_CalibrationConfigType motor;
    MotorControlCdd_ControlConfigType control;
    SafetySupervisor_ConfigType safety;
    uint32 index;

    Test_MotorMutation = 0U;
    Test_ControlMutation = 0U;
    Test_ControlApprovalMask = 0U;
    Test_SafetyMutation = 0U;
    TEST_CHECK(MotorControlCdd_CfgGetCalibration(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CfgGetControl(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(SafetySupervisor_CfgGet(NULL_PTR) == E_NOT_OK);
    TEST_CHECK(MotorControlCdd_CfgGetCalibration(&motor) == E_OK);
    TEST_CHECK(MotorControlCdd_CfgGetControl(&control) == E_OK);
    TEST_CHECK(SafetySupervisor_CfgGet(&safety) == E_OK);
    TEST_CHECK(motor.measurementApproved == FALSE);
    TEST_CHECK(safety.measurementApproved == FALSE);
    TEST_CHECK(control.currentScaleApproved == FALSE);
    TEST_CHECK(control.controlParametersApproved == FALSE);
    TEST_CHECK(control.samplingTimingApproved == FALSE);
    TEST_CHECK(control.currentBaseMa == 0U);
    TEST_CHECK(control.identity.schemaVersion ==
               MOTOR_CONTROL_CDD_CONTROL_SCHEMA_VERSION);
    TEST_CHECK(control.identity.boardProfileCode ==
               motor.identity.boardProfileCode);
    TEST_CHECK(motor.sampleCount == 64U);
    TEST_CHECK(safety.readySnapshotMaximumAgeMs == 1U);
    TEST_CHECK(motor.identity.schemaVersion == safety.identity.schemaVersion);
    TEST_CHECK(motor.identity.boardProfileCode ==
               safety.identity.boardProfileCode);
    for (index = 0U;
         index < SC1_MEASUREMENT_IDENTITY_DIGEST_WORD_COUNT;
         index++)
    {
        TEST_CHECK(motor.identity.evidenceSha256[index] == 0U);
        TEST_CHECK(safety.identity.evidenceSha256[index] == 0U);
        TEST_CHECK(control.identity.evidenceSha256[index] == 0U);
        TEST_CHECK(control.identity.configurationSha256[index] == 0U);
    }
    return 0;
}

static int Test_RuntimeCorruptionClearsDestinations(void)
{
    MotorControlCdd_CalibrationConfigType motor;
    SafetySupervisor_ConfigType safety;
    uint32 mutation;

    for (mutation = 1U; mutation <= 14U; mutation++)
    {
        if (mutation == 5U)
        {
            continue;
        }
        motor.measurementApproved = TRUE;
        motor.identity.schemaVersion = 99U;
        Test_MotorMutation = mutation;
        if (mutation <= 13U)
        {
            TEST_CHECK(MotorControlCdd_CfgGetCalibration(&motor) == E_NOT_OK);
            TEST_CHECK(motor.measurementApproved == FALSE);
            TEST_CHECK(motor.identity.schemaVersion == 0U);
        }

        safety.measurementApproved = TRUE;
        safety.identity.schemaVersion = 99U;
        Test_SafetyMutation = mutation;
        TEST_CHECK(SafetySupervisor_CfgGet(&safety) == E_NOT_OK);
        TEST_CHECK(safety.measurementApproved == FALSE);
        TEST_CHECK(safety.identity.schemaVersion == 0U);
    }
    return 0;
}

static int Test_ControlConfigurationApprovalAndCorruptionMatrix(void)
{
    MotorControlCdd_ControlConfigType control;
    uint32 approvalMask;
    uint32 mutation;

    for (approvalMask = 1U; approvalMask <= 7U; approvalMask++)
    {
        Test_ControlApprovalMask = approvalMask;
        Test_ControlMutation = 18U;
        TEST_CHECK(MotorControlCdd_CfgGetControl(&control) == E_OK);
        TEST_CHECK(control.currentScaleApproved ==
                   (((approvalMask & 1U) != 0U) ? TRUE : FALSE));
        TEST_CHECK(control.controlParametersApproved ==
                   (((approvalMask & 2U) != 0U) ? TRUE : FALSE));
        TEST_CHECK(control.samplingTimingApproved ==
                   (((approvalMask & 4U) != 0U) ? TRUE : FALSE));
        TEST_CHECK(control.identity.evidenceSha256[0] == 1U);
        TEST_CHECK(control.identity.configurationSha256[0] == 2U);
    }

    Test_ControlApprovalMask = 7U;
    for (mutation = 1U; mutation <= 19U; mutation++)
    {
        if (mutation == 18U)
        {
            continue;
        }
        control.currentScaleApproved = TRUE;
        control.identity.schemaVersion = 99U;
        Test_ControlMutation = mutation;
        TEST_CHECK(MotorControlCdd_CfgGetControl(&control) == E_NOT_OK);
        TEST_CHECK(control.currentScaleApproved == FALSE);
        TEST_CHECK(control.identity.schemaVersion == 0U);
    }

    Test_ControlMutation = 0U;
    Test_ControlApprovalMask = 0U;
    return 0;
}

static int Test_ApprovedShapeRequiresMeasuredIdentityAndProperWindows(void)
{
    MotorControlCdd_CalibrationConfigType motor;
    SafetySupervisor_ConfigType safety;

    Test_MotorMutation = 5U;
    Test_SafetyMutation = 5U;
    TEST_CHECK(MotorControlCdd_CfgGetCalibration(&motor) == E_OK);
    TEST_CHECK(SafetySupervisor_CfgGet(&safety) == E_OK);
    TEST_CHECK(motor.measurementApproved == TRUE);
    TEST_CHECK(safety.measurementApproved == TRUE);
    TEST_CHECK(motor.identity.evidenceSha256[0] == 1U);
    TEST_CHECK(safety.identity.evidenceSha256[0] == 1U);
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_DefaultSc1aProfilesAreCoherentAndUnapproved();
    failures += Test_RuntimeCorruptionClearsDestinations();
    failures += Test_ApprovedShapeRequiresMeasuredIdentityAndProperWindows();
    failures += Test_ControlConfigurationApprovalAndCorruptionMatrix();
    if (failures == 0)
    {
        (void)printf("PASS: SC-1/SC-2 immutable configuration matrix\n");
        return 0;
    }
    (void)printf("FAIL: %d SC-1/SC-2 configuration group(s)\n", failures);
    return 1;
}
