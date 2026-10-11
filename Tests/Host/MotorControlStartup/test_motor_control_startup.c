#include <stdio.h>
#include <string.h>

#include "MotorControlStartup.h"

#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlStartup.c"

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

typedef enum
{
    TEST_CFG_ALIGN_CYCLES = 0,
    TEST_CFG_RAMP_CYCLES,
    TEST_CFG_QUAL_CYCLES,
    TEST_CFG_HANDOVER_CYCLES,
    TEST_CFG_CONFIRM_CYCLES,
    TEST_CFG_ALIGN_D,
    TEST_CFG_ALIGN_Q,
    TEST_CFG_FINAL_D,
    TEST_CFG_FINAL_Q,
    TEST_CFG_CAP,
    TEST_CFG_SPEED,
    TEST_CFG_QUAL_REQUIRED,
    TEST_CFG_MIN_SPEED,
    TEST_CFG_QUAL_ANGLE,
    TEST_CFG_QUAL_SPEED,
    TEST_CFG_HANDOVER_ANGLE,
    TEST_CFG_HANDOVER_SPEED,
    TEST_CFG_FINAL_ANGLE,
    TEST_CFG_FINAL_SPEED
} Test_ConfigFieldType;

typedef struct
{
    Test_ConfigFieldType field;
    sint64 value;
} Test_InvalidConfigCaseType;

STATIC MotorControlStartup_ConfigType Test_DefaultConfig(void)
{
    MotorControlStartup_ConfigType config =
    {
        2U, 4U, 3U, 2U, 2U,
        65530U,
        100, -100, -103, 103, 2000,
        4,
        2U, 4U, 32U, 0U,
        8U, 2U, 8U, 0U
    };

    return config;
}

STATIC MotorControlStartup_StepInputType Test_ValidInput(void)
{
    MotorControlStartup_StepInputType input;

    input.observer.angleQ16 = 0U;
    input.observer.speedAngleCounts = 4;
    input.observer.numericValid = TRUE;
    input.observer.angleValid = TRUE;
    input.observer.speedValid = TRUE;
    input.observer.varianceValid = TRUE;
    input.observer.bemfValid = TRUE;
    input.observer.currentValid = TRUE;
    input.observer.directionValid = TRUE;
    input.generationValid = TRUE;
    input.safetyValid = TRUE;
    input.commandContinueValid = TRUE;
    input.stopRequested = FALSE;
    return input;
}

STATIC boolean Test_ObjectIsZero(const MotorControlStartup_Type *object)
{
    const uint8 *bytes = (const uint8 *)object;
    uint32 byteIndex;

    for (byteIndex = 0U;
         byteIndex < (uint32)sizeof(MotorControlStartup_Type);
         byteIndex++)
    {
        if (bytes[byteIndex] != 0U)
        {
            return FALSE;
        }
    }
    return TRUE;
}

STATIC void Test_SetConfigField(
    MotorControlStartup_ConfigType *config,
    Test_ConfigFieldType field,
    sint64 value)
{
    switch (field)
    {
        case TEST_CFG_ALIGN_CYCLES:
            config->alignCycles = (uint32)value;
            break;
        case TEST_CFG_RAMP_CYCLES:
            config->openLoopRampCycles = (uint32)value;
            break;
        case TEST_CFG_QUAL_CYCLES:
            config->qualificationCycles = (uint32)value;
            break;
        case TEST_CFG_HANDOVER_CYCLES:
            config->handoverCycles = (uint32)value;
            break;
        case TEST_CFG_CONFIRM_CYCLES:
            config->outputConfirmCycles = (uint32)value;
            break;
        case TEST_CFG_ALIGN_D:
            config->alignDRefQ15 = (sint32)value;
            break;
        case TEST_CFG_ALIGN_Q:
            config->alignQRefQ15 = (sint32)value;
            break;
        case TEST_CFG_FINAL_D:
            config->finalDRefQ15 = (sint32)value;
            break;
        case TEST_CFG_FINAL_Q:
            config->finalQRefQ15 = (sint32)value;
            break;
        case TEST_CFG_CAP:
            config->absoluteCurrentCapQ15 = (sint32)value;
            break;
        case TEST_CFG_SPEED:
            config->finalOpenLoopSpeedAngleCounts = (sint32)value;
            break;
        case TEST_CFG_QUAL_REQUIRED:
            config->qualificationRequiredCycles = (uint32)value;
            break;
        case TEST_CFG_MIN_SPEED:
            config->minimumObservableSpeedAngleCounts = (uint32)value;
            break;
        case TEST_CFG_QUAL_ANGLE:
            config->qualificationMaxAngleErrorQ16 = (uint32)value;
            break;
        case TEST_CFG_QUAL_SPEED:
            config->qualificationMaxSpeedErrorAngleCounts = (uint32)value;
            break;
        case TEST_CFG_HANDOVER_ANGLE:
            config->handoverMaxAngleCorrectionQ16 = (uint32)value;
            break;
        case TEST_CFG_HANDOVER_SPEED:
            config->handoverMaxSpeedSlewAngleCounts = (uint32)value;
            break;
        case TEST_CFG_FINAL_ANGLE:
            config->finalMaxAngleErrorQ16 = (uint32)value;
            break;
        default:
            config->finalMaxSpeedErrorAngleCounts = (uint32)value;
            break;
    }
}

STATIC int Test_ReachQualification(
    MotorControlStartup_Type *object,
    const MotorControlStartup_ConfigType *config,
    MotorControlStartup_StepInputType *input)
{
    MotorControlStartup_ResultType result;
    uint32 cycle;

    TEST_CHECK(MotorControlStartup_Init(object, config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(object, 10U) == E_OK);
    for (cycle = 0U; cycle < config->alignCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(object, input, &result) == E_OK);
    }
    for (cycle = 0U; cycle < config->openLoopRampCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(object, input, &result) == E_OK);
    }
    TEST_CHECK(object->phase ==
               MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY);
    return 0;
}

STATIC int Test_ReachHandover(
    MotorControlStartup_Type *object,
    const MotorControlStartup_ConfigType *config,
    MotorControlStartup_StepInputType *input)
{
    MotorControlStartup_ResultType result;
    uint32 cycle;

    TEST_CHECK(Test_ReachQualification(object, config, input) == 0);
    input->observer.speedAngleCounts =
        config->finalOpenLoopSpeedAngleCounts;
    for (cycle = 0U;
         cycle < config->qualificationRequiredCycles;
         cycle++)
    {
        input->observer.angleQ16 =
            (uint16)(object->selectedAngleQ16 +
                     (uint16)object->selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(object, input, &result) == E_OK);
    }
    TEST_CHECK(object->phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER);
    return 0;
}

STATIC int Test_ReachOutputConfirm(
    MotorControlStartup_Type *object,
    const MotorControlStartup_ConfigType *config,
    MotorControlStartup_StepInputType *input)
{
    MotorControlStartup_ResultType result;
    uint32 cycle;

    TEST_CHECK(Test_ReachHandover(object, config, input) == 0);
    for (cycle = 0U; cycle < config->handoverCycles; cycle++)
    {
        input->observer.speedAngleCounts = object->selectedSpeedAngleCounts;
        input->observer.angleQ16 =
            (uint16)(object->selectedAngleQ16 +
                     (uint16)object->selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(object, input, &result) == E_OK);
    }
    TEST_CHECK(object->phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM);
    return 0;
}

STATIC int Test_InitializationAndConfigMatrix(void)
{
    static const Test_InvalidConfigCaseType invalidCases[] =
    {
        {TEST_CFG_ALIGN_CYCLES, 0},
        {TEST_CFG_RAMP_CYCLES, 0},
        {TEST_CFG_QUAL_CYCLES, 0},
        {TEST_CFG_HANDOVER_CYCLES, 0},
        {TEST_CFG_CONFIRM_CYCLES, 0},
        {TEST_CFG_ALIGN_D, -32769}, {TEST_CFG_ALIGN_D, 32768},
        {TEST_CFG_ALIGN_Q, -32769}, {TEST_CFG_ALIGN_Q, 32768},
        {TEST_CFG_FINAL_D, -32769}, {TEST_CFG_FINAL_D, 32768},
        {TEST_CFG_FINAL_Q, -32769}, {TEST_CFG_FINAL_Q, 32768},
        {TEST_CFG_CAP, 0}, {TEST_CFG_CAP, 32768},
        {TEST_CFG_SPEED, -32769}, {TEST_CFG_SPEED, 0},
        {TEST_CFG_SPEED, 32768},
        {TEST_CFG_QUAL_REQUIRED, 0}, {TEST_CFG_QUAL_REQUIRED, 4},
        {TEST_CFG_MIN_SPEED, 0}, {TEST_CFG_MIN_SPEED, 32769},
        {TEST_CFG_QUAL_ANGLE, 32769}, {TEST_CFG_QUAL_SPEED, 32769},
        {TEST_CFG_HANDOVER_ANGLE, 0},
        {TEST_CFG_HANDOVER_ANGLE, 32769},
        {TEST_CFG_HANDOVER_SPEED, 0},
        {TEST_CFG_HANDOVER_SPEED, 32769},
        {TEST_CFG_FINAL_ANGLE, 9}, {TEST_CFG_FINAL_SPEED, 32769}
    };
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_Type object;
    uint32 caseIndex;

    TEST_CHECK(MotorControlStartup_Init(NULL_PTR, &config) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Init(&object, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(Test_ObjectIsZero(&object) == TRUE);

    for (caseIndex = 0U;
         caseIndex <
             (uint32)(sizeof(invalidCases) / sizeof(invalidCases[0]));
         caseIndex++)
    {
        config = Test_DefaultConfig();
        Test_SetConfigField(
            &config, invalidCases[caseIndex].field,
            invalidCases[caseIndex].value);
        object.initialized = TRUE;
        object.totalCycle = 123U;
        TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_NOT_OK);
        TEST_CHECK(Test_ObjectIsZero(&object) == TRUE);
    }

    config = Test_DefaultConfig();
    config.alignCycles = 99996U;
    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_NOT_OK);
    TEST_CHECK(Test_ObjectIsZero(&object) == TRUE);
    config = Test_DefaultConfig();
    config.absoluteCurrentCapQ15 = 100;
    config.finalQRefQ15 = 101;
    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_NOT_OK);
    config = Test_DefaultConfig();
    config.absoluteCurrentCapQ15 = 32767;
    config.alignDRefQ15 = -32767;
    config.finalDRefQ15 = 32767;
    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(object.initialized == TRUE);
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE);
    return 0;
}

STATIC int Test_CurrentCapBoundaries(void)
{
    static const Test_ConfigFieldType fields[] =
    {
        TEST_CFG_ALIGN_D, TEST_CFG_ALIGN_Q,
        TEST_CFG_FINAL_D, TEST_CFG_FINAL_Q
    };
    static const sint32 values[] = {101, -101, 101, -101};
    MotorControlStartup_ConfigType config;
    MotorControlStartup_Type object;
    uint32 fieldIndex;

    for (fieldIndex = 0U;
         fieldIndex < (uint32)(sizeof(fields) / sizeof(fields[0]));
         fieldIndex++)
    {
        config = Test_DefaultConfig();
        config.absoluteCurrentCapQ15 = 100;
        Test_SetConfigField(&config, fields[fieldIndex], values[fieldIndex]);
        TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_NOT_OK);
        TEST_CHECK(Test_ObjectIsZero(&object) == TRUE);
    }
    return 0;
}

STATIC int Test_OpenLoopAngleWrapBothDirections(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    uint32 cycle;

    config.openLoopRampCycles = 1U;
    config.startAngleQ16 = 65534U;
    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    for (cycle = 0U; cycle < config.alignCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((result.selectedSpeedAngleCounts == 4) &&
               (result.selectedAngleQ16 == 2U));

    config.finalOpenLoopSpeedAngleCounts = -4;
    config.startAngleQ16 = 2U;
    input.observer.speedAngleCounts = -4;
    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    for (cycle = 0U; cycle < config.alignCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((result.selectedSpeedAngleCounts == -4) &&
               (result.selectedAngleQ16 == 65534U));
    return 0;
}

STATIC int Test_BeginAndExactPhaseCycles(void)
{
    static const sint32 expectedD[] = {50, -1, -52, -103};
    static const sint32 expectedQ[] = {-50, 1, 52, 103};
    static const sint32 expectedSpeed[] = {1, 2, 3, 4};
    static const uint16 expectedAngle[] = {65531U, 65533U, 0U, 4U};
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object = {0};
    uint32 cycle;
    uint32 savedTotal;

    TEST_CHECK(MotorControlStartup_Begin(NULL_PTR, 1U) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 1U) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 0U) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 100000U) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 99988U) == E_NOT_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE) &&
               (object.totalCycle == 0U));
    TEST_CHECK(MotorControlStartup_Begin(&object, 99987U) == E_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_ALIGN) &&
               (object.totalCycle == 99987U) &&
               (object.phaseCycle == 0U));
    for (cycle = 0U; cycle < config.alignCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    for (cycle = 0U; cycle < config.openLoopRampCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    input.observer.varianceValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    input.observer.varianceValid = TRUE;
    for (cycle = 0U; cycle < config.qualificationRequiredCycles; cycle++)
    {
        input.observer.angleQ16 =
            (uint16)(object.selectedAngleQ16 +
                     (uint16)object.selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    for (cycle = 0U; cycle < config.handoverCycles; cycle++)
    {
        input.observer.angleQ16 =
            (uint16)(object.selectedAngleQ16 +
                     (uint16)object.selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    for (cycle = 0U; cycle < config.outputConfirmCycles; cycle++)
    {
        input.observer.angleQ16 =
            (uint16)(object.selectedAngleQ16 +
                     (uint16)object.selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE) &&
               (object.totalCycle == MOTOR_CONTROL_STARTUP_MAX_CYCLES) &&
               (result.readyForRun == TRUE));

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    savedTotal = object.totalCycle;
    TEST_CHECK(MotorControlStartup_Step(NULL_PTR, &input, &result) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Step(&object, NULL_PTR, &result) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(object.totalCycle == savedTotal);

    for (cycle = 0U; cycle < config.alignCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
        TEST_CHECK(result.phase == MOTOR_CONTROL_STARTUP_PHASE_ALIGN);
        TEST_CHECK((result.selectedAngleQ16 == 65530U) &&
                   (result.selectedSpeedAngleCounts == 0) &&
                   (result.dRefQ15 == 100) && (result.qRefQ15 == -100));
    }
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_OPEN_LOOP_RAMP);
    for (cycle = 0U; cycle < config.openLoopRampCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
        TEST_CHECK(result.phase == MOTOR_CONTROL_STARTUP_PHASE_OPEN_LOOP_RAMP);
        TEST_CHECK(result.dRefQ15 == expectedD[cycle]);
        TEST_CHECK(result.qRefQ15 == expectedQ[cycle]);
        TEST_CHECK(result.selectedSpeedAngleCounts == expectedSpeed[cycle]);
        TEST_CHECK(result.selectedAngleQ16 == expectedAngle[cycle]);
    }
    TEST_CHECK(object.phase ==
               MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY);
    TEST_CHECK(object.phaseCycle == 0U);
    TEST_CHECK(object.totalCycle == 16U);
    return 0;
}

STATIC int Test_StopFaultPrecedenceAndRearm(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    sint32 savedD;
    sint32 savedQ;
    uint32 savedTotal;
    uint32 invalidField;

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    input.stopRequested = TRUE;
    input.generationValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED) &&
               (result.failure == MOTOR_CONTROL_STARTUP_FAILURE_GUARD) &&
               (result.stopRequired == FALSE));
    TEST_CHECK(MotorControlStartup_RearmFromReady(&object) == E_NOT_OK);

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    input = Test_ValidInput();
    input.stopRequested = TRUE;
    input.safetyValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((result.failure == MOTOR_CONTROL_STARTUP_FAILURE_GUARD) &&
               (result.stopRequired == FALSE));

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    input = Test_ValidInput();
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    savedD = object.dRefQ15;
    savedQ = object.qRefQ15;
    savedTotal = object.totalCycle;
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_ABORTED) &&
               (result.phase == MOTOR_CONTROL_STARTUP_PHASE_ALIGN) &&
               (result.stopRequired == TRUE) &&
               (result.dRefQ15 == savedD) && (result.qRefQ15 == savedQ) &&
               (object.totalCycle == savedTotal));
    input = Test_ValidInput();
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((result.phase == MOTOR_CONTROL_STARTUP_PHASE_ABORTED) &&
               (result.stopRequired == TRUE) &&
               (object.totalCycle == savedTotal));
    TEST_CHECK(MotorControlStartup_RearmFromReady(&object) == E_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_IDLE) &&
               (object.totalCycle == 0U) && (object.dRefQ15 == 0));
    TEST_CHECK(MotorControlStartup_RearmFromReady(&object) == E_NOT_OK);

    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    TEST_CHECK(MotorControlStartup_RearmFromReady(&object) == E_NOT_OK);
    input.commandContinueValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.stopRequired == TRUE);

    for (invalidField = 0U; invalidField < 7U; invalidField++)
    {
        input = Test_ValidInput();
        TEST_CHECK(Test_ReachHandover(&object, &config, &input) == 0);
        input.stopRequested = TRUE;
        if (invalidField == 0U) { input.observer.numericValid = FALSE; }
        else if (invalidField == 1U) { input.observer.angleValid = FALSE; }
        else if (invalidField == 2U) { input.observer.speedValid = FALSE; }
        else if (invalidField == 3U) { input.observer.varianceValid = FALSE; }
        else if (invalidField == 4U) { input.observer.bemfValid = FALSE; }
        else if (invalidField == 5U) { input.observer.currentValid = FALSE; }
        else { input.observer.directionValid = FALSE; }
        TEST_CHECK(MotorControlStartup_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK((result.failure ==
                    MOTOR_CONTROL_STARTUP_FAILURE_OBSERVER_LOSS) &&
                   (result.stopRequired == FALSE));
    }

    input = Test_ValidInput();
    TEST_CHECK(Test_ReachOutputConfirm(&object, &config, &input) == 0);
    input.stopRequested = TRUE;
    input.observer.angleQ16 = (uint16)(object.selectedAngleQ16 + 100U);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((result.failure ==
                MOTOR_CONTROL_STARTUP_FAILURE_OBSERVER_LOSS) &&
               (result.stopRequired == FALSE));

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    input = Test_ValidInput();
    input.stopRequested = TRUE;
    input.observer.speedAngleCounts = 32768;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC);

    for (invalidField = 0U; invalidField < 11U; invalidField++)
    {
        TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
        TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
        input = Test_ValidInput();
        if (invalidField == 0U) { input.observer.numericValid = (boolean)2U; }
        else if (invalidField == 1U) { input.observer.angleValid = (boolean)2U; }
        else if (invalidField == 2U) { input.observer.speedValid = (boolean)2U; }
        else if (invalidField == 3U) { input.observer.varianceValid = (boolean)2U; }
        else if (invalidField == 4U) { input.observer.bemfValid = (boolean)2U; }
        else if (invalidField == 5U) { input.observer.currentValid = (boolean)2U; }
        else if (invalidField == 6U) { input.observer.directionValid = (boolean)2U; }
        else if (invalidField == 7U) { input.generationValid = (boolean)2U; }
        else if (invalidField == 8U) { input.safetyValid = (boolean)2U; }
        else if (invalidField == 9U) { input.commandContinueValid = (boolean)2U; }
        else { input.stopRequested = (boolean)2U; }
        TEST_CHECK(MotorControlStartup_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK(result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC);
    }

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    input = Test_ValidInput();
    input.safetyValid = (boolean)2U;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC);

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    object.totalCycle = MOTOR_CONTROL_STARTUP_MAX_CYCLES;
    input = Test_ValidInput();
    input.stopRequested = TRUE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(result.failure == MOTOR_CONTROL_STARTUP_FAILURE_START_TIMEOUT);
    TEST_CHECK(MotorControlStartup_RearmFromReady(NULL_PTR) == E_NOT_OK);
    return 0;
}

STATIC int Test_ControlledStopAllActivePhases(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input;
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    MotorControlStartup_PhaseType savedPhase;
    uint16 savedAngle;
    sint32 savedSpeed;
    sint32 savedD;
    sint32 savedQ;
    uint32 savedTotal;
    uint32 phaseIndex;
    uint32 cycle;

    for (phaseIndex = 0U; phaseIndex < 5U; phaseIndex++)
    {
        input = Test_ValidInput();
        if (phaseIndex == 0U)
        {
            TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
            TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
        }
        else if (phaseIndex == 1U)
        {
            TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
            TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
            for (cycle = 0U; cycle < config.alignCycles; cycle++)
            {
                TEST_CHECK(MotorControlStartup_Step(
                               &object, &input, &result) == E_OK);
            }
        }
        else if (phaseIndex == 2U)
        {
            TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
        }
        else if (phaseIndex == 3U)
        {
            TEST_CHECK(Test_ReachHandover(&object, &config, &input) == 0);
        }
        else
        {
            TEST_CHECK(Test_ReachOutputConfirm(&object, &config, &input) == 0);
        }

        savedPhase = object.phase;
        savedAngle = object.selectedAngleQ16;
        savedSpeed = object.selectedSpeedAngleCounts;
        savedD = object.dRefQ15;
        savedQ = object.qRefQ15;
        savedTotal = object.totalCycle;
        if ((phaseIndex & 1U) == 0U)
        {
            input.stopRequested = TRUE;
        }
        else
        {
            input.commandContinueValid = FALSE;
        }
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
        TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_ABORTED) &&
                   (result.phase == savedPhase) &&
                   (result.stopRequired == TRUE) &&
                   (result.selectedAngleQ16 == savedAngle) &&
                   (result.selectedSpeedAngleCounts == savedSpeed) &&
                   (result.dRefQ15 == savedD) && (result.qRefQ15 == savedQ) &&
                   (object.selectedAngleQ16 == savedAngle) &&
                   (object.selectedSpeedAngleCounts == savedSpeed) &&
                   (object.totalCycle == savedTotal));
    }
    return 0;
}

STATIC int Test_FailedTerminalRetention(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    uint32 savedTotal;

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    input.generationValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    savedTotal = object.totalCycle;
    input = Test_ValidInput();
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED) &&
               (object.failure == MOTOR_CONTROL_STARTUP_FAILURE_GUARD) &&
               (result.failure == MOTOR_CONTROL_STARTUP_FAILURE_GUARD) &&
               (object.dRefQ15 == 0) && (object.qRefQ15 == 0) &&
               (result.dRefQ15 == 0) && (result.qRefQ15 == 0) &&
               (object.totalCycle == savedTotal) &&
               (result.readyForRun == FALSE));
    return 0;
}

STATIC int Test_RuntimeIntegrity(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    uint32 corruption;

    for (corruption = 0U; corruption < 10U; corruption++)
    {
        TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
        TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
        if (corruption == 0U) { object.failure = (MotorControlStartup_FailureType)99; }
        else if (corruption == 1U) { object.dRefQ15 = config.absoluteCurrentCapQ15 + 1; }
        else if (corruption == 2U) { object.observerQualified = (boolean)2U; }
        else if (corruption == 3U) { object.qualificationConsecutiveCycles = config.qualificationRequiredCycles + 1U; }
        else if (corruption == 4U) { object.dRefRamp.duration++; }
        else if (corruption == 5U) { object.qRefRamp.direction = 2; }
        else if (corruption == 6U) { object.speedRamp.remainder = object.speedRamp.duration; }
        else if (corruption == 7U) { object.speedRamp.error = object.speedRamp.duration; }
        else if (corruption == 8U) { object.speedRamp.baseIncrement++; }
        else { object.dRefRamp.current = MOTOR_CONTROL_STARTUP_Q15_MAX; }
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
        TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED) &&
                   (result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC));
    }

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    object.config.alignDRefQ15 = (-2147483647 - 1);
    object.config.finalDRefQ15 = 2147483647;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED) &&
               (result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC) &&
               (result.dRefQ15 == 0) && (result.qRefQ15 == 0));
    return 0;
}

STATIC int Test_RuntimeCounterBoundaries(void)
{
    static const uint32 invalidAlignCycles[] =
    {
        2U, 3U, ((uint32)0xFFFFFFFFUL)
    };
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    uint32 caseIndex;
    uint32 cycle;
    uint32 savedPhaseCycle;
    uint32 savedTotal;

    for (caseIndex = 0U;
         caseIndex <
             (uint32)(sizeof(invalidAlignCycles) /
                      sizeof(invalidAlignCycles[0]));
         caseIndex++)
    {
        TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
        TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
        object.phaseCycle = invalidAlignCycles[caseIndex];
        savedPhaseCycle = object.phaseCycle;
        savedTotal = object.totalCycle;
        TEST_CHECK(MotorControlStartup_Step(
                       &object, &input, &result) == E_NOT_OK);
        TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED) &&
                   (result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC) &&
                   (object.phaseCycle == savedPhaseCycle) &&
                   (object.totalCycle == savedTotal) &&
                   (result.dRefQ15 == 0) && (result.qRefQ15 == 0));
    }

    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    for (cycle = 0U; cycle < config.alignCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    object.dRefRamp.elapsed = config.openLoopRampCycles;
    object.qRefRamp.elapsed = config.openLoopRampCycles;
    object.speedRamp.elapsed = config.openLoopRampCycles;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC) &&
               (object.phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED));

    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    object.phaseCycle = config.qualificationCycles;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((result.failure == MOTOR_CONTROL_STARTUP_FAILURE_NUMERIC) &&
               (object.phase == MOTOR_CONTROL_STARTUP_PHASE_FAILED));
    return 0;
}

STATIC int Test_QualificationMatrixAndFinalCycle(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input;
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    uint32 invalidFlag;

    config.qualificationCycles = 20U;
    for (invalidFlag = 0U; invalidFlag < 7U; invalidFlag++)
    {
        input = Test_ValidInput();
        TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
        input.observer.angleQ16 =
            (uint16)(object.selectedAngleQ16 +
                     (uint16)object.selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
        TEST_CHECK(object.qualificationConsecutiveCycles == 1U);
        if (invalidFlag == 0U) { input.observer.numericValid = FALSE; }
        else if (invalidFlag == 1U) { input.observer.angleValid = FALSE; }
        else if (invalidFlag == 2U) { input.observer.speedValid = FALSE; }
        else if (invalidFlag == 3U) { input.observer.varianceValid = FALSE; }
        else if (invalidFlag == 4U) { input.observer.bemfValid = FALSE; }
        else if (invalidFlag == 5U) { input.observer.currentValid = FALSE; }
        else { input.observer.directionValid = FALSE; }
        input.observer.angleQ16 =
            (uint16)(object.selectedAngleQ16 +
                     (uint16)object.selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
        TEST_CHECK(object.qualificationConsecutiveCycles == 0U);
    }

    config = Test_DefaultConfig();
    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.varianceValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    input.observer.varianceValid = TRUE;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER) &&
               (result.observerQualified == TRUE));

    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.varianceValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    input.observer.varianceValid = TRUE;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(result.failure ==
               MOTOR_CONTROL_STARTUP_FAILURE_QUALIFICATION_TIMEOUT);

    config = Test_DefaultConfig();
    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts +
                 (uint16)config.qualificationMaxAngleErrorQ16);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.qualificationConsecutiveCycles == 1U);

    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.speedAngleCounts =
        (sint32)config.minimumObservableSpeedAngleCounts - 1;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.qualificationConsecutiveCycles == 0U);

    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.speedAngleCounts =
        -config.finalOpenLoopSpeedAngleCounts;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.qualificationConsecutiveCycles == 0U);

    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts +
                 (uint16)(config.qualificationMaxAngleErrorQ16 + 1U));
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.qualificationConsecutiveCycles == 0U);

    config.qualificationMaxSpeedErrorAngleCounts = 1U;
    input = Test_ValidInput();
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.speedAngleCounts =
        config.finalOpenLoopSpeedAngleCounts + 2;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.qualificationConsecutiveCycles == 0U);

    config = Test_DefaultConfig();
    config.finalOpenLoopSpeedAngleCounts = -4;
    input = Test_ValidInput();
    input.observer.speedAngleCounts = -4;
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.qualificationConsecutiveCycles == 1U);

    config.qualificationMaxAngleErrorQ16 = 32767U;
    input = Test_ValidInput();
    input.observer.speedAngleCounts = -4;
    TEST_CHECK(Test_ReachQualification(&object, &config, &input) == 0);
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts + 32768U);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.qualificationConsecutiveCycles == 0U);
    return 0;
}

STATIC int Test_HandoverOutputAndObserverLoss(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    uint16 previousAngle;
    sint32 previousSpeed;

    TEST_CHECK(Test_ReachHandover(&object, &config, &input) == 0);
    previousAngle = object.selectedAngleQ16;
    previousSpeed = object.selectedSpeedAngleCounts;
    input.observer.speedAngleCounts = 10;
    input.observer.angleQ16 =
        (uint16)(previousAngle + (uint16)(previousSpeed + 2) + 20U);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(result.phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER);
    TEST_CHECK(object.selectedSpeedAngleCounts - previousSpeed == 2);
    TEST_CHECK(MotorControlStartup_CircularDelta(
                   object.selectedAngleQ16,
                   (uint16)(previousAngle +
                            (uint16)object.selectedSpeedAngleCounts)) == 8);

    input.observer.speedAngleCounts = object.selectedSpeedAngleCounts + 2;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)input.observer.speedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((object.phase ==
                MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
               (result.handoverComplete == TRUE));

    input.observer.speedAngleCounts = object.selectedSpeedAngleCounts;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((result.phase ==
                MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
               (result.readyForRun == FALSE));
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)object.selectedSpeedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((object.phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE) &&
               (result.phase ==
                MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
               (result.readyForRun == TRUE));
    previousAngle = object.selectedAngleQ16;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((result.phase == MOTOR_CONTROL_STARTUP_PHASE_COMPLETE) &&
               (result.readyForRun == TRUE) &&
               (object.selectedAngleQ16 == previousAngle));
    TEST_CHECK(MotorControlStartup_RearmFromReady(&object) == E_OK);

    config = Test_DefaultConfig();
    config.handoverCycles = 1U;
    config.handoverMaxAngleCorrectionQ16 = 1U;
    config.finalMaxAngleErrorQ16 = 0U;
    input = Test_ValidInput();
    TEST_CHECK(Test_ReachHandover(&object, &config, &input) == 0);
    input.observer.angleQ16 = (uint16)(object.selectedAngleQ16 + 100U);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK(result.failure ==
               MOTOR_CONTROL_STARTUP_FAILURE_HANDOVER_TIMEOUT);

    config = Test_DefaultConfig();
    input = Test_ValidInput();
    TEST_CHECK(Test_ReachOutputConfirm(&object, &config, &input) == 0);
    input.stopRequested = TRUE;
    input.observer.currentValid = FALSE;
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_NOT_OK);
    TEST_CHECK((result.failure ==
                MOTOR_CONTROL_STARTUP_FAILURE_OBSERVER_LOSS) &&
               (result.stopRequired == FALSE));
    return 0;
}

STATIC int Test_HandoverDirectionWrapAndFinalBounds(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_ResultType result;
    MotorControlStartup_Type object;
    uint16 predictedAngle;
    uint16 previousAngle;
    sint32 previousSpeed;

    config.handoverCycles = 1U;
    config.finalMaxAngleErrorQ16 = 5U;
    config.finalMaxSpeedErrorAngleCounts = 2U;
    TEST_CHECK(Test_ReachHandover(&object, &config, &input) == 0);
    previousAngle = object.selectedAngleQ16;
    previousSpeed = object.selectedSpeedAngleCounts;
    input.observer.speedAngleCounts = previousSpeed + 4;
    predictedAngle =
        (uint16)(previousAngle +
                 (uint16)(previousSpeed +
                          (sint32)config.handoverMaxSpeedSlewAngleCounts));
    input.observer.angleQ16 =
        (uint16)(predictedAngle +
                 (uint16)(config.handoverMaxAngleCorrectionQ16 +
                          config.finalMaxAngleErrorQ16));
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((object.phase ==
                MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM) &&
               (object.selectedSpeedAngleCounts == previousSpeed + 2) &&
               (MotorControlStartup_CircularDelta(
                    input.observer.angleQ16,
                    object.selectedAngleQ16) == 5) &&
               ((input.observer.speedAngleCounts -
                 object.selectedSpeedAngleCounts) == 2));

    config = Test_DefaultConfig();
    config.finalOpenLoopSpeedAngleCounts = -4;
    config.startAngleQ16 = 20U;
    input = Test_ValidInput();
    input.observer.speedAngleCounts = -4;
    TEST_CHECK(Test_ReachHandover(&object, &config, &input) == 0);
    previousAngle = object.selectedAngleQ16;
    previousSpeed = object.selectedSpeedAngleCounts;
    TEST_CHECK(previousAngle == 2U);
    input.observer.speedAngleCounts = previousSpeed - 6;
    predictedAngle =
        (uint16)(previousAngle +
                 (uint16)(previousSpeed -
                          (sint32)config.handoverMaxSpeedSlewAngleCounts));
    input.observer.angleQ16 = (uint16)(predictedAngle - 20U);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK((object.selectedSpeedAngleCounts == previousSpeed - 2) &&
               (MotorControlStartup_CircularDelta(
                    object.selectedAngleQ16, predictedAngle) == -8) &&
               (object.selectedAngleQ16 > 65000U));

    input.observer.speedAngleCounts = object.selectedSpeedAngleCounts - 2;
    input.observer.angleQ16 =
        (uint16)(object.selectedAngleQ16 +
                 (uint16)input.observer.speedAngleCounts);
    TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM);
    return 0;
}

STATIC int Test_DdaAndCircularProperties(void)
{
    static const sint32 endpoints[] =
        {-32768, -100, -1, 0, 1, 100, 32767};
    MotorControlStartup_RampType ramp;
    uint32 startIndex;
    uint32 targetIndex;
    uint32 duration;
    uint32 step;
    uint32 current;
    uint32 target;

    for (startIndex = 0U;
         startIndex < (uint32)(sizeof(endpoints) / sizeof(endpoints[0]));
         startIndex++)
    {
        for (targetIndex = 0U;
             targetIndex <
                 (uint32)(sizeof(endpoints) / sizeof(endpoints[0]));
             targetIndex++)
        {
            for (duration = 1U; duration <= 9U; duration++)
            {
                sint64 delta = (sint64)endpoints[targetIndex] -
                               (sint64)endpoints[startIndex];
                sint32 sign = (delta > 0) ? 1 : ((delta < 0) ? -1 : 0);
                uint64 magnitude = (uint64)((delta < 0) ? -delta : delta);

                MotorControlStartup_PrepareRamp(
                    &ramp, endpoints[startIndex], endpoints[targetIndex],
                    duration);
                for (step = 1U; step <= duration; step++)
                {
                    sint32 expected = endpoints[startIndex] +
                        (sign * (sint32)((magnitude * step) / duration));
                    TEST_CHECK(MotorControlStartup_AdvanceRamp(&ramp) ==
                               expected);
                }
                TEST_CHECK(ramp.current == endpoints[targetIndex]);
            }
        }
    }

    for (current = 0U; current <= 65535U; current += 257U)
    {
        for (target = 0U; target <= 65535U; target += 509U)
        {
            uint16 raw = (uint16)((uint16)target - (uint16)current);
            sint32 expected = (raw <= 32767U) ? (sint32)raw :
                              ((sint32)raw - 65536);
            TEST_CHECK(MotorControlStartup_CircularDelta(
                           (uint16)target, (uint16)current) == expected);
        }
    }
    TEST_CHECK(MotorControlStartup_CircularDelta(32768U, 0U) == -32768);
    TEST_CHECK(MotorControlStartup_CircularDelta(0U, 32768U) == -32768);
    return 0;
}

STATIC int Test_ValidateCycleIsNonMutatingAndStopIndependent(void)
{
    MotorControlStartup_ConfigType config = Test_DefaultConfig();
    MotorControlStartup_StepInputType input = Test_ValidInput();
    MotorControlStartup_Type object;
    MotorControlStartup_Type before;
    MotorControlStartup_ResultType result;
    sint32 currentQRef;
    uint32 cycle;

    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   NULL_PTR, &input, &currentQRef) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_Init(&object, &config) == E_OK);
    TEST_CHECK(MotorControlStartup_Begin(&object, 10U) == E_OK);
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, NULL_PTR, &currentQRef) == E_NOT_OK);
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, NULL_PTR) == E_NOT_OK);

    input.stopRequested = TRUE;
    input.commandContinueValid = FALSE;
    before = object;
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, &currentQRef) == E_OK);
    TEST_CHECK(currentQRef == object.qRefQ15);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    input.stopRequested = FALSE;
    input.commandContinueValid = TRUE;
    for (cycle = 0U; cycle < config.alignCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_OPEN_LOOP_RAMP);
    before = object;
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, &currentQRef) == E_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    for (cycle = 0U; cycle < config.openLoopRampCycles; cycle++)
    {
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_OBSERVER_QUALIFY);
    before = object;
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, &currentQRef) == E_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    input.observer.speedAngleCounts = config.finalOpenLoopSpeedAngleCounts;
    for (cycle = 0U; cycle < config.qualificationRequiredCycles; cycle++)
    {
        input.observer.angleQ16 =
            (uint16)(object.selectedAngleQ16 +
                     (uint16)object.selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_HANDOVER);
    input.observer.speedAngleCounts = object.selectedSpeedAngleCounts;
    input.observer.angleQ16 = object.selectedAngleQ16;
    before = object;
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, &currentQRef) == E_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    for (cycle = 0U; cycle < config.handoverCycles; cycle++)
    {
        input.observer.speedAngleCounts = object.selectedSpeedAngleCounts;
        input.observer.angleQ16 =
            (uint16)(object.selectedAngleQ16 +
                     (uint16)object.selectedSpeedAngleCounts);
        TEST_CHECK(MotorControlStartup_Step(&object, &input, &result) == E_OK);
    }
    TEST_CHECK(object.phase == MOTOR_CONTROL_STARTUP_PHASE_OUTPUT_CONFIRM);
    input.observer.speedAngleCounts = object.selectedSpeedAngleCounts;
    input.observer.angleQ16 = object.selectedAngleQ16;
    before = object;
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, &currentQRef) == E_OK);
    TEST_CHECK(memcmp(&object, &before, sizeof(object)) == 0);

    input.generationValid = FALSE;
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, &currentQRef) == E_NOT_OK);
    input.generationValid = TRUE;
    input.safetyValid = FALSE;
    TEST_CHECK(MotorControlStartup_ValidateCycle(
                   &object, &input, &currentQRef) == E_NOT_OK);
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_InitializationAndConfigMatrix();
    failures += Test_CurrentCapBoundaries();
    failures += Test_OpenLoopAngleWrapBothDirections();
    failures += Test_BeginAndExactPhaseCycles();
    failures += Test_StopFaultPrecedenceAndRearm();
    failures += Test_ControlledStopAllActivePhases();
    failures += Test_FailedTerminalRetention();
    failures += Test_RuntimeIntegrity();
    failures += Test_RuntimeCounterBoundaries();
    failures += Test_QualificationMatrixAndFinalCycle();
    failures += Test_HandoverOutputAndObserverLoss();
    failures += Test_HandoverDirectionWrapAndFinalBounds();
    failures += Test_DdaAndCircularProperties();
    failures += Test_ValidateCycleIsNonMutatingAndStopIndependent();

    if (failures == 0)
    {
        (void)printf("PASS: MotorControl Startup Step-8B1 host matrix\n");
        return 0;
    }

    (void)printf("FAIL: %d MotorControl Startup host test group(s)\n",
                 failures);
    return 1;
}
