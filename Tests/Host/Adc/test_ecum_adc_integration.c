#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "Adc.h"
#include "Mcu.h"
#include "MotorControlCdd.h"
#include "Os.h"
#include "SafetySupervisor.h"
#include "SerialCommunicationCdd.h"
#include "StateManager.h"
#include "Wdg.h"
#include "WdgM.h"

#include "../../../Software/Bsw/Services/EcuM/Src/EcuM.c"

typedef enum
{
    TEST_CALL_MCU_EARLY_CLOCK = 1,
    TEST_CALL_MOTOR_EARLY_SAFE,
    TEST_CALL_STATE_BOOT_SAFE,
    TEST_CALL_MCU_STARTUP_VALIDATION,
    TEST_CALL_MCU_CLOCK_BASELINE,
    TEST_CALL_WDG_START,
    TEST_CALL_SAFE_CHECK,
    TEST_CALL_MCU_ADC12_CLOCK,
    TEST_CALL_ADC_INIT,
    TEST_CALL_ADC_COMMON_CHECK,
    TEST_CALL_MOTOR_INIT,
    TEST_CALL_MOTOR_STATIC_CHECK,
    TEST_CALL_MCU_SERIAL_CLOCK,
    TEST_CALL_SERIAL_INIT,
    TEST_CALL_SERIAL_STATIC_CHECK,
    TEST_CALL_SAFETY_INIT,
    TEST_CALL_STATE_SELF_TEST,
    TEST_CALL_OS_INIT,
    TEST_CALL_OS_COUNTER,
    TEST_CALL_WDG_TRIGGER,
    TEST_CALL_EMERGENCY_OFF,
    TEST_CALL_OS_RUNNING,
    TEST_CALL_CLOCK_EVENTS,
    TEST_CALL_WDGM_PREPARE,
    TEST_CALL_WDGM_ACCEPT,
    TEST_CALL_WDGM_HAS_AUTHORITY,
    TEST_CALL_WDGM_MAIN,
    TEST_CALL_WDGM_GET_FAULT,
    TEST_CALL_START_OS
} Test_CallType;

#define TEST_MAX_CALLS 64U

static jmp_buf Test_FailureJump;
static boolean Test_JumpOnEmergencyOff;
static Test_CallType Test_Calls[TEST_MAX_CALLS];
static uint32 Test_CallCount;
static Test_CallType Test_FailingCall;
static Os_TickType Test_OsTick;
static boolean Test_OsRunning;
static uint32 Test_ClockEvents;
static boolean Test_OutputSafe;
static uint32 Test_WdgTriggerCount;
static boolean Test_WdgMActive;
static boolean Test_WdgMRuntimeHealthy;
static uint32 Test_WdgMFaultEvent;

#define TEST_CHECK(condition)                                             \
    do                                                                    \
    {                                                                     \
        if (!(condition))                                                 \
        {                                                                 \
            (void)printf("FAIL line %d: %s\n", __LINE__, #condition);     \
            return 1;                                                     \
        }                                                                 \
    } while (0)

static void Test_RecordCall(Test_CallType call)
{
    if (Test_CallCount < TEST_MAX_CALLS)
    {
        Test_Calls[Test_CallCount] = call;
        Test_CallCount++;
    }
}

static Std_ReturnType Test_ReturnForCall(Test_CallType call)
{
    Test_RecordCall(call);
    if (Test_FailingCall == call)
    {
        /* Model the selected checkpoint observing a loss of safe integrity. */
        Test_OutputSafe = FALSE;
        return E_NOT_OK;
    }
    return E_OK;
}

static void Test_ResetFixture(void)
{
    (void)memset(Test_Calls, 0, sizeof(Test_Calls));
    Test_CallCount = 0U;
    Test_FailingCall = (Test_CallType)0;
    Test_OsTick = 42U;
    Test_OsRunning = TRUE;
    Test_ClockEvents = MCU_CLOCK_FAULT_EVENT_NONE;
    Test_OutputSafe = TRUE;
    Test_WdgTriggerCount = 0U;
    Test_WdgMActive = FALSE;
    Test_WdgMRuntimeHealthy = TRUE;
    Test_WdgMFaultEvent = 0U;
    Test_JumpOnEmergencyOff = FALSE;

    EcuM_WdgRefreshAuthorityActive = FALSE;
    EcuM_LastBootCheckpointTick = 0U;
    (void)memset(&EcuM_StartupInfo, 0, sizeof(EcuM_StartupInfo));
}

Std_ReturnType Mcu_EarlyClockGateInit(void)
{
    return Test_ReturnForCall(TEST_CALL_MCU_EARLY_CLOCK);
}

Std_ReturnType MotorControlCdd_EarlySafeInit(void)
{
    return Test_ReturnForCall(TEST_CALL_MOTOR_EARLY_SAFE);
}

Std_ReturnType StateManager_InitBootSafe(void)
{
    return Test_ReturnForCall(TEST_CALL_STATE_BOOT_SAFE);
}

Std_ReturnType Mcu_StartupValidationInit(Mcu_StartupInfoType *StartupInfo)
{
    if (StartupInfo != NULL_PTR)
    {
        StartupInfo->rawDbgmcuIdcode = 0x20036468U;
        StartupInfo->bootIdentity = 0x1234A5A5U;
    }
    return Test_ReturnForCall(TEST_CALL_MCU_STARTUP_VALIDATION);
}

Std_ReturnType Mcu_ClockBaselineInit(void)
{
    return Test_ReturnForCall(TEST_CALL_MCU_CLOCK_BASELINE);
}

Std_ReturnType Wdg_Start(void)
{
    return Test_ReturnForCall(TEST_CALL_WDG_START);
}

Std_ReturnType MotorControlCdd_CheckEarlySafeState(void)
{
    Test_RecordCall(TEST_CALL_SAFE_CHECK);
    if ((Test_FailingCall == TEST_CALL_SAFE_CHECK) ||
        (Test_OutputSafe != TRUE))
    {
        return E_NOT_OK;
    }
    return E_OK;
}

Std_ReturnType Mcu_Adc12PeripheralClockInit(void)
{
    return Test_ReturnForCall(TEST_CALL_MCU_ADC12_CLOCK);
}

Std_ReturnType Adc_Init(void)
{
    return Test_ReturnForCall(TEST_CALL_ADC_INIT);
}

Std_ReturnType Adc_CheckCommonConfiguration(void)
{
    return Test_ReturnForCall(TEST_CALL_ADC_COMMON_CHECK);
}

Std_ReturnType MotorControlCdd_Init(void)
{
    return Test_ReturnForCall(TEST_CALL_MOTOR_INIT);
}

Std_ReturnType MotorControlCdd_CheckStaticConfiguration(void)
{
    Test_RecordCall(TEST_CALL_MOTOR_STATIC_CHECK);
    if ((Test_FailingCall == TEST_CALL_MOTOR_STATIC_CHECK) ||
        (Test_OutputSafe != TRUE))
    {
        return E_NOT_OK;
    }
    return E_OK;
}

Std_ReturnType Mcu_SerialPeripheralClockInit(void)
{
    return Test_ReturnForCall(TEST_CALL_MCU_SERIAL_CLOCK);
}

Std_ReturnType SerialCommunicationCdd_Init(uint32 BootIdentity)
{
    if (BootIdentity != 0x1234A5A5U)
    {
        return E_NOT_OK;
    }
    return Test_ReturnForCall(TEST_CALL_SERIAL_INIT);
}

Std_ReturnType SerialCommunicationCdd_CheckStaticConfiguration(void)
{
    return Test_ReturnForCall(TEST_CALL_SERIAL_STATIC_CHECK);
}

Std_ReturnType SafetySupervisor_Init(void)
{
    return Test_ReturnForCall(TEST_CALL_SAFETY_INIT);
}

Std_ReturnType StateManager_EnterSelfTest(void)
{
    return Test_ReturnForCall(TEST_CALL_STATE_SELF_TEST);
}

Std_ReturnType Os_Init(void)
{
    return Test_ReturnForCall(TEST_CALL_OS_INIT);
}

Os_TickType Os_GetCounterValue(void)
{
    Test_RecordCall(TEST_CALL_OS_COUNTER);
    return Test_OsTick;
}

boolean Os_IsRunning(void)
{
    Test_RecordCall(TEST_CALL_OS_RUNNING);
    return Test_OsRunning;
}

uint32 Mcu_GetClockFaultEvents(void)
{
    Test_RecordCall(TEST_CALL_CLOCK_EVENTS);
    return Test_ClockEvents;
}

Std_ReturnType WdgM_PrepareHandoff(
    uint32 CurrentTick,
    WdgM_HandoffTokenType *Token)
{
    (void)CurrentTick;
    if ((Token == NULL_PTR) ||
        (Test_ReturnForCall(TEST_CALL_WDGM_PREPARE) != E_OK))
    {
        return E_NOT_OK;
    }
    *Token = 0x12345678U;
    return E_OK;
}

Std_ReturnType WdgM_AcceptHandoff(WdgM_HandoffTokenType Token)
{
    if ((Token != 0x12345678U) ||
        (Test_ReturnForCall(TEST_CALL_WDGM_ACCEPT) != E_OK))
    {
        return E_NOT_OK;
    }
    Test_WdgMActive = TRUE;
    return E_OK;
}

boolean WdgM_HasRefreshAuthority(void)
{
    Test_RecordCall(TEST_CALL_WDGM_HAS_AUTHORITY);
    return Test_WdgMActive;
}

uint32 WdgM_GetFaultEvent(void)
{
    Test_RecordCall(TEST_CALL_WDGM_GET_FAULT);
    return Test_WdgMFaultEvent;
}

void WdgM_MainFunction(void)
{
    Test_RecordCall(TEST_CALL_WDGM_MAIN);
    if (Test_WdgMRuntimeHealthy == TRUE)
    {
        Wdg_Trigger();
    }
    else
    {
        MotorControlCdd_EmergencyOff();
        Test_WdgMFaultEvent =
            (1UL << 16U) | WDGM_EVENT_CAUSE_SAFETY_ALIVE;
        Test_WdgMActive = FALSE;
    }
}

void Wdg_Trigger(void)
{
    Test_RecordCall(TEST_CALL_WDG_TRIGGER);
    Test_WdgTriggerCount++;
}

void MotorControlCdd_EmergencyOff(void)
{
    Test_RecordCall(TEST_CALL_EMERGENCY_OFF);
    Test_OutputSafe = TRUE;
    if (Test_JumpOnEmergencyOff == TRUE)
    {
        longjmp(Test_FailureJump, 1);
    }
}

void StartOS(AppModeType Mode)
{
    (void)Mode;
    Test_RecordCall(TEST_CALL_START_OS);
}

static int Test_InitOrderAndSuccessfulCheckpoint(void)
{
    static const Test_CallType expectedCalls[] =
    {
        TEST_CALL_MCU_EARLY_CLOCK,
        TEST_CALL_MOTOR_EARLY_SAFE,
        TEST_CALL_STATE_BOOT_SAFE,
        TEST_CALL_MCU_STARTUP_VALIDATION,
        TEST_CALL_MCU_CLOCK_BASELINE,
        TEST_CALL_WDG_START,
        TEST_CALL_SAFE_CHECK,
        TEST_CALL_MCU_ADC12_CLOCK,
        TEST_CALL_ADC_INIT,
        TEST_CALL_ADC_COMMON_CHECK,
        TEST_CALL_MOTOR_INIT,
        TEST_CALL_MOTOR_STATIC_CHECK,
        TEST_CALL_MCU_SERIAL_CLOCK,
        TEST_CALL_SERIAL_INIT,
        TEST_CALL_SERIAL_STATIC_CHECK,
        TEST_CALL_SAFETY_INIT,
        TEST_CALL_STATE_SELF_TEST,
        TEST_CALL_OS_INIT,
        TEST_CALL_MOTOR_STATIC_CHECK,
        TEST_CALL_SERIAL_STATIC_CHECK,
        TEST_CALL_OS_COUNTER,
        TEST_CALL_WDG_TRIGGER
    };

    Test_ResetFixture();
    EcuM_Init();
    TEST_CHECK(Test_CallCount ==
               (sizeof(expectedCalls) / sizeof(expectedCalls[0])));
    TEST_CHECK(memcmp(Test_Calls, expectedCalls, sizeof(expectedCalls)) == 0);
    TEST_CHECK(EcuM_WdgRefreshAuthorityActive == TRUE);
    TEST_CHECK(EcuM_LastBootCheckpointTick == Test_OsTick);
    TEST_CHECK(Test_WdgTriggerCount == 1U);
    return 0;
}

static int Test_InitAdcFailuresRemainSafeAndUnrefreshed(void)
{
    static const Test_CallType failingCalls[] =
    {
        TEST_CALL_STATE_BOOT_SAFE,
        TEST_CALL_MCU_ADC12_CLOCK,
        TEST_CALL_ADC_INIT,
        TEST_CALL_ADC_COMMON_CHECK,
        TEST_CALL_MOTOR_INIT,
        TEST_CALL_MOTOR_STATIC_CHECK,
        TEST_CALL_MCU_SERIAL_CLOCK,
        TEST_CALL_SERIAL_INIT,
        TEST_CALL_SERIAL_STATIC_CHECK,
        TEST_CALL_SAFETY_INIT,
        TEST_CALL_STATE_SELF_TEST
    };
    uint32 index;

    for (index = 0U;
         index < (sizeof(failingCalls) / sizeof(failingCalls[0]));
         index++)
    {
        Test_ResetFixture();
        Test_FailingCall = failingCalls[index];
        Test_JumpOnEmergencyOff = TRUE;

        if (setjmp(Test_FailureJump) == 0)
        {
            EcuM_Init();
            TEST_CHECK(FALSE);
        }

        TEST_CHECK(Test_OutputSafe == TRUE);
        TEST_CHECK(EcuM_WdgRefreshAuthorityActive == FALSE);
        TEST_CHECK(Test_WdgTriggerCount == 0U);
        TEST_CHECK(Test_Calls[Test_CallCount - 1U] ==
                   TEST_CALL_EMERGENCY_OFF);

        {
            boolean failingCallObserved = FALSE;
            uint32 callIndex;
            for (callIndex = 0U; callIndex < Test_CallCount; callIndex++)
            {
                if (Test_Calls[callIndex] == failingCalls[index])
                {
                    failingCallObserved = TRUE;
                }
            }
            TEST_CHECK(failingCallObserved == TRUE);
        }

        if (failingCalls[index] == TEST_CALL_ADC_INIT)
        {
            uint32 callIndex;
            for (callIndex = 0U; callIndex < Test_CallCount; callIndex++)
            {
                TEST_CHECK(Test_Calls[callIndex] !=
                           TEST_CALL_ADC_COMMON_CHECK);
            }
        }
    }
    return 0;
}

static int Test_OneWayRuntimeHandoff(void)
{
    static const Test_CallType firstCheckpointCalls[] =
    {
        TEST_CALL_OS_COUNTER,
        TEST_CALL_OS_RUNNING,
        TEST_CALL_CLOCK_EVENTS,
        TEST_CALL_ADC_COMMON_CHECK,
        TEST_CALL_MOTOR_STATIC_CHECK,
        TEST_CALL_SERIAL_STATIC_CHECK,
        TEST_CALL_WDGM_PREPARE,
        TEST_CALL_WDG_TRIGGER,
        TEST_CALL_WDGM_ACCEPT
    };
    static const Test_CallType runtimeCalls[] =
    {
        TEST_CALL_OS_COUNTER,
        TEST_CALL_WDGM_HAS_AUTHORITY,
        TEST_CALL_WDGM_MAIN,
        TEST_CALL_WDG_TRIGGER,
        TEST_CALL_WDGM_HAS_AUTHORITY
    };

    Test_ResetFixture();
    EcuM_Init();
    Test_CallCount = 0U;
    Test_WdgTriggerCount = 0U;
    Test_OsTick += OS_BOOT_SUPERVISION_PERIOD_TICKS;
    EcuM_BootSupervisorMainFunction();
    TEST_CHECK(Test_CallCount ==
               (sizeof(firstCheckpointCalls) /
                sizeof(firstCheckpointCalls[0])));
    TEST_CHECK(memcmp(Test_Calls, firstCheckpointCalls,
                      sizeof(firstCheckpointCalls)) == 0);
    TEST_CHECK(EcuM_WdgRefreshAuthorityActive == FALSE);
    TEST_CHECK(Test_WdgMActive == TRUE);
    TEST_CHECK(Test_WdgTriggerCount == 1U);

    Test_CallCount = 0U;
    Test_WdgTriggerCount = 0U;
    Test_OsTick += OS_RUNTIME_SUPERVISION_PERIOD_TICKS;
    EcuM_BootSupervisorMainFunction();
    TEST_CHECK(Test_CallCount ==
               (sizeof(runtimeCalls) / sizeof(runtimeCalls[0])));
    TEST_CHECK(memcmp(Test_Calls, runtimeCalls, sizeof(runtimeCalls)) == 0);
    TEST_CHECK(Test_WdgTriggerCount == 1U);
    return 0;
}

static int Test_HandoffFailuresStopRefreshAuthority(void)
{
    static const Test_CallType handoffFailures[] =
    {
        TEST_CALL_WDGM_PREPARE,
        TEST_CALL_WDGM_ACCEPT
    };
    uint32 index;

    for (index = 0U;
         index < (sizeof(handoffFailures) / sizeof(handoffFailures[0]));
         index++)
    {
        Test_ResetFixture();
        EcuM_Init();
        Test_CallCount = 0U;
        Test_WdgTriggerCount = 0U;
        Test_OsTick += OS_BOOT_SUPERVISION_PERIOD_TICKS;
        Test_FailingCall = handoffFailures[index];
        Test_JumpOnEmergencyOff = TRUE;

        if (setjmp(Test_FailureJump) == 0)
        {
            EcuM_BootSupervisorMainFunction();
            TEST_CHECK(FALSE);
        }

        TEST_CHECK(EcuM_WdgRefreshAuthorityActive == FALSE);
        TEST_CHECK(Test_Calls[Test_CallCount - 1U] ==
                   TEST_CALL_EMERGENCY_OFF);
        if (handoffFailures[index] == TEST_CALL_WDGM_PREPARE)
        {
            TEST_CHECK(Test_WdgTriggerCount == 0U);
            TEST_CHECK(Test_WdgMActive == FALSE);
        }
        else
        {
            TEST_CHECK(Test_WdgTriggerCount == 1U);
            TEST_CHECK(Test_WdgMActive == FALSE);
        }
    }

    Test_ResetFixture();
    EcuM_Init();
    Test_OsTick += OS_BOOT_SUPERVISION_PERIOD_TICKS;
    EcuM_BootSupervisorMainFunction();
    TEST_CHECK(Test_WdgMActive == TRUE);
    Test_CallCount = 0U;
    Test_WdgTriggerCount = 0U;
    Test_WdgMRuntimeHealthy = FALSE;
    Test_OsTick += OS_RUNTIME_SUPERVISION_PERIOD_TICKS;
    EcuM_BootSupervisorMainFunction();
    TEST_CHECK(Test_WdgTriggerCount == 0U);
    TEST_CHECK(EcuM_WdgRefreshAuthorityActive == FALSE);
    TEST_CHECK(Test_WdgMActive == FALSE);
    TEST_CHECK((Test_WdgMFaultEvent & 0xFFFFU) ==
               WDGM_EVENT_CAUSE_SAFETY_ALIVE);
    {
        uint32 emergencyOffCalls = 0U;
        uint32 callIndex;

        for (callIndex = 0U; callIndex < Test_CallCount; callIndex++)
        {
            if (Test_Calls[callIndex] == TEST_CALL_EMERGENCY_OFF)
            {
                emergencyOffCalls++;
            }
        }
        TEST_CHECK(emergencyOffCalls == 1U);
    }
    TEST_CHECK(Test_Calls[Test_CallCount - 1U] == TEST_CALL_WDGM_GET_FAULT);

    Test_CallCount = 0U;
    Test_OsTick += OS_RUNTIME_SUPERVISION_PERIOD_TICKS;
    EcuM_BootSupervisorMainFunction();
    TEST_CHECK(Test_CallCount == 3U);
    TEST_CHECK(Test_Calls[0] == TEST_CALL_OS_COUNTER);
    TEST_CHECK(Test_Calls[1] == TEST_CALL_WDGM_HAS_AUTHORITY);
    TEST_CHECK(Test_Calls[2] == TEST_CALL_WDGM_GET_FAULT);
    return 0;
}

static int Test_SupervisorCommonCorruptionStopsRefresh(void)
{
    Test_ResetFixture();
    EcuM_Init();
    TEST_CHECK(EcuM_WdgRefreshAuthorityActive == TRUE);

    Test_CallCount = 0U;
    Test_WdgTriggerCount = 0U;
    Test_OsTick += OS_BOOT_SUPERVISION_PERIOD_TICKS;
    Test_FailingCall = TEST_CALL_ADC_COMMON_CHECK;
    Test_JumpOnEmergencyOff = TRUE;

    if (setjmp(Test_FailureJump) == 0)
    {
        EcuM_BootSupervisorMainFunction();
        TEST_CHECK(FALSE);
    }

    TEST_CHECK(Test_OutputSafe == TRUE);
    TEST_CHECK(EcuM_WdgRefreshAuthorityActive == FALSE);
    TEST_CHECK(Test_WdgTriggerCount == 0U);
    TEST_CHECK(Test_Calls[Test_CallCount - 1U] == TEST_CALL_EMERGENCY_OFF);
    {
        boolean commonCheckObserved = FALSE;
        uint32 callIndex;
        for (callIndex = 0U; callIndex < Test_CallCount; callIndex++)
        {
            if (Test_Calls[callIndex] == TEST_CALL_ADC_COMMON_CHECK)
            {
                commonCheckObserved = TRUE;
            }
        }
        TEST_CHECK(commonCheckObserved == TRUE);
    }
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_InitOrderAndSuccessfulCheckpoint();
    failures += Test_InitAdcFailuresRemainSafeAndUnrefreshed();
    failures += Test_SupervisorCommonCorruptionStopsRefresh();
    failures += Test_OneWayRuntimeHandoff();
    failures += Test_HandoffFailuresStopRefreshAuthority();

    if (failures == 0)
    {
        (void)printf("PASS: EcuM/ADC exact-source integration matrix\n");
        return 0;
    }

    (void)printf("FAIL: %d EcuM/ADC integration group(s)\n", failures);
    return 1;
}
