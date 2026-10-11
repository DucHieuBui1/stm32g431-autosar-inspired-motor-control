#include <stdio.h>

#include "EcuM.h"
#include "Os_Cfg.h"
#include "SafetySupervisor.h"
#include "StateManager.h"

static unsigned int Test_Failures;
static unsigned int Test_WaitCalls;
static unsigned int Test_ClearCalls;
static EventMaskType Test_LastWaitMask;
static EventMaskType Test_LastClearMask;
static unsigned int Test_EcuMCalls;
static unsigned int Test_SafetyCalls;
static unsigned int Test_StateCalls;

#define TEST_CHECK(condition)                                                   \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            (void)printf("FAIL:%u: %s\n", (unsigned int)__LINE__, #condition); \
            Test_Failures++;                                                    \
        }                                                                       \
    } while (0)

StatusType WaitEvent(EventMaskType Mask)
{
    Test_WaitCalls++;
    Test_LastWaitMask = Mask;

    if (Test_WaitCalls == 1U)
    {
        return E_OS_OK;
    }

    return E_OS_STATE;
}

StatusType ClearEvent(EventMaskType Mask)
{
    Test_ClearCalls++;
    Test_LastClearMask = Mask;
    return E_OS_OK;
}

void EcuM_BootSupervisorMainFunction(void)
{
    Test_EcuMCalls++;
}

void SafetySupervisor_MainFunction(void)
{
    Test_SafetyCalls++;
}

void StateManager_MainFunction(void)
{
    Test_StateCalls++;
}

#include "../../../Software/Bsw/Services/Os/Src/Os_Cfg.c"

static void Test_Reset(void)
{
    Test_WaitCalls = 0U;
    Test_ClearCalls = 0U;
    Test_LastWaitMask = 0U;
    Test_LastClearMask = 0U;
    Test_EcuMCalls = 0U;
    Test_SafetyCalls = 0U;
    Test_StateCalls = 0U;
}

static void Test_BootSupervisorTaskWiring(void)
{
    Test_Reset();
    Os_CfgBootSupervisorTask();

    TEST_CHECK(Test_WaitCalls == 2U);
    TEST_CHECK(Test_ClearCalls == 1U);
    TEST_CHECK(Test_LastWaitMask == OS_BOOT_SUPERVISION_EVENT);
    TEST_CHECK(Test_LastClearMask == OS_BOOT_SUPERVISION_EVENT);
    TEST_CHECK(Test_EcuMCalls == 1U);
    TEST_CHECK(Test_SafetyCalls == 0U);
    TEST_CHECK(Test_StateCalls == 0U);
}

static void Test_SafetyTaskWiring(void)
{
    Test_Reset();
    Os_CfgSafetyTask();

    TEST_CHECK(Test_WaitCalls == 2U);
    TEST_CHECK(Test_ClearCalls == 1U);
    TEST_CHECK(Test_LastWaitMask == OS_SAFETY_TASK_EVENT);
    TEST_CHECK(Test_LastClearMask == OS_SAFETY_TASK_EVENT);
    TEST_CHECK(Test_EcuMCalls == 0U);
    TEST_CHECK(Test_SafetyCalls == 1U);
    TEST_CHECK(Test_StateCalls == 0U);
}

static void Test_StateTaskWiring(void)
{
    Test_Reset();
    Os_CfgStateTask();

    TEST_CHECK(Test_WaitCalls == 2U);
    TEST_CHECK(Test_ClearCalls == 1U);
    TEST_CHECK(Test_LastWaitMask == OS_STATE_TASK_EVENT);
    TEST_CHECK(Test_LastClearMask == OS_STATE_TASK_EVENT);
    TEST_CHECK(Test_EcuMCalls == 0U);
    TEST_CHECK(Test_SafetyCalls == 0U);
    TEST_CHECK(Test_StateCalls == 1U);
}

int main(void)
{
    Test_BootSupervisorTaskWiring();
    Test_SafetyTaskWiring();
    Test_StateTaskWiring();

    if (Test_Failures == 0U)
    {
        (void)printf("PASS: Step-7A OS configuration exact-source host matrix\n");
        return 0;
    }

    return 1;
}
