#include <stdio.h>
#include <string.h>

#include "Os.h"
#include "Os_Cfg.h"
#include "stm32g431xx.h"

SysTick_Type OsHost_SysTick;
SCB_Type OsHost_Scb;
uint32_t SystemCoreClock;

static uint32_t Test_PriorityGrouping;
static uint32_t Test_Priorities[3];
static uint32_t Test_BasePriority;

static uint32_t Test_IrqIndex(IRQn_Type irq)
{
    if (irq == SVCall_IRQn)
    {
        return 0U;
    }
    if (irq == SysTick_IRQn)
    {
        return 1U;
    }
    return 2U;
}

void OsHost_NvicSetPriorityGrouping(uint32_t grouping)
{
    Test_PriorityGrouping = grouping;
}

uint32_t OsHost_NvicGetPriorityGrouping(void)
{
    return Test_PriorityGrouping;
}

void OsHost_NvicSetPriority(IRQn_Type irq, uint32_t priority)
{
    Test_Priorities[Test_IrqIndex(irq)] = priority;
}

uint32_t OsHost_NvicGetPriority(IRQn_Type irq)
{
    return Test_Priorities[Test_IrqIndex(irq)];
}

uint32_t OsHost_GetBasePri(void)
{
    return Test_BasePriority;
}

void OsHost_SetBasePri(uint32_t value)
{
    Test_BasePriority = value;
}

void OsHost_SetBasePriMax(uint32_t value)
{
    if ((Test_BasePriority == 0U) || (value < Test_BasePriority))
    {
        Test_BasePriority = value;
    }
}

#include "../../../Software/Bsw/Services/Os/Src/Os.c"

void Os_PortStartFirstTask(void)
{
}

void Os_CfgBootSupervisorTask(void)
{
}

void Os_CfgSafetyTask(void)
{
}

void Os_CfgStateTask(void)
{
}

#define TEST_CHECK(condition)                                             \
    do                                                                    \
    {                                                                     \
        if (!(condition))                                                 \
        {                                                                 \
            (void)printf("FAIL line %d: %s\n", __LINE__, #condition);     \
            return 1;                                                     \
        }                                                                 \
    } while (0)

static void Test_ResetFixture(void)
{
    (void)memset(&OsHost_SysTick, 0, sizeof(OsHost_SysTick));
    (void)memset(&OsHost_Scb, 0, sizeof(OsHost_Scb));
    (void)memset(Test_Priorities, 0, sizeof(Test_Priorities));
    (void)memset((void *)Os_TaskTable, 0, sizeof(Os_TaskTable));
    SystemCoreClock = OS_EXPECTED_HCLK_HZ;
    Test_PriorityGrouping = UINT32_MAX;
    Test_BasePriority = 0U;
    Os_Counter = 0U;
    Os_BootPeriodCounter = 0U;
    Os_CurrentTask = OS_INVALID_TASK_INDEX;
    Os_Initialized = FALSE;
    Os_Running = FALSE;
}

static void Test_MakeAllExtendedTasksWaiting(void)
{
    Os_TaskTable[OS_BOOT_TASK_INDEX].state = OS_TASK_WAITING;
    Os_TaskTable[OS_BOOT_TASK_INDEX].waitMask =
        OS_BOOT_SUPERVISION_EVENT;
    Os_TaskTable[OS_BOOT_TASK_INDEX].events = 0U;
    Os_TaskTable[OS_STATE_TASK_INDEX].state = OS_TASK_WAITING;
    Os_TaskTable[OS_STATE_TASK_INDEX].waitMask = OS_STATE_TASK_EVENT;
    Os_TaskTable[OS_STATE_TASK_INDEX].events = 0U;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].state = OS_TASK_WAITING;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].waitMask = OS_SAFETY_TASK_EVENT;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].events = 0U;
    Os_TaskTable[OS_IDLE_TASK_INDEX].state = OS_TASK_RUNNING;
    Os_CurrentTask = OS_IDLE_TASK_INDEX;
}

static int Test_StaticTaskConfiguration(void)
{
    Test_ResetFixture();
    TEST_CHECK(Os_Init() == E_OK);
    TEST_CHECK(Os_Init() == E_NOT_OK);
    TEST_CHECK(Os_TaskTable[OS_IDLE_TASK_INDEX].priority == 0U);
    TEST_CHECK(Os_TaskTable[OS_BOOT_TASK_INDEX].priority == 1U);
    TEST_CHECK(Os_TaskTable[OS_STATE_TASK_INDEX].priority == 2U);
    TEST_CHECK(Os_TaskTable[OS_SAFETY_TASK_INDEX].priority == 3U);
    TEST_CHECK(Os_TaskTable[OS_BOOT_TASK_INDEX].entry ==
               Os_CfgBootSupervisorTask);
    TEST_CHECK(Os_TaskTable[OS_STATE_TASK_INDEX].entry == Os_CfgStateTask);
    TEST_CHECK(Os_TaskTable[OS_SAFETY_TASK_INDEX].entry == Os_CfgSafetyTask);
    TEST_CHECK(Os_IsTaskStackValid(OS_IDLE_TASK_INDEX) == TRUE);
    TEST_CHECK(Os_IsTaskStackValid(OS_BOOT_TASK_INDEX) == TRUE);
    TEST_CHECK(Os_IsTaskStackValid(OS_STATE_TASK_INDEX) == TRUE);
    TEST_CHECK(Os_IsTaskStackValid(OS_SAFETY_TASK_INDEX) == TRUE);
    TEST_CHECK(SysTick->CTRL == 0U);
    TEST_CHECK(SysTick->LOAD == OS_SYSTICK_RELOAD);
    TEST_CHECK(Test_Priorities[Test_IrqIndex(SVCall_IRQn)] ==
               OS_SVC_PRIORITY);
    TEST_CHECK(Test_Priorities[Test_IrqIndex(SysTick_IRQn)] ==
               OS_SYSTICK_PRIORITY);
    TEST_CHECK(Test_Priorities[Test_IrqIndex(PendSV_IRQn)] ==
               OS_PENDSV_PRIORITY);
    return 0;
}

static int Test_OneMillisecondEventsAndPriorityOrder(void)
{
    uint32 tick;

    Test_ResetFixture();
    TEST_CHECK(Os_Init() == E_OK);
    Os_Running = TRUE;
    Test_MakeAllExtendedTasksWaiting();

    SysTick_Handler();
    TEST_CHECK(Os_Counter == 1U);
    TEST_CHECK(Os_TaskTable[OS_SAFETY_TASK_INDEX].state == OS_TASK_READY);
    TEST_CHECK(Os_TaskTable[OS_STATE_TASK_INDEX].state == OS_TASK_READY);
    TEST_CHECK(Os_TaskTable[OS_BOOT_TASK_INDEX].state == OS_TASK_WAITING);
    (void)Os_PortSelectNextStackPointer();
    TEST_CHECK(Os_CurrentTask == OS_SAFETY_TASK_INDEX);
    Os_TaskTable[OS_SAFETY_TASK_INDEX].state = OS_TASK_WAITING;
    (void)Os_PortSelectNextStackPointer();
    TEST_CHECK(Os_CurrentTask == OS_STATE_TASK_INDEX);

    for (tick = 2U; tick <= 10U; tick++)
    {
        Test_MakeAllExtendedTasksWaiting();
        SysTick_Handler();
    }
    TEST_CHECK(Os_Counter == 10U);
    TEST_CHECK(Os_TaskTable[OS_SAFETY_TASK_INDEX].state == OS_TASK_READY);
    TEST_CHECK(Os_TaskTable[OS_STATE_TASK_INDEX].state == OS_TASK_READY);
    TEST_CHECK(Os_TaskTable[OS_BOOT_TASK_INDEX].state == OS_TASK_READY);
    TEST_CHECK((Os_TaskTable[OS_BOOT_TASK_INDEX].events &
                OS_BOOT_SUPERVISION_EVENT) != 0U);
    (void)Os_PortSelectNextStackPointer();
    TEST_CHECK(Os_CurrentTask == OS_SAFETY_TASK_INDEX);
    Os_TaskTable[OS_SAFETY_TASK_INDEX].state = OS_TASK_WAITING;
    (void)Os_PortSelectNextStackPointer();
    TEST_CHECK(Os_CurrentTask == OS_STATE_TASK_INDEX);
    Os_TaskTable[OS_STATE_TASK_INDEX].state = OS_TASK_WAITING;
    (void)Os_PortSelectNextStackPointer();
    TEST_CHECK(Os_CurrentTask == OS_BOOT_TASK_INDEX);
    return 0;
}

static int Test_EventApiRejectsInvalidUse(void)
{
    Test_ResetFixture();
    TEST_CHECK(Os_Init() == E_OK);
    TEST_CHECK(WaitEvent(OS_SAFETY_TASK_EVENT) == E_OS_STATE);
    Os_Running = TRUE;
    Os_CurrentTask = OS_IDLE_TASK_INDEX;
    Os_TaskTable[OS_IDLE_TASK_INDEX].state = OS_TASK_RUNNING;
    TEST_CHECK(WaitEvent(0U) == E_OS_VALUE);
    TEST_CHECK(ClearEvent(0U) == E_OS_VALUE);
    TEST_CHECK(WaitEvent(OS_SAFETY_TASK_EVENT) == E_OS_ACCESS);
    TEST_CHECK(ClearEvent(OS_SAFETY_TASK_EVENT) == E_OS_ACCESS);
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_StaticTaskConfiguration();
    failures += Test_OneMillisecondEventsAndPriorityOrder();
    failures += Test_EventApiRejectsInvalidUse();

    if (failures == 0)
    {
        (void)printf("PASS: Step-7A OS exact-source host matrix\n");
        return 0;
    }

    (void)printf("FAIL: %d OS host group(s)\n", failures);
    return 1;
}
