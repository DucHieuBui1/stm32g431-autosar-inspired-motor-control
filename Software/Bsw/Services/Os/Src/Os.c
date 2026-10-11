#include "Os.h"

#include <stdint.h>

#include "Os_Cfg.h"
#include "stm32g431xx.h"

#if (__NVIC_PRIO_BITS != 4U)
#error "The reviewed STM32G431 OS port requires four implemented NVIC bits."
#endif

#define OS_TASK_COUNT                         4U
#define OS_IDLE_TASK_INDEX                    0U
#define OS_BOOT_TASK_INDEX                    1U
#define OS_STATE_TASK_INDEX                   2U
#define OS_SAFETY_TASK_INDEX                  3U
#define OS_INVALID_TASK_INDEX                 0xFFU

#define OS_IDLE_TASK_PRIORITY                 0U
#define OS_BOOT_TASK_PRIORITY                 1U
#define OS_STATE_TASK_PRIORITY                2U
#define OS_SAFETY_TASK_PRIORITY               3U

#define OS_TASK_STACK_WORDS                   128U
#define OS_STACK_GUARD_VALUE                  0xC0DEF00DU
#define OS_STACK_FILL_VALUE                   0xA5A5A5A5U
#define OS_INITIAL_XPSR                       0x01000000U

#define OS_EXPECTED_HCLK_HZ                   170000000U
#define OS_SYSTICK_RELOAD                     169999U

#define OS_SVC_PRIORITY                       7U
#define OS_SYSTICK_PRIORITY                   8U
#define OS_PENDSV_PRIORITY                    15U
#define OS_KERNEL_BASEPRI                     \
    (OS_SYSTICK_PRIORITY << (8U - __NVIC_PRIO_BITS))

#define OS_SYSTICK_CONTROL_MASK               \
    (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | \
     SysTick_CTRL_ENABLE_Msk)

#define OS_SYSTICK_CONTROL_RUNNING            OS_SYSTICK_CONTROL_MASK

typedef uint8 Os_TaskIndexType;
typedef void (*Os_TaskEntryType)(void);

typedef enum
{
    OS_TASK_SUSPENDED = 0,
    OS_TASK_READY,
    OS_TASK_RUNNING,
    OS_TASK_WAITING
} Os_TaskStateType;

typedef struct
{
    uint32 *stackPointer;
    uint32 *stackBase;
    uint32 stackWords;
    Os_TaskEntryType entry;
    EventMaskType events;
    EventMaskType waitMask;
    uint8 priority;
    boolean extendedTask;
    Os_TaskStateType state;
} Os_TaskControlBlockType;

_Alignas(8) STATIC uint32 Os_IdleTaskStack[OS_TASK_STACK_WORDS];
_Alignas(8) STATIC uint32 Os_BootTaskStack[OS_TASK_STACK_WORDS];
_Alignas(8) STATIC uint32 Os_StateTaskStack[OS_TASK_STACK_WORDS];
_Alignas(8) STATIC uint32 Os_SafetyTaskStack[OS_TASK_STACK_WORDS];

/* Shared by task context and SysTick/PendSV context. */
STATIC volatile Os_TaskControlBlockType Os_TaskTable[OS_TASK_COUNT];
STATIC volatile Os_TickType Os_Counter;
STATIC volatile Os_TickType Os_BootPeriodCounter;
STATIC volatile Os_TaskIndexType Os_CurrentTask = OS_INVALID_TASK_INDEX;
STATIC volatile boolean Os_Initialized = FALSE;
STATIC volatile boolean Os_Running = FALSE;

extern void Os_PortStartFirstTask(void);
void Os_PortSaveCurrentStackPointer(uint32 *stackPointer);
uint32 *Os_PortSelectNextStackPointer(void);
uint32 *Os_PortGetCurrentStackPointer(void);

STATIC void Os_FatalError(void)
{
    SysTick->CTRL = 0U;

    for (;;)
    {
        /* A running IWDG resets the safe node; no refresh is permitted here. */
    }
}

STATIC void Os_TaskReturned(void)
{
    Os_FatalError();
}

STATIC void Os_IdleTask(void)
{
    for (;;)
    {
        __WFI();
    }
}

STATIC uint32 *Os_InitializeTaskStack(
    uint32 *stackBase,
    uint32 stackWords,
    Os_TaskEntryType entry)
{
    uint32 index;
    uint32 *stackPointer;
    uint32 entryAddress = (uint32)(uintptr_t)entry;
    uint32 returnAddress = (uint32)(uintptr_t)Os_TaskReturned;

    for (index = 0U; index < stackWords; index++)
    {
        stackBase[index] = OS_STACK_FILL_VALUE;
    }

    stackBase[0] = OS_STACK_GUARD_VALUE;
    stackPointer = &stackBase[stackWords];

    /* Cortex-M hardware exception frame, restored on exception return. */
    *(--stackPointer) = OS_INITIAL_XPSR;
    *(--stackPointer) = entryAddress | 1U;
    *(--stackPointer) = returnAddress | 1U;
    *(--stackPointer) = 0U; /* R12 */
    *(--stackPointer) = 0U; /* R3 */
    *(--stackPointer) = 0U; /* R2 */
    *(--stackPointer) = 0U; /* R1 */
    *(--stackPointer) = 0U; /* R0 */

    /* Software-saved frame restored by SVC/PendSV. */
    *(--stackPointer) = 0U; /* R11 */
    *(--stackPointer) = 0U; /* R10 */
    *(--stackPointer) = 0U; /* R9 */
    *(--stackPointer) = 0U; /* R8 */
    *(--stackPointer) = 0U; /* R7 */
    *(--stackPointer) = 0U; /* R6 */
    *(--stackPointer) = 0U; /* R5 */
    *(--stackPointer) = 0U; /* R4 */

    return stackPointer;
}

STATIC boolean Os_IsTaskStackValid(Os_TaskIndexType taskIndex)
{
    if ((taskIndex < OS_TASK_COUNT) &&
        (Os_TaskTable[taskIndex].stackBase[0] == OS_STACK_GUARD_VALUE))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC uint32 Os_EnterCritical(void)
{
    uint32 previousBasePriority = __get_BASEPRI();

    __set_BASEPRI_MAX(OS_KERNEL_BASEPRI);
    __DSB();
    __ISB();

    return previousBasePriority;
}

STATIC void Os_ExitCritical(uint32 previousBasePriority)
{
    __set_BASEPRI(previousBasePriority);
    __DSB();
    __ISB();
}

STATIC void Os_RequestDispatch(void)
{
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}

STATIC void Os_SetEventFromSysTick(
    Os_TaskIndexType taskIndex,
    EventMaskType event)
{
    volatile Os_TaskControlBlockType *task = &Os_TaskTable[taskIndex];

    task->events |= event;

    if ((task->state == OS_TASK_WAITING) &&
        ((task->events & task->waitMask) != 0U))
    {
        task->state = OS_TASK_READY;
    }
}

Std_ReturnType Os_Init(void)
{
    uint32 sysTickControlState;

    if ((Os_Initialized == TRUE) ||
        (SystemCoreClock != OS_EXPECTED_HCLK_HZ))
    {
        return E_NOT_OK;
    }

    SysTick->CTRL = 0U;
    SysTick->LOAD = OS_SYSTICK_RELOAD;
    SysTick->VAL = 0U;
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;
    SCB->CCR |= SCB_CCR_STKALIGN_Msk;

    NVIC_SetPriorityGrouping(0U);
    NVIC_SetPriority(SVCall_IRQn, OS_SVC_PRIORITY);
    NVIC_SetPriority(SysTick_IRQn, OS_SYSTICK_PRIORITY);
    NVIC_SetPriority(PendSV_IRQn, OS_PENDSV_PRIORITY);

    Os_TaskTable[OS_IDLE_TASK_INDEX].stackBase = Os_IdleTaskStack;
    Os_TaskTable[OS_IDLE_TASK_INDEX].stackWords = OS_TASK_STACK_WORDS;
    Os_TaskTable[OS_IDLE_TASK_INDEX].entry = Os_IdleTask;
    Os_TaskTable[OS_IDLE_TASK_INDEX].priority = OS_IDLE_TASK_PRIORITY;
    Os_TaskTable[OS_IDLE_TASK_INDEX].extendedTask = FALSE;
    Os_TaskTable[OS_IDLE_TASK_INDEX].events = 0U;
    Os_TaskTable[OS_IDLE_TASK_INDEX].waitMask = 0U;
    Os_TaskTable[OS_IDLE_TASK_INDEX].state = OS_TASK_READY;
    Os_TaskTable[OS_IDLE_TASK_INDEX].stackPointer =
        Os_InitializeTaskStack(Os_IdleTaskStack, OS_TASK_STACK_WORDS,
                               Os_IdleTask);

    Os_TaskTable[OS_BOOT_TASK_INDEX].stackBase = Os_BootTaskStack;
    Os_TaskTable[OS_BOOT_TASK_INDEX].stackWords = OS_TASK_STACK_WORDS;
    Os_TaskTable[OS_BOOT_TASK_INDEX].entry = Os_CfgBootSupervisorTask;
    Os_TaskTable[OS_BOOT_TASK_INDEX].priority = OS_BOOT_TASK_PRIORITY;
    Os_TaskTable[OS_BOOT_TASK_INDEX].extendedTask = TRUE;
    Os_TaskTable[OS_BOOT_TASK_INDEX].events = 0U;
    Os_TaskTable[OS_BOOT_TASK_INDEX].waitMask = 0U;
    Os_TaskTable[OS_BOOT_TASK_INDEX].state = OS_TASK_READY;
    Os_TaskTable[OS_BOOT_TASK_INDEX].stackPointer =
        Os_InitializeTaskStack(Os_BootTaskStack, OS_TASK_STACK_WORDS,
                               Os_CfgBootSupervisorTask);

    Os_TaskTable[OS_STATE_TASK_INDEX].stackBase = Os_StateTaskStack;
    Os_TaskTable[OS_STATE_TASK_INDEX].stackWords = OS_TASK_STACK_WORDS;
    Os_TaskTable[OS_STATE_TASK_INDEX].entry = Os_CfgStateTask;
    Os_TaskTable[OS_STATE_TASK_INDEX].priority = OS_STATE_TASK_PRIORITY;
    Os_TaskTable[OS_STATE_TASK_INDEX].extendedTask = TRUE;
    Os_TaskTable[OS_STATE_TASK_INDEX].events = 0U;
    Os_TaskTable[OS_STATE_TASK_INDEX].waitMask = 0U;
    Os_TaskTable[OS_STATE_TASK_INDEX].state = OS_TASK_READY;
    Os_TaskTable[OS_STATE_TASK_INDEX].stackPointer =
        Os_InitializeTaskStack(Os_StateTaskStack, OS_TASK_STACK_WORDS,
                               Os_CfgStateTask);

    Os_TaskTable[OS_SAFETY_TASK_INDEX].stackBase = Os_SafetyTaskStack;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].stackWords = OS_TASK_STACK_WORDS;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].entry = Os_CfgSafetyTask;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].priority = OS_SAFETY_TASK_PRIORITY;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].extendedTask = TRUE;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].events = 0U;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].waitMask = 0U;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].state = OS_TASK_READY;
    Os_TaskTable[OS_SAFETY_TASK_INDEX].stackPointer =
        Os_InitializeTaskStack(Os_SafetyTaskStack, OS_TASK_STACK_WORDS,
                               Os_CfgSafetyTask);

    Os_Counter = 0U;
    Os_BootPeriodCounter = 0U;
    Os_CurrentTask = OS_INVALID_TASK_INDEX;

    sysTickControlState = SysTick->CTRL;

    if (((sysTickControlState & OS_SYSTICK_CONTROL_MASK) != 0U) ||
        ((SysTick->LOAD & SysTick_LOAD_RELOAD_Msk) != OS_SYSTICK_RELOAD) ||
        ((SCB->CCR & SCB_CCR_STKALIGN_Msk) == 0U) ||
        (NVIC_GetPriorityGrouping() != 0U) ||
        (NVIC_GetPriority(SVCall_IRQn) != OS_SVC_PRIORITY) ||
        (NVIC_GetPriority(SysTick_IRQn) != OS_SYSTICK_PRIORITY) ||
        (NVIC_GetPriority(PendSV_IRQn) != OS_PENDSV_PRIORITY) ||
        (Os_IsTaskStackValid(OS_IDLE_TASK_INDEX) != TRUE) ||
        (Os_IsTaskStackValid(OS_BOOT_TASK_INDEX) != TRUE) ||
        (Os_IsTaskStackValid(OS_STATE_TASK_INDEX) != TRUE) ||
        (Os_IsTaskStackValid(OS_SAFETY_TASK_INDEX) != TRUE))
    {
        return E_NOT_OK;
    }

    Os_Initialized = TRUE;
    return E_OK;
}

void StartOS(AppModeType Mode)
{
    if ((Mode != OSDEFAULTAPPMODE) ||
        (Os_Initialized != TRUE) ||
        (Os_Running == TRUE))
    {
        Os_FatalError();
    }

    __set_BASEPRI_MAX(OS_KERNEL_BASEPRI);
    __DSB();
    __ISB();

    Os_Running = TRUE;
    (void)Os_PortSelectNextStackPointer();

    SysTick->VAL = 0U;
    SysTick->CTRL = OS_SYSTICK_CONTROL_RUNNING;

    if ((SysTick->CTRL & OS_SYSTICK_CONTROL_MASK) !=
        OS_SYSTICK_CONTROL_RUNNING)
    {
        Os_FatalError();
    }

    Os_PortStartFirstTask();
    Os_FatalError();
}

StatusType WaitEvent(EventMaskType Mask)
{
    uint32 previousBasePriority;
    Os_TaskIndexType currentTask;
    volatile Os_TaskControlBlockType *task;

    if (Mask == 0U)
    {
        return E_OS_VALUE;
    }

    previousBasePriority = Os_EnterCritical();
    currentTask = Os_CurrentTask;

    if ((Os_Running != TRUE) || (currentTask >= OS_TASK_COUNT))
    {
        Os_ExitCritical(previousBasePriority);
        return E_OS_STATE;
    }

    task = &Os_TaskTable[currentTask];

    if ((task->extendedTask != TRUE) ||
        (task->state != OS_TASK_RUNNING))
    {
        Os_ExitCritical(previousBasePriority);
        return E_OS_ACCESS;
    }

    if ((task->events & Mask) == 0U)
    {
        task->waitMask = Mask;
        task->state = OS_TASK_WAITING;
        Os_RequestDispatch();
    }

    Os_ExitCritical(previousBasePriority);
    return E_OS_OK;
}

StatusType ClearEvent(EventMaskType Mask)
{
    uint32 previousBasePriority;
    Os_TaskIndexType currentTask;
    volatile Os_TaskControlBlockType *task;

    if (Mask == 0U)
    {
        return E_OS_VALUE;
    }

    previousBasePriority = Os_EnterCritical();
    currentTask = Os_CurrentTask;

    if ((Os_Running != TRUE) || (currentTask >= OS_TASK_COUNT))
    {
        Os_ExitCritical(previousBasePriority);
        return E_OS_STATE;
    }

    task = &Os_TaskTable[currentTask];

    if ((task->extendedTask != TRUE) ||
        (task->state != OS_TASK_RUNNING))
    {
        Os_ExitCritical(previousBasePriority);
        return E_OS_ACCESS;
    }

    task->events &= ~Mask;
    Os_ExitCritical(previousBasePriority);

    return E_OS_OK;
}

Os_TickType Os_GetCounterValue(void)
{
    return Os_Counter;
}

boolean Os_IsRunning(void)
{
    return Os_Running;
}

void SysTick_Handler(void)
{
    if (Os_Running != TRUE)
    {
        Os_FatalError();
    }

    Os_Counter++;
    Os_BootPeriodCounter++;

    Os_SetEventFromSysTick(OS_SAFETY_TASK_INDEX, OS_SAFETY_TASK_EVENT);
    Os_SetEventFromSysTick(OS_STATE_TASK_INDEX, OS_STATE_TASK_EVENT);

    if (Os_BootPeriodCounter >= OS_BOOT_SUPERVISION_PERIOD_TICKS)
    {
        Os_BootPeriodCounter = 0U;
        Os_SetEventFromSysTick(OS_BOOT_TASK_INDEX,
                               OS_BOOT_SUPERVISION_EVENT);
    }

    Os_RequestDispatch();

    /* ES0431 exception-return workaround for this nonfatal Cat2 handler. */
    __DSB();
}

void Os_PortSaveCurrentStackPointer(uint32 *stackPointer)
{
    Os_TaskIndexType currentTask = Os_CurrentTask;

    if ((currentTask >= OS_TASK_COUNT) ||
        (Os_IsTaskStackValid(currentTask) != TRUE))
    {
        Os_FatalError();
    }

    Os_TaskTable[currentTask].stackPointer = stackPointer;
}

uint32 *Os_PortSelectNextStackPointer(void)
{
    Os_TaskIndexType taskIndex;
    Os_TaskIndexType selectedTask = OS_INVALID_TASK_INDEX;
    uint8 selectedPriority = 0U;

    if ((Os_CurrentTask < OS_TASK_COUNT) &&
        (Os_TaskTable[Os_CurrentTask].state == OS_TASK_RUNNING))
    {
        Os_TaskTable[Os_CurrentTask].state = OS_TASK_READY;
    }

    for (taskIndex = 0U; taskIndex < OS_TASK_COUNT; taskIndex++)
    {
        if ((Os_TaskTable[taskIndex].state == OS_TASK_READY) &&
            ((selectedTask == OS_INVALID_TASK_INDEX) ||
             (Os_TaskTable[taskIndex].priority > selectedPriority)))
        {
            selectedTask = taskIndex;
            selectedPriority = Os_TaskTable[taskIndex].priority;
        }
    }

    if ((selectedTask >= OS_TASK_COUNT) ||
        (Os_IsTaskStackValid(selectedTask) != TRUE))
    {
        Os_FatalError();
    }

    Os_CurrentTask = selectedTask;
    Os_TaskTable[selectedTask].state = OS_TASK_RUNNING;

    return Os_TaskTable[selectedTask].stackPointer;
}

uint32 *Os_PortGetCurrentStackPointer(void)
{
    Os_TaskIndexType currentTask = Os_CurrentTask;

    if ((currentTask >= OS_TASK_COUNT) ||
        (Os_IsTaskStackValid(currentTask) != TRUE))
    {
        Os_FatalError();
    }

    return Os_TaskTable[currentTask].stackPointer;
}
