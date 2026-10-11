#include "Os_Cfg.h"

#include "EcuM.h"
#include "SafetySupervisor.h"
#include "StateManager.h"

void Os_CfgBootSupervisorTask(void)
{
    for (;;)
    {
        if (WaitEvent(OS_BOOT_SUPERVISION_EVENT) != E_OS_OK)
        {
            return;
        }

        if (ClearEvent(OS_BOOT_SUPERVISION_EVENT) != E_OS_OK)
        {
            return;
        }

        EcuM_BootSupervisorMainFunction();
    }
}

void Os_CfgSafetyTask(void)
{
    for (;;)
    {
        if (WaitEvent(OS_SAFETY_TASK_EVENT) != E_OS_OK)
        {
            return;
        }

        if (ClearEvent(OS_SAFETY_TASK_EVENT) != E_OS_OK)
        {
            return;
        }

        SafetySupervisor_MainFunction();
    }
}

void Os_CfgStateTask(void)
{
    for (;;)
    {
        if (WaitEvent(OS_STATE_TASK_EVENT) != E_OS_OK)
        {
            return;
        }

        if (ClearEvent(OS_STATE_TASK_EVENT) != E_OS_OK)
        {
            return;
        }

        StateManager_MainFunction();
    }
}
