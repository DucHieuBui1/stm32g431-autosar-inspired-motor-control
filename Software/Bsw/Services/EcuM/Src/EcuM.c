#include "EcuM.h"

#include "Std_Types.h"
#include "Adc.h"
#include "Mcu.h"
#include "MotorControlCdd.h"
#include "Os.h"
#include "Os_Cfg.h"
#include "SafetySupervisor.h"
#include "SerialCommunicationCdd.h"
#include "StateManager.h"
#include "Wdg.h"
#include "WdgM.h"

STATIC boolean EcuM_WdgRefreshAuthorityActive = FALSE;
STATIC Os_TickType EcuM_LastBootCheckpointTick;
STATIC Mcu_StartupInfoType EcuM_StartupInfo;

STATIC void EcuM_EnterEarlySafeFailure(void)
{
    EcuM_WdgRefreshAuthorityActive = FALSE;
    MotorControlCdd_EmergencyOff();

    for (;;)
    {
        /* Do not progress boot, authorize outputs, or refresh IWDG. */
    }
}

void EcuM_Init(void)
{
    if (Mcu_EarlyClockGateInit() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (MotorControlCdd_EarlySafeInit() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (StateManager_InitBootSafe() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (Mcu_StartupValidationInit(&EcuM_StartupInfo) != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (Mcu_ClockBaselineInit() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (Wdg_Start() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (MotorControlCdd_CheckEarlySafeState() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (Mcu_Adc12PeripheralClockInit() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if ((Adc_Init() != E_OK) ||
        (Adc_CheckCommonConfiguration() != E_OK))
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (MotorControlCdd_Init() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (MotorControlCdd_CheckStaticConfiguration() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if ((Mcu_SerialPeripheralClockInit() != E_OK) ||
        (SerialCommunicationCdd_Init(EcuM_StartupInfo.bootIdentity) != E_OK) ||
        (SerialCommunicationCdd_CheckStaticConfiguration() != E_OK))
    {
        EcuM_EnterEarlySafeFailure();
    }

    if ((SafetySupervisor_Init() != E_OK) ||
        (StateManager_EnterSelfTest() != E_OK))
    {
        EcuM_EnterEarlySafeFailure();
    }

    EcuM_WdgRefreshAuthorityActive = TRUE;

    if (Os_Init() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (MotorControlCdd_CheckStaticConfiguration() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    if (SerialCommunicationCdd_CheckStaticConfiguration() != E_OK)
    {
        EcuM_EnterEarlySafeFailure();
    }

    EcuM_LastBootCheckpointTick = Os_GetCounterValue();
    Wdg_Trigger();
}

void EcuM_Start(void)
{
    StartOS(OSDEFAULTAPPMODE);
    EcuM_EnterEarlySafeFailure();
}

void EcuM_BootSupervisorMainFunction(void)
{
    Os_TickType currentTick = Os_GetCounterValue();

    if (EcuM_WdgRefreshAuthorityActive == TRUE)
    {
        Os_TickType elapsedTicks =
            currentTick - EcuM_LastBootCheckpointTick;
        WdgM_HandoffTokenType handoffToken;

        if ((Os_IsRunning() != TRUE) ||
            (elapsedTicks != OS_BOOT_SUPERVISION_PERIOD_TICKS) ||
            (Mcu_GetClockFaultEvents() != MCU_CLOCK_FAULT_EVENT_NONE) ||
            (Adc_CheckCommonConfiguration() != E_OK) ||
            (MotorControlCdd_CheckStaticConfiguration() != E_OK) ||
            (SerialCommunicationCdd_CheckStaticConfiguration() != E_OK) ||
            (WdgM_PrepareHandoff(currentTick, &handoffToken) != E_OK))
        {
            EcuM_EnterEarlySafeFailure();
        }

        EcuM_LastBootCheckpointTick = currentTick;
        Wdg_Trigger();
        EcuM_WdgRefreshAuthorityActive = FALSE;
        if (WdgM_AcceptHandoff(handoffToken) != E_OK)
        {
            EcuM_EnterEarlySafeFailure();
        }
        return;
    }

    if (WdgM_HasRefreshAuthority() != TRUE)
    {
        if ((WdgM_GetFaultEvent() & 0xFFFFU) != 0U)
        {
            return;
        }
        EcuM_EnterEarlySafeFailure();
    }

    WdgM_MainFunction();
    if (WdgM_HasRefreshAuthority() != TRUE)
    {
        if ((WdgM_GetFaultEvent() & 0xFFFFU) != 0U)
        {
            return;
        }
        EcuM_EnterEarlySafeFailure();
    }
}
