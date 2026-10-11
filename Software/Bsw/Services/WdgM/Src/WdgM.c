#include "WdgM.h"

#include "MotorControlCdd.h"
#include "Os.h"
#include "Os_Cfg.h"
#include "SafetySupervisor.h"
#include "StateManager.h"
#include "Wdg.h"

#define WDGM_TOKEN_MAGIC                    0x5744474DU
#define WDGM_SAFETY_ALIVE_PER_CYCLE         10U
#define WDGM_STATE_ALIVE_PER_CYCLE          10U

#ifndef WDGM_TEST_FAULT_READ_HOOK
#define WDGM_TEST_FAULT_READ_HOOK() ((void)0)
#endif

typedef enum
{
    WDGM_UNINITIALIZED = 0,
    WDGM_TOKEN_PREPARED,
    WDGM_ACTIVE,
    WDGM_FAILED
} WdgM_StateType;

STATIC WdgM_StateType WdgM_State;
STATIC WdgM_HandoffTokenType WdgM_PreparedToken;
STATIC uint32 WdgM_LastTick;
STATIC uint32 WdgM_LastSafetyAlive;
STATIC uint32 WdgM_LastStateAlive;
_Alignas(4) STATIC uint32 WdgM_FaultEvent;

STATIC void WdgM_PublishFault(uint16 cause)
{
    uint32 previous = __atomic_load_n(&WdgM_FaultEvent, __ATOMIC_RELAXED);
    uint32 sequence = (previous >> 16U) + 1U;
    uint32 event = (sequence << 16U) | (uint32)cause;

    MotorControlCdd_EmergencyOff();
    __atomic_store_n(&WdgM_FaultEvent, event, __ATOMIC_RELEASE);
    StateManager_NotifyCriticalFault(
        STATE_MANAGER_CRITICAL_CLASS_WDGM);
    WdgM_State = WDGM_FAILED;
}

Std_ReturnType WdgM_PrepareHandoff(
    uint32 CurrentTick,
    WdgM_HandoffTokenType *Token)
{
    uint32 safetyAlive;
    uint32 stateAlive;
    StateManager_StateType state;

    if ((Token == NULL_PTR) ||
        (WdgM_State != WDGM_UNINITIALIZED) ||
        (Os_IsRunning() != TRUE))
    {
        return E_NOT_OK;
    }

    safetyAlive = SafetySupervisor_GetAliveCounter();
    stateAlive = StateManager_GetAliveCounter();
    state = StateManager_GetState();
    if ((safetyAlive != WDGM_SAFETY_ALIVE_PER_CYCLE) ||
        (stateAlive != WDGM_STATE_ALIVE_PER_CYCLE) ||
        ((state != STATE_MANAGER_SELF_TEST) &&
         (state != STATE_MANAGER_CALIBRATING) &&
         (state != STATE_MANAGER_FAULT_LATCHED)))
    {
        return E_NOT_OK;
    }

    WdgM_LastTick = CurrentTick;
    WdgM_LastSafetyAlive = safetyAlive;
    WdgM_LastStateAlive = stateAlive;
    __atomic_store_n(&WdgM_FaultEvent, 0U, __ATOMIC_RELAXED);
    WdgM_PreparedToken = WDGM_TOKEN_MAGIC ^ CurrentTick ^
                         safetyAlive ^ (stateAlive << 16U);
    if (WdgM_PreparedToken == 0U)
    {
        WdgM_PreparedToken = WDGM_TOKEN_MAGIC;
    }

    *Token = WdgM_PreparedToken;
    WdgM_State = WDGM_TOKEN_PREPARED;
    return E_OK;
}

Std_ReturnType WdgM_AcceptHandoff(WdgM_HandoffTokenType Token)
{
    if ((WdgM_State != WDGM_TOKEN_PREPARED) ||
        (Token == 0U) ||
        (Token != WdgM_PreparedToken))
    {
        return E_NOT_OK;
    }

    WdgM_PreparedToken = 0U;
    WdgM_State = WDGM_ACTIVE;
    return E_OK;
}

void WdgM_MainFunction(void)
{
    uint32 currentTick;
    uint32 safetyAlive;
    uint32 stateAlive;
    uint16 failureCause = WDGM_EVENT_CAUSE_NONE;
    StateManager_StateType state;

    if (WdgM_State != WDGM_ACTIVE)
    {
        return;
    }

    currentTick = Os_GetCounterValue();
    safetyAlive = SafetySupervisor_GetAliveCounter();
    stateAlive = StateManager_GetAliveCounter();
    state = StateManager_GetState();

    if ((uint32)(currentTick - WdgM_LastTick) !=
        OS_RUNTIME_SUPERVISION_PERIOD_TICKS)
    {
        failureCause |= WDGM_EVENT_CAUSE_OS_TIME;
    }
    if ((uint32)(safetyAlive - WdgM_LastSafetyAlive) !=
        WDGM_SAFETY_ALIVE_PER_CYCLE)
    {
        failureCause |= WDGM_EVENT_CAUSE_SAFETY_ALIVE;
    }
    if ((uint32)(stateAlive - WdgM_LastStateAlive) !=
        WDGM_STATE_ALIVE_PER_CYCLE)
    {
        failureCause |= WDGM_EVENT_CAUSE_STATE_ALIVE;
    }
    if ((state == STATE_MANAGER_STARTING) ||
        (state == STATE_MANAGER_RUN) ||
        (state == STATE_MANAGER_STOPPING))
    {
        /* The FOC alive profile is deliberately unavailable in Step 7A. */
        failureCause |= WDGM_EVENT_CAUSE_FOC_PROFILE;
    }

    if (failureCause != WDGM_EVENT_CAUSE_NONE)
    {
        WdgM_PublishFault(failureCause);
        return;
    }

    WdgM_LastTick = currentTick;
    WdgM_LastSafetyAlive = safetyAlive;
    WdgM_LastStateAlive = stateAlive;
    Wdg_Trigger();
}

uint32 WdgM_GetFaultEvent(void)
{
    uint32 event = __atomic_load_n(&WdgM_FaultEvent, __ATOMIC_ACQUIRE);

    WDGM_TEST_FAULT_READ_HOOK();
    return event;
}

boolean WdgM_HasRefreshAuthority(void)
{
    return (WdgM_State == WDGM_ACTIVE) ? TRUE : FALSE;
}
