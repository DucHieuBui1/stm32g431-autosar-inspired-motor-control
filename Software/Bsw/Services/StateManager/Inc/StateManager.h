#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "Std_Types.h"

typedef enum
{
    STATE_MANAGER_RESET = 0,
    STATE_MANAGER_BOOT_SAFE,
    STATE_MANAGER_SELF_TEST,
    STATE_MANAGER_CALIBRATING,
    STATE_MANAGER_READY,
    STATE_MANAGER_STARTING,
    STATE_MANAGER_RUN,
    STATE_MANAGER_STOPPING,
    STATE_MANAGER_FAULT_LATCHED,
    STATE_MANAGER_SHUTDOWN
} StateManager_StateType;

#define STATE_MANAGER_FAULT_NONE             0x00000000U
#define STATE_MANAGER_FAULT_CLOCK            0x00000001U
#define STATE_MANAGER_FAULT_MOTOR_BREAK       0x00000002U
#define STATE_MANAGER_FAULT_MOTOR_UPDATE      0x00000004U
#define STATE_MANAGER_FAULT_MOTOR_ADC         0x00000008U
#define STATE_MANAGER_FAULT_SAFETY_ADC        0x00000010U
#define STATE_MANAGER_FAULT_WDGM              0x00000020U
#define STATE_MANAGER_FAULT_INVARIANT         0x00000040U
#define STATE_MANAGER_FAULT_CALIBRATION       0x00000080U
#define STATE_MANAGER_FAULT_UNCLASSIFIED_CRITICAL 0x00000100U

#define STATE_MANAGER_CRITICAL_CLASS_CLOCK     0x00010000U
#define STATE_MANAGER_CRITICAL_CLASS_MOTOR     0x00020000U
#define STATE_MANAGER_CRITICAL_CLASS_SAFETY    0x00040000U
#define STATE_MANAGER_CRITICAL_CLASS_WDGM      0x00080000U
#define STATE_MANAGER_CRITICAL_CLASS_INVARIANT 0x00100000U
#define STATE_MANAGER_CRITICAL_CLASS_ALL       0x001F0000U

/**
 * Monotonically publish one or more critical classes into the private state
 * interlock. Reserved or zero input also publishes INVARIANT. This operation
 * never clears a pending class and performs no OS, peripheral or logging work.
 */
void StateManager_NotifyCriticalFault(uint32 CriticalClassMask);

/** Establish the sole state writer in BOOT_SAFE after early-safe succeeds. */
Std_ReturnType StateManager_InitBootSafe(void);

/** Enter SELF_TEST after all non-actuating MCAL/CDD services initialize. */
Std_ReturnType StateManager_EnterSelfTest(void);

/** Execute one 1 ms state/fault aggregation activation. */
void StateManager_MainFunction(void);

/** Acquire the current top-level operating state. */
StateManager_StateType StateManager_GetState(void);

/** Acquire the reset-latched aggregate fault mask. */
uint32 StateManager_GetFaultMask(void);

/** Acquire the 1 ms runnable alive counter. */
uint32 StateManager_GetAliveCounter(void);

#endif /* STATE_MANAGER_H */
