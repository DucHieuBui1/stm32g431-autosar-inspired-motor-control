#ifndef OS_H
#define OS_H

#include "Std_Types.h"

typedef uint8 AppModeType;
typedef uint8 StatusType;
typedef uint32 EventMaskType;
typedef uint32 Os_TickType;

#define OSDEFAULTAPPMODE       ((AppModeType)0U)

#define E_OS_OK                ((StatusType)0U)
#define E_OS_ACCESS            ((StatusType)1U)
#define E_OS_STATE             ((StatusType)2U)
#define E_OS_VALUE             ((StatusType)3U)

/**
 * @brief Initialize the static ECC1 kernel slice and validate its Cortex-M4
 *        port configuration without enabling SysTick.
 *
 * @return E_OK when static task contexts, stack guards, priority grouping,
 *         exception priorities and the disabled 1 ms SysTick configuration
 *         read back correctly.
 * @return E_NOT_OK on a configuration or readback failure.
 */
Std_ReturnType Os_Init(void);

/**
 * @brief Start the configured OS mode and dispatch the first static task.
 *
 * The static ECC1 slice contains idle, 1 ms safety/state, and 10 ms
 * boot/runtime-supervision extended tasks. The service does not return.
 *
 * @pre Os_Init returned E_OK and IWDG boot supervision is active.
 */
void StartOS(AppModeType Mode);

/** @brief Wait until at least one event in Mask is set for the current task. */
StatusType WaitEvent(EventMaskType Mask);

/** @brief Clear events owned by the current extended task. */
StatusType ClearEvent(EventMaskType Mask);

/** @brief Return the atomic 1 ms OS counter value. */
Os_TickType Os_GetCounterValue(void);

/** @brief Report whether StartOS has entered the running state. */
boolean Os_IsRunning(void);

#endif /* OS_H */
