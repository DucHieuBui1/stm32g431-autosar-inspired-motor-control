#ifndef OS_CFG_H
#define OS_CFG_H

#include "Os.h"

#define OS_BOOT_SUPERVISION_EVENT          ((EventMaskType)0x00000001U)
#define OS_SAFETY_TASK_EVENT               ((EventMaskType)0x00000002U)
#define OS_STATE_TASK_EVENT                ((EventMaskType)0x00000004U)
#define OS_BOOT_SUPERVISION_PERIOD_TICKS   ((Os_TickType)10U)
#define OS_RUNTIME_SUPERVISION_PERIOD_TICKS \
    OS_BOOT_SUPERVISION_PERIOD_TICKS

/** @brief Static extended-task entry for boot/runtime watchdog supervision. */
void Os_CfgBootSupervisorTask(void);

/** @brief Static extended-task entry for the 1 ms ADC2 safety path. */
void Os_CfgSafetyTask(void);

/** @brief Static extended-task entry for the 1 ms state/fault aggregator. */
void Os_CfgStateTask(void);

#endif /* OS_CFG_H */
