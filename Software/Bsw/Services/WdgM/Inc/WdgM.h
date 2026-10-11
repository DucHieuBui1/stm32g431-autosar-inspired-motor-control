#ifndef WDGM_H
#define WDGM_H

#include "Std_Types.h"

typedef uint32 WdgM_HandoffTokenType;

#define WDGM_EVENT_CAUSE_NONE               0x0000U
#define WDGM_EVENT_CAUSE_OS_TIME            0x0001U
#define WDGM_EVENT_CAUSE_SAFETY_ALIVE       0x0002U
#define WDGM_EVENT_CAUSE_STATE_ALIVE        0x0004U
#define WDGM_EVENT_CAUSE_FOC_PROFILE        0x0008U
#define WDGM_EVENT_CAUSE_HANDOFF            0x0010U

/** Self-test runtime supervision and create a one-use handoff token. */
Std_ReturnType WdgM_PrepareHandoff(
    uint32 CurrentTick,
    WdgM_HandoffTokenType *Token);

/** Consume the prepared token and irreversibly take refresh authority. */
Std_ReturnType WdgM_AcceptHandoff(WdgM_HandoffTokenType Token);

/** Execute the 10 ms alive/time validation and conditionally refresh IWDG. */
void WdgM_MainFunction(void);

/** Acquire the reset-latched WdgM supervision event word. */
uint32 WdgM_GetFaultEvent(void);

/** Report whether WdgM owns the one-way runtime refresh authority. */
boolean WdgM_HasRefreshAuthority(void);

#endif /* WDGM_H */
