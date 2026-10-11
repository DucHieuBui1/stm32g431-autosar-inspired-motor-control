#ifndef RTE_COMMAND_POLICY_H
#define RTE_COMMAND_POLICY_H

#include "Std_Types.h"

typedef struct
{
    uint8 command;
    uint16 leaseMs;
    sint32 currentMa;
    uint32 stateSnapshot;
} Rte_CommandPolicyRequestType;

typedef enum
{
    RTE_COMMAND_POLICY_NOT_READY = 0
} Rte_CommandPolicyDecisionType;

/** BSW-to-ASW command-policy boundary; no direct CDD or hardware API crosses it. */
Std_ReturnType Rte_CommandPolicyEvaluate(
    const Rte_CommandPolicyRequestType *request,
    Rte_CommandPolicyDecisionType *decision);

#endif /* RTE_COMMAND_POLICY_H */
