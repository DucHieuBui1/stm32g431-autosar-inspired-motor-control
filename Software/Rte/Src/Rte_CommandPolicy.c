#include "Rte_CommandPolicy.h"

#include "CommandArbitration.h"

Std_ReturnType Rte_CommandPolicyEvaluate(
    const Rte_CommandPolicyRequestType *request,
    Rte_CommandPolicyDecisionType *decision)
{
    CommandArbitration_RequestType aswRequest;
    CommandArbitration_DecisionType aswDecision;

    if ((request == NULL_PTR) || (decision == NULL_PTR))
    {
        return E_NOT_OK;
    }

    aswRequest.command = request->command;
    aswRequest.leaseMs = request->leaseMs;
    aswRequest.currentMa = request->currentMa;
    aswRequest.stateSnapshot = request->stateSnapshot;
    if (CommandArbitration_Evaluate(&aswRequest, &aswDecision) != E_OK)
    {
        return E_NOT_OK;
    }
    if (aswDecision != COMMAND_ARBITRATION_NOT_READY)
    {
        return E_NOT_OK;
    }
    *decision = RTE_COMMAND_POLICY_NOT_READY;
    return E_OK;
}
