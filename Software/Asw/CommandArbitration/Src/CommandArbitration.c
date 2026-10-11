#include "CommandArbitration.h"

#define COMMAND_ARBITRATION_START       0x01U
#define COMMAND_ARBITRATION_SET_CURRENT 0x03U
#define COMMAND_ARBITRATION_MAX_LEASE_MS 100U

Std_ReturnType CommandArbitration_Evaluate(
    const CommandArbitration_RequestType *request,
    CommandArbitration_DecisionType *decision)
{
    if ((request == NULL_PTR) || (decision == NULL_PTR) ||
        ((request->command != COMMAND_ARBITRATION_START) &&
         (request->command != COMMAND_ARBITRATION_SET_CURRENT)) ||
        (request->leaseMs == 0U) ||
        (request->leaseMs > COMMAND_ARBITRATION_MAX_LEASE_MS) ||
        (request->currentMa != 0))
    {
        return E_NOT_OK;
    }

    /* Nonzero authority and state transitions remain explicitly unavailable. */
    *decision = COMMAND_ARBITRATION_NOT_READY;
    return E_OK;
}
