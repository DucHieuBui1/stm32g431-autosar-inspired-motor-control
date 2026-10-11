#ifndef COMMAND_ARBITRATION_H
#define COMMAND_ARBITRATION_H

#include "Std_Types.h"

typedef struct
{
    uint8 command;
    uint16 leaseMs;
    sint32 currentMa;
    uint32 stateSnapshot;
} CommandArbitration_RequestType;

typedef enum
{
    COMMAND_ARBITRATION_NOT_READY = 0
} CommandArbitration_DecisionType;

/**
 * Evaluate a structurally valid future motor-command candidate. The SWC has no
 * transport, state-transition, setpoint, peripheral or emergency-off access.
 */
Std_ReturnType CommandArbitration_Evaluate(
    const CommandArbitration_RequestType *request,
    CommandArbitration_DecisionType *decision);

#endif /* COMMAND_ARBITRATION_H */
