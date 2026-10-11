#ifndef USER_COMMAND_H
#define USER_COMMAND_H

#include "Std_Types.h"

typedef struct
{
    boolean enabled;
    sint32 currentMa;
    uint32 runDurationMs;
    uint16 leaseMs;
} UserCommand_ProfileType;

/**
 * Copy the immutable standalone commissioning profile after validating it.
 * The destination is cleared before validation so every failure is fail-safe.
 */
Std_ReturnType UserCommand_GetProfile(UserCommand_ProfileType *Profile);

#endif /* USER_COMMAND_H */
