#ifndef MCU_COMMUNICATION_H
#define MCU_COMMUNICATION_H

#include "Std_Types.h"

Std_ReturnType Mcu_BootIdentityPrepare(
    uint32 RawResetFlags,
    uint32 *BootIdentity);

#endif /* MCU_COMMUNICATION_H */
