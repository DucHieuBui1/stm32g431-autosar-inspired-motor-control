#ifndef SC1_MEASUREMENT_IDENTITY_H
#define SC1_MEASUREMENT_IDENTITY_H

#include "Std_Types.h"

#define SC1_MEASUREMENT_IDENTITY_DIGEST_WORD_COUNT 8U
#define SC1_MEASUREMENT_SCHEMA_VERSION             1UL
#define SC1_MEASUREMENT_BOARD_PROFILE_CODE         0x1367C051UL

typedef struct
{
    uint32 schemaVersion;
    uint32 boardProfileCode;
    uint32 evidenceSha256[SC1_MEASUREMENT_IDENTITY_DIGEST_WORD_COUNT];
} Sc1_MeasurementIdentityType;

#endif /* SC1_MEASUREMENT_IDENTITY_H */
