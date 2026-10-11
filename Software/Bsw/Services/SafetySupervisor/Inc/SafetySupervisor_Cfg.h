#ifndef SAFETY_SUPERVISOR_CFG_H
#define SAFETY_SUPERVISOR_CFG_H

#include "Sc1_MeasurementIdentity.h"

typedef struct
{
    uint32 numerator;
    uint32 denominator;
    sint32 offset;
} SafetySupervisor_ScalingMetadataType;

typedef struct
{
    boolean measurementApproved;
    uint16 vbusRawMinimum;
    uint16 vbusRawMaximum;
    uint16 temperatureRawMinimum;
    uint16 temperatureRawMaximum;
    SafetySupervisor_ScalingMetadataType vbusMillivolts;
    SafetySupervisor_ScalingMetadataType temperatureCentiDegrees;
    uint32 readySnapshotMaximumAgeMs;
    Sc1_MeasurementIdentityType identity;
} SafetySupervisor_ConfigType;

/** Copy and validate the immutable SC-1 raw safety-limit configuration. */
Std_ReturnType SafetySupervisor_CfgGet(SafetySupervisor_ConfigType *Config);

#endif /* SAFETY_SUPERVISOR_CFG_H */
