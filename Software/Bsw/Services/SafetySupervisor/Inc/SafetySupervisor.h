#ifndef SAFETY_SUPERVISOR_H
#define SAFETY_SUPERVISOR_H

#include "Std_Types.h"
#include "Sc1_MeasurementIdentity.h"

typedef struct
{
    uint16 vbusRaw;
    uint16 temperatureRaw;
    uint32 acquisitionSequence;
    uint32 osTick;
    uint32 pwmGenerationId;
    boolean acquisitionValid;
    boolean limitsValid;
} SafetySupervisor_SnapshotType;

typedef struct
{
    Sc1_MeasurementIdentityType identity;
    boolean structuralValid;
    boolean measurementApproved;
} SafetySupervisor_ReadinessType;

#define SAFETY_SUPERVISOR_EVENT_CAUSE_NONE          0x0000U
#define SAFETY_SUPERVISOR_EVENT_CAUSE_START_FAILED  0x0001U
#define SAFETY_SUPERVISOR_EVENT_CAUSE_TIMEOUT       0x0002U
#define SAFETY_SUPERVISOR_EVENT_CAUSE_ADC_FAILED    0x0004U
#define SAFETY_SUPERVISOR_EVENT_CAUSE_SEQUENCE      0x0008U
#define SAFETY_SUPERVISOR_EVENT_CAUSE_INTERNAL      0x0010U

/** Initialize the non-actuating ADC2 supervision service before StartOS. */
Std_ReturnType SafetySupervisor_Init(void);

/** Execute one nonblocking 1 ms ADC2 safety acquisition activation. */
void SafetySupervisor_MainFunction(void);

/** Consume the newest complete three-slot mailbox snapshot. */
Std_ReturnType SafetySupervisor_GetLatestSnapshot(
    SafetySupervisor_SnapshotType *Snapshot);

/** Copy immutable-configuration approval and identity readiness. */
Std_ReturnType SafetySupervisor_GetReadiness(
    SafetySupervisor_ReadinessType *Readiness);

/** Evaluate the exact one-millisecond SC-1 READY snapshot contract. */
boolean SafetySupervisor_IsSnapshotReady(
    const SafetySupervisor_SnapshotType *Snapshot,
    uint32 CurrentTick);

/** Acquire the reset-latched producer event word. */
uint32 SafetySupervisor_GetFaultEvent(void);

/** Acquire the 1 ms runnable alive counter. */
uint32 SafetySupervisor_GetAliveCounter(void);

/** Acquire the number of publications dropped while every slot was busy. */
Std_ReturnType SafetySupervisor_GetDropCount(uint32 *DropCount);

#endif /* SAFETY_SUPERVISOR_H */
