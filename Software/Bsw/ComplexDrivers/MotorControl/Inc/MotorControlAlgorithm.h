#ifndef MOTOR_CONTROL_ALGORITHM_H
#define MOTOR_CONTROL_ALGORITHM_H

#include "Std_Types.h"

#define MOTOR_CONTROL_ALGORITHM_Q15_MIN       ((sint32)-32768)
#define MOTOR_CONTROL_ALGORITHM_Q15_MAX       ((sint32)32767)
#define MOTOR_CONTROL_ALGORITHM_Q15_ONE       ((sint32)32767)
#define MOTOR_CONTROL_ALGORITHM_Q30_ONE       ((uint32)1073741824UL)

typedef struct
{
    sint32 alphaQ15;
    sint32 betaQ15;
} MotorControlAlgorithm_AlphaBetaType;

typedef struct
{
    sint32 dQ15;
    sint32 qQ15;
} MotorControlAlgorithm_DqType;

typedef struct
{
    sint32 kpQ15;
    sint32 kiQ15;
    sint32 lowerQ15;
    sint32 upperQ15;
} MotorControlAlgorithm_PiConfigType;

typedef struct
{
    MotorControlAlgorithm_PiConfigType config;
    sint64 integratorQ30;
    boolean initialized;
} MotorControlAlgorithm_PiType;

typedef struct
{
    sint32 outputQ15;
    boolean limited;
} MotorControlAlgorithm_PiResultType;

typedef struct
{
    uint16 dutyAQ15;
    uint16 dutyBQ15;
    uint16 dutyCQ15;
    uint8 sector;
    boolean limited;
} MotorControlAlgorithm_SvpwmResultType;

Std_ReturnType MotorControlAlgorithm_Clarke(
    sint32 PhaseAQ15,
    sint32 PhaseBQ15,
    sint32 PhaseCQ15,
    uint32 PhaseSumToleranceQ15,
    MotorControlAlgorithm_AlphaBetaType *Result);

Std_ReturnType MotorControlAlgorithm_Park(
    const MotorControlAlgorithm_AlphaBetaType *AlphaBeta,
    sint32 SinQ15,
    sint32 CosQ15,
    uint32 TrigNormToleranceQ30,
    MotorControlAlgorithm_DqType *Result);

Std_ReturnType MotorControlAlgorithm_InversePark(
    const MotorControlAlgorithm_DqType *Dq,
    sint32 SinQ15,
    sint32 CosQ15,
    uint32 TrigNormToleranceQ30,
    MotorControlAlgorithm_AlphaBetaType *Result);

Std_ReturnType MotorControlAlgorithm_PiInit(
    MotorControlAlgorithm_PiType *Object,
    const MotorControlAlgorithm_PiConfigType *Config,
    sint32 InitialIntegratorQ15);

Std_ReturnType MotorControlAlgorithm_PiReset(
    MotorControlAlgorithm_PiType *Object,
    sint32 IntegratorQ15);

Std_ReturnType MotorControlAlgorithm_PiStep(
    MotorControlAlgorithm_PiType *Object,
    sint32 ErrorQ15,
    MotorControlAlgorithm_PiResultType *Result);

Std_ReturnType MotorControlAlgorithm_Svpwm(
    const MotorControlAlgorithm_AlphaBetaType *AlphaBeta,
    MotorControlAlgorithm_SvpwmResultType *Result);

#endif /* MOTOR_CONTROL_ALGORITHM_H */
