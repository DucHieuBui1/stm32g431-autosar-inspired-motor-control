#include "MotorControlAlgorithm.h"

#define MOTOR_CONTROL_ALGORITHM_INV_SQRT3_Q15       ((sint32)18919)
#define MOTOR_CONTROL_ALGORITHM_SQRT3_OVER_2_Q15    ((sint32)28378)
#define MOTOR_CONTROL_ALGORITHM_ONE_HALF_Q15         ((sint32)16384)
#define MOTOR_CONTROL_ALGORITHM_Q15_SCALE            ((sint64)32768)
#define MOTOR_CONTROL_ALGORITHM_PHASE_SUM_MAX_Q15    ((uint32)98304UL)

STATIC boolean MotorControlAlgorithm_IsQ15(sint32 value)
{
    return ((value >= MOTOR_CONTROL_ALGORITHM_Q15_MIN) &&
            (value <= MOTOR_CONTROL_ALGORITHM_Q15_MAX)) ? TRUE : FALSE;
}

STATIC sint64 MotorControlAlgorithm_RoundQ30(sint64 valueQ30)
{
    if (valueQ30 >= 0)
    {
        return (valueQ30 + 16384) >> 15U;
    }

    return -(((-valueQ30) + 16384) >> 15U);
}

STATIC sint32 MotorControlAlgorithm_DivideByTwoAway(sint32 value)
{
    if (value >= 0)
    {
        return (value + 1) >> 1U;
    }

    return -(((-value) + 1) >> 1U);
}

STATIC void MotorControlAlgorithm_ClearAlphaBeta(
    MotorControlAlgorithm_AlphaBetaType *result)
{
    if (result != NULL_PTR)
    {
        result->alphaQ15 = 0;
        result->betaQ15 = 0;
    }
}

STATIC void MotorControlAlgorithm_ClearDq(
    MotorControlAlgorithm_DqType *result)
{
    if (result != NULL_PTR)
    {
        result->dQ15 = 0;
        result->qQ15 = 0;
    }
}

STATIC boolean MotorControlAlgorithm_TrigNormValid(
    sint32 sinQ15,
    sint32 cosQ15,
    uint32 toleranceQ30)
{
    uint64 normQ30;
    uint64 differenceQ30;

    if ((MotorControlAlgorithm_IsQ15(sinQ15) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(cosQ15) != TRUE) ||
        (toleranceQ30 > MOTOR_CONTROL_ALGORITHM_Q30_ONE))
    {
        return FALSE;
    }

    normQ30 = (uint64)((sint64)sinQ15 * (sint64)sinQ15) +
                (uint64)((sint64)cosQ15 * (sint64)cosQ15);
    if (normQ30 >= (uint64)MOTOR_CONTROL_ALGORITHM_Q30_ONE)
    {
        differenceQ30 = normQ30 -
                        (uint64)MOTOR_CONTROL_ALGORITHM_Q30_ONE;
    }
    else
    {
        differenceQ30 = (uint64)MOTOR_CONTROL_ALGORITHM_Q30_ONE -
                        normQ30;
    }

    return (differenceQ30 <= (uint64)toleranceQ30) ? TRUE : FALSE;
}

STATIC boolean MotorControlAlgorithm_PiConfigValid(
    const MotorControlAlgorithm_PiConfigType *config)
{
    if (config == NULL_PTR)
    {
        return FALSE;
    }

    return ((config->kpQ15 >= 0) &&
            (config->kpQ15 <= MOTOR_CONTROL_ALGORITHM_Q15_MAX) &&
            (config->kiQ15 >= 0) &&
            (config->kiQ15 <= MOTOR_CONTROL_ALGORITHM_Q15_MAX) &&
            (MotorControlAlgorithm_IsQ15(config->lowerQ15) == TRUE) &&
            (MotorControlAlgorithm_IsQ15(config->upperQ15) == TRUE) &&
            (config->lowerQ15 < config->upperQ15)) ? TRUE : FALSE;
}

STATIC boolean MotorControlAlgorithm_PiObjectValid(
    const MotorControlAlgorithm_PiType *object)
{
    sint64 lowerQ30;
    sint64 upperQ30;

    if ((object == NULL_PTR) || (object->initialized != TRUE) ||
        (MotorControlAlgorithm_PiConfigValid(&object->config) != TRUE))
    {
        return FALSE;
    }

    lowerQ30 = (sint64)object->config.lowerQ15 *
               MOTOR_CONTROL_ALGORITHM_Q15_SCALE;
    upperQ30 = (sint64)object->config.upperQ15 *
               MOTOR_CONTROL_ALGORITHM_Q15_SCALE;
    return ((object->integratorQ30 >= lowerQ30) &&
            (object->integratorQ30 <= upperQ30)) ? TRUE : FALSE;
}

Std_ReturnType MotorControlAlgorithm_Clarke(
    sint32 PhaseAQ15,
    sint32 PhaseBQ15,
    sint32 PhaseCQ15,
    uint32 PhaseSumToleranceQ15,
    MotorControlAlgorithm_AlphaBetaType *Result)
{
    sint64 phaseSumQ15;
    uint64 phaseSumMagnitudeQ15;
    sint64 betaQ15;

    MotorControlAlgorithm_ClearAlphaBeta(Result);
    if ((Result == NULL_PTR) ||
        (MotorControlAlgorithm_IsQ15(PhaseAQ15) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(PhaseBQ15) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(PhaseCQ15) != TRUE) ||
        (PhaseSumToleranceQ15 >
         MOTOR_CONTROL_ALGORITHM_PHASE_SUM_MAX_Q15))
    {
        return E_NOT_OK;
    }

    phaseSumQ15 = (sint64)PhaseAQ15 + (sint64)PhaseBQ15 +
                  (sint64)PhaseCQ15;
    phaseSumMagnitudeQ15 = (phaseSumQ15 >= 0) ?
        (uint64)phaseSumQ15 : (uint64)(-phaseSumQ15);
    if (phaseSumMagnitudeQ15 > (uint64)PhaseSumToleranceQ15)
    {
        return E_NOT_OK;
    }

    betaQ15 = MotorControlAlgorithm_RoundQ30(
        ((sint64)PhaseAQ15 + (2 * (sint64)PhaseBQ15)) *
        (sint64)MOTOR_CONTROL_ALGORITHM_INV_SQRT3_Q15);
    if ((betaQ15 < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN) ||
        (betaQ15 > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX))
    {
        return E_NOT_OK;
    }

    Result->alphaQ15 = PhaseAQ15;
    Result->betaQ15 = (sint32)betaQ15;
    return E_OK;
}

Std_ReturnType MotorControlAlgorithm_Park(
    const MotorControlAlgorithm_AlphaBetaType *AlphaBeta,
    sint32 SinQ15,
    sint32 CosQ15,
    uint32 TrigNormToleranceQ30,
    MotorControlAlgorithm_DqType *Result)
{
    sint64 dQ15;
    sint64 qQ15;

    MotorControlAlgorithm_ClearDq(Result);
    if ((Result == NULL_PTR) || (AlphaBeta == NULL_PTR) ||
        (MotorControlAlgorithm_IsQ15(AlphaBeta->alphaQ15) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(AlphaBeta->betaQ15) != TRUE) ||
        (MotorControlAlgorithm_TrigNormValid(
             SinQ15, CosQ15, TrigNormToleranceQ30) != TRUE))
    {
        return E_NOT_OK;
    }

    dQ15 = MotorControlAlgorithm_RoundQ30(
        ((sint64)AlphaBeta->alphaQ15 * (sint64)CosQ15) +
        ((sint64)AlphaBeta->betaQ15 * (sint64)SinQ15));
    qQ15 = MotorControlAlgorithm_RoundQ30(
        -((sint64)AlphaBeta->alphaQ15 * (sint64)SinQ15) +
        ((sint64)AlphaBeta->betaQ15 * (sint64)CosQ15));
    if ((dQ15 < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN) ||
        (dQ15 > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX) ||
        (qQ15 < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN) ||
        (qQ15 > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX))
    {
        return E_NOT_OK;
    }

    Result->dQ15 = (sint32)dQ15;
    Result->qQ15 = (sint32)qQ15;
    return E_OK;
}

Std_ReturnType MotorControlAlgorithm_InversePark(
    const MotorControlAlgorithm_DqType *Dq,
    sint32 SinQ15,
    sint32 CosQ15,
    uint32 TrigNormToleranceQ30,
    MotorControlAlgorithm_AlphaBetaType *Result)
{
    sint64 alphaQ15;
    sint64 betaQ15;

    MotorControlAlgorithm_ClearAlphaBeta(Result);
    if ((Result == NULL_PTR) || (Dq == NULL_PTR) ||
        (MotorControlAlgorithm_IsQ15(Dq->dQ15) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(Dq->qQ15) != TRUE) ||
        (MotorControlAlgorithm_TrigNormValid(
             SinQ15, CosQ15, TrigNormToleranceQ30) != TRUE))
    {
        return E_NOT_OK;
    }

    alphaQ15 = MotorControlAlgorithm_RoundQ30(
        ((sint64)Dq->dQ15 * (sint64)CosQ15) -
        ((sint64)Dq->qQ15 * (sint64)SinQ15));
    betaQ15 = MotorControlAlgorithm_RoundQ30(
        ((sint64)Dq->dQ15 * (sint64)SinQ15) +
        ((sint64)Dq->qQ15 * (sint64)CosQ15));
    if ((alphaQ15 < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN) ||
        (alphaQ15 > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX) ||
        (betaQ15 < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN) ||
        (betaQ15 > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX))
    {
        return E_NOT_OK;
    }

    Result->alphaQ15 = (sint32)alphaQ15;
    Result->betaQ15 = (sint32)betaQ15;
    return E_OK;
}

Std_ReturnType MotorControlAlgorithm_PiInit(
    MotorControlAlgorithm_PiType *Object,
    const MotorControlAlgorithm_PiConfigType *Config,
    sint32 InitialIntegratorQ15)
{
    if (Object == NULL_PTR)
    {
        return E_NOT_OK;
    }

    Object->config.kpQ15 = 0;
    Object->config.kiQ15 = 0;
    Object->config.lowerQ15 = 0;
    Object->config.upperQ15 = 0;
    Object->integratorQ30 = 0;
    Object->initialized = FALSE;

    if ((MotorControlAlgorithm_PiConfigValid(Config) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(InitialIntegratorQ15) != TRUE) ||
        (InitialIntegratorQ15 < Config->lowerQ15) ||
        (InitialIntegratorQ15 > Config->upperQ15))
    {
        return E_NOT_OK;
    }

    Object->config = *Config;
    Object->integratorQ30 =
        (sint64)InitialIntegratorQ15 *
        MOTOR_CONTROL_ALGORITHM_Q15_SCALE;
    Object->initialized = TRUE;
    return E_OK;
}

Std_ReturnType MotorControlAlgorithm_PiReset(
    MotorControlAlgorithm_PiType *Object,
    sint32 IntegratorQ15)
{
    if ((MotorControlAlgorithm_PiObjectValid(Object) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(IntegratorQ15) != TRUE) ||
        (IntegratorQ15 < Object->config.lowerQ15) ||
        (IntegratorQ15 > Object->config.upperQ15))
    {
        return E_NOT_OK;
    }

    Object->integratorQ30 =
        (sint64)IntegratorQ15 * MOTOR_CONTROL_ALGORITHM_Q15_SCALE;
    return E_OK;
}

Std_ReturnType MotorControlAlgorithm_PiStep(
    MotorControlAlgorithm_PiType *Object,
    sint32 ErrorQ15,
    MotorControlAlgorithm_PiResultType *Result)
{
    sint64 lowerQ30;
    sint64 upperQ30;
    sint64 pQ30;
    sint64 deltaIQ30;
    sint64 rawCandidateI;
    sint64 candidateI;
    sint64 candidateU;
    sint64 reportedU;
    boolean holdIntegrator;

    if (Result != NULL_PTR)
    {
        Result->outputQ15 = 0;
        Result->limited = FALSE;
    }
    if ((Result == NULL_PTR) ||
        (MotorControlAlgorithm_PiObjectValid(Object) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(ErrorQ15) != TRUE))
    {
        return E_NOT_OK;
    }

    lowerQ30 = (sint64)Object->config.lowerQ15 *
               MOTOR_CONTROL_ALGORITHM_Q15_SCALE;
    upperQ30 = (sint64)Object->config.upperQ15 *
               MOTOR_CONTROL_ALGORITHM_Q15_SCALE;
    pQ30 = (sint64)Object->config.kpQ15 * (sint64)ErrorQ15;
    deltaIQ30 = (sint64)Object->config.kiQ15 * (sint64)ErrorQ15;
    rawCandidateI = Object->integratorQ30 + deltaIQ30;
    candidateI = rawCandidateI;
    if (candidateI > upperQ30)
    {
        candidateI = upperQ30;
    }
    else if (candidateI < lowerQ30)
    {
        candidateI = lowerQ30;
    }

    candidateU = pQ30 + candidateI;
    holdIntegrator = (((candidateU > upperQ30) && (ErrorQ15 > 0)) ||
                      ((candidateU < lowerQ30) && (ErrorQ15 < 0))) ?
                     TRUE : FALSE;
    if (holdIntegrator == TRUE)
    {
        reportedU = pQ30 + Object->integratorQ30;
    }
    else
    {
        Object->integratorQ30 = candidateI;
        reportedU = candidateU;
    }

    Result->limited = (((rawCandidateI > upperQ30) ||
                        (rawCandidateI < lowerQ30) ||
                        (candidateU > upperQ30) ||
                        (candidateU < lowerQ30))) ? TRUE : FALSE;
    if (reportedU > upperQ30)
    {
        reportedU = upperQ30;
    }
    else if (reportedU < lowerQ30)
    {
        reportedU = lowerQ30;
    }

    Result->outputQ15 =
        (sint32)MotorControlAlgorithm_RoundQ30(reportedU);
    return E_OK;
}

Std_ReturnType MotorControlAlgorithm_Svpwm(
    const MotorControlAlgorithm_AlphaBetaType *AlphaBeta,
    MotorControlAlgorithm_SvpwmResultType *Result)
{
    sint32 va;
    sint32 vb;
    sint32 vc;
    sint32 vmax;
    sint32 vmin;
    sint32 center;
    sint64 xa;
    sint64 xb;
    sint64 xc;

    if (Result != NULL_PTR)
    {
        Result->dutyAQ15 = 0U;
        Result->dutyBQ15 = 0U;
        Result->dutyCQ15 = 0U;
        Result->sector = 0U;
        Result->limited = FALSE;
    }
    if ((Result == NULL_PTR) || (AlphaBeta == NULL_PTR) ||
        (MotorControlAlgorithm_IsQ15(AlphaBeta->alphaQ15) != TRUE) ||
        (MotorControlAlgorithm_IsQ15(AlphaBeta->betaQ15) != TRUE))
    {
        return E_NOT_OK;
    }

    va = AlphaBeta->alphaQ15;
    vb = (sint32)MotorControlAlgorithm_RoundQ30(
        -((sint64)AlphaBeta->alphaQ15 *
          (sint64)MOTOR_CONTROL_ALGORITHM_ONE_HALF_Q15) +
        ((sint64)AlphaBeta->betaQ15 *
         (sint64)MOTOR_CONTROL_ALGORITHM_SQRT3_OVER_2_Q15));
    vc = (sint32)MotorControlAlgorithm_RoundQ30(
        -((sint64)AlphaBeta->alphaQ15 *
          (sint64)MOTOR_CONTROL_ALGORITHM_ONE_HALF_Q15) -
        ((sint64)AlphaBeta->betaQ15 *
         (sint64)MOTOR_CONTROL_ALGORITHM_SQRT3_OVER_2_Q15));

    vmax = va;
    vmin = va;
    if (vb > vmax)
    {
        vmax = vb;
    }
    if (vc > vmax)
    {
        vmax = vc;
    }
    if (vb < vmin)
    {
        vmin = vb;
    }
    if (vc < vmin)
    {
        vmin = vc;
    }

    center = MotorControlAlgorithm_DivideByTwoAway(vmax + vmin);
    xa = (sint64)va - (sint64)center;
    xb = (sint64)vb - (sint64)center;
    xc = (sint64)vc - (sint64)center;
    if (xa > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX)
    {
        xa = MOTOR_CONTROL_ALGORITHM_Q15_MAX;
        Result->limited = TRUE;
    }
    else if (xa < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN)
    {
        xa = MOTOR_CONTROL_ALGORITHM_Q15_MIN;
        Result->limited = TRUE;
    }
    if (xb > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX)
    {
        xb = MOTOR_CONTROL_ALGORITHM_Q15_MAX;
        Result->limited = TRUE;
    }
    else if (xb < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN)
    {
        xb = MOTOR_CONTROL_ALGORITHM_Q15_MIN;
        Result->limited = TRUE;
    }
    if (xc > (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MAX)
    {
        xc = MOTOR_CONTROL_ALGORITHM_Q15_MAX;
        Result->limited = TRUE;
    }
    else if (xc < (sint64)MOTOR_CONTROL_ALGORITHM_Q15_MIN)
    {
        xc = MOTOR_CONTROL_ALGORITHM_Q15_MIN;
        Result->limited = TRUE;
    }

    Result->dutyAQ15 = (uint16)((xa + 32768) >> 1U);
    Result->dutyBQ15 = (uint16)((xb + 32768) >> 1U);
    Result->dutyCQ15 = (uint16)((xc + 32768) >> 1U);

    if ((va >= vb) && (vb >= vc))
    {
        Result->sector = 1U;
    }
    else if ((vb > va) && (va >= vc))
    {
        Result->sector = 2U;
    }
    else if ((vb >= vc) && (vc > va))
    {
        Result->sector = 3U;
    }
    else if ((vc > vb) && (vb > va))
    {
        Result->sector = 4U;
    }
    else if ((vc > va) && (va >= vb))
    {
        Result->sector = 5U;
    }
    else
    {
        Result->sector = 6U;
    }

    return E_OK;
}
