#include <limits.h>
#include <stdio.h>

#include "MotorControlAlgorithm.h"

#include "../../../Software/Bsw/ComplexDrivers/MotorControl/Src/MotorControlAlgorithm.c"

#define TEST_CHECK(condition)                                                \
    do                                                                       \
    {                                                                        \
        if (!(condition))                                                    \
        {                                                                    \
            (void)printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__,           \
                         #condition);                                        \
            return 1;                                                        \
        }                                                                    \
    } while (0)

STATIC sint32 Test_Abs(sint32 value)
{
    return (value < 0) ? -value : value;
}

STATIC sint64 Test_ReferenceRoundQ30(sint64 valueQ30)
{
    return (valueQ30 >= 0) ?
        ((valueQ30 + 16384) >> 15U) :
        -(((-valueQ30) + 16384) >> 15U);
}

STATIC sint32 Test_ReferenceHalfAway(sint32 value)
{
    return (value >= 0) ?
        ((value + 1) >> 1U) : -(((-value) + 1) >> 1U);
}

STATIC uint8 Test_ReferenceSector(sint32 va, sint32 vb, sint32 vc)
{
    if ((va >= vb) && (vb >= vc))
    {
        return 1U;
    }
    if ((vb > va) && (va >= vc))
    {
        return 2U;
    }
    if ((vb >= vc) && (vc > va))
    {
        return 3U;
    }
    if ((vc > vb) && (vb > va))
    {
        return 4U;
    }
    if ((vc > va) && (va >= vb))
    {
        return 5U;
    }
    return 6U;
}

STATIC sint64 Test_ReferenceClipQ15(sint64 value, boolean *limited)
{
    if (value > 32767)
    {
        *limited = TRUE;
        return 32767;
    }
    if (value < -32768)
    {
        *limited = TRUE;
        return -32768;
    }
    return value;
}

STATIC void Test_ReferenceSvpwm(
    sint32 alphaQ15,
    sint32 betaQ15,
    sint32 commonModeQ15,
    MotorControlAlgorithm_SvpwmResultType *result)
{
    sint32 va = alphaQ15 + commonModeQ15;
    sint32 vb = (sint32)Test_ReferenceRoundQ30(
        -((sint64)alphaQ15 * 16384) +
        ((sint64)betaQ15 * 28378)) + commonModeQ15;
    sint32 vc = (sint32)Test_ReferenceRoundQ30(
        -((sint64)alphaQ15 * 16384) -
        ((sint64)betaQ15 * 28378)) + commonModeQ15;
    sint32 vmax = va;
    sint32 vmin = va;
    sint32 center;
    sint64 xa;
    sint64 xb;
    sint64 xc;

    if (vb > vmax) { vmax = vb; }
    if (vc > vmax) { vmax = vc; }
    if (vb < vmin) { vmin = vb; }
    if (vc < vmin) { vmin = vc; }
    center = Test_ReferenceHalfAway(vmax + vmin);
    result->limited = FALSE;
    xa = Test_ReferenceClipQ15(
        (sint64)va - (sint64)center, &result->limited);
    xb = Test_ReferenceClipQ15(
        (sint64)vb - (sint64)center, &result->limited);
    xc = Test_ReferenceClipQ15(
        (sint64)vc - (sint64)center, &result->limited);
    result->dutyAQ15 = (uint16)((xa + 32768) >> 1U);
    result->dutyBQ15 = (uint16)((xb + 32768) >> 1U);
    result->dutyCQ15 = (uint16)((xc + 32768) >> 1U);
    result->sector = Test_ReferenceSector(va, vb, vc);
}

STATIC int Test_TransformsNominalAndBoundaries(void)
{
    static const struct
    {
        sint32 sinQ15;
        sint32 cosQ15;
        uint32 toleranceQ30;
    } rotations[] =
    {
        {0, 32767, 65535U},
        {23170, 23170, 50000U},
        {32767, 0, 65535U},
        {23170, -23170, 50000U},
        {0, -32768, 0U},
        {-23170, -23170, 50000U},
        {-32768, 0, 0U},
        {-23170, 23170, 50000U}
    };
    MotorControlAlgorithm_AlphaBetaType alphaBeta;
    MotorControlAlgorithm_AlphaBetaType roundTrip;
    MotorControlAlgorithm_DqType dq;
    uint32 rotationIndex;

    TEST_CHECK(MotorControlAlgorithm_Clarke(
                   16384, -8192, -8192, 0U, &alphaBeta) == E_OK);
    TEST_CHECK(alphaBeta.alphaQ15 == 16384);
    TEST_CHECK(alphaBeta.betaQ15 == 0);

    TEST_CHECK(MotorControlAlgorithm_Clarke(
                   0, 14189, -14189, 0U, &alphaBeta) == E_OK);
    TEST_CHECK(alphaBeta.alphaQ15 == 0);
    TEST_CHECK(Test_Abs(alphaBeta.betaQ15 - 16384) <= 1);

    alphaBeta.alphaQ15 = 10000;
    alphaBeta.betaQ15 = -5000;
    TEST_CHECK(MotorControlAlgorithm_Park(
                   &alphaBeta, 0, 32767, 65535U, &dq) == E_OK);
    TEST_CHECK(Test_Abs(dq.dQ15 - 10000) <= 1);
    TEST_CHECK(Test_Abs(dq.qQ15 + 5000) <= 1);
    TEST_CHECK(MotorControlAlgorithm_InversePark(
                   &dq, 0, 32767, 65535U, &roundTrip) == E_OK);
    TEST_CHECK(Test_Abs(roundTrip.alphaQ15 - alphaBeta.alphaQ15) <= 2);
    TEST_CHECK(Test_Abs(roundTrip.betaQ15 - alphaBeta.betaQ15) <= 2);

    TEST_CHECK(MotorControlAlgorithm_Park(
                   &alphaBeta, 32767, 0, 65535U, &dq) == E_OK);
    TEST_CHECK(Test_Abs(dq.dQ15 + 5000) <= 1);
    TEST_CHECK(Test_Abs(dq.qQ15 + 10000) <= 1);

    alphaBeta.alphaQ15 = 8000;
    alphaBeta.betaQ15 = -4000;
    TEST_CHECK(MotorControlAlgorithm_Park(
                   &alphaBeta, 23170, 23170, 50000U, &dq) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_InversePark(
                   &dq, 23170, 23170, 50000U, &roundTrip) == E_OK);
    TEST_CHECK(Test_Abs(roundTrip.alphaQ15 - alphaBeta.alphaQ15) <= 3);
    TEST_CHECK(Test_Abs(roundTrip.betaQ15 - alphaBeta.betaQ15) <= 3);

    for (rotationIndex = 0U;
         rotationIndex <
             (uint32)(sizeof(rotations) / sizeof(rotations[0]));
         rotationIndex++)
    {
        alphaBeta.alphaQ15 = 4000;
        alphaBeta.betaQ15 = -2000;
        TEST_CHECK(MotorControlAlgorithm_Park(
                       &alphaBeta,
                       rotations[rotationIndex].sinQ15,
                       rotations[rotationIndex].cosQ15,
                       rotations[rotationIndex].toleranceQ30,
                       &dq) == E_OK);
        TEST_CHECK(MotorControlAlgorithm_InversePark(
                       &dq,
                       rotations[rotationIndex].sinQ15,
                       rotations[rotationIndex].cosQ15,
                       rotations[rotationIndex].toleranceQ30,
                       &roundTrip) == E_OK);
        TEST_CHECK(Test_Abs(roundTrip.alphaQ15 - alphaBeta.alphaQ15) <= 3);
        TEST_CHECK(Test_Abs(roundTrip.betaQ15 - alphaBeta.betaQ15) <= 3);
    }
    return 0;
}

STATIC int Test_TransformRejection(void)
{
    static const struct
    {
        sint32 phaseA;
        sint32 phaseB;
        sint32 phaseC;
        uint32 tolerance;
    } clarkeInvalid[] =
    {
        {32768, 0, 0, 98304U}, {-32769, 0, 0, 98304U},
        {0, 32768, 0, 98304U}, {0, -32769, 0, 98304U},
        {0, 0, 32768, 98304U}, {0, 0, -32769, 98304U},
        {0, 0, 0, 98305U}
    };
    static const struct
    {
        sint32 alpha;
        sint32 beta;
        sint32 sinValue;
        sint32 cosValue;
        uint32 tolerance;
    } parkInvalid[] =
    {
        {32768, 0, 0, 32767, 65535U},
        {-32769, 0, 0, 32767, 65535U},
        {0, 32768, 0, 32767, 65535U},
        {0, -32769, 0, 32767, 65535U},
        {1000, 0, 32768, 0, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, -32769, 0, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, 0, 32768, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, 0, -32769, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, 0, 32767, MOTOR_CONTROL_ALGORITHM_Q30_ONE + 1U},
        {1000, 0, 0, 0, 0U}
    };
    static const struct
    {
        sint32 dValue;
        sint32 qValue;
        sint32 sinValue;
        sint32 cosValue;
        uint32 tolerance;
    } inverseInvalid[] =
    {
        {32768, 0, 0, 32767, 65535U},
        {-32769, 0, 0, 32767, 65535U},
        {0, 32768, 0, 32767, 65535U},
        {0, -32769, 0, 32767, 65535U},
        {1000, 0, 32768, 0, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, -32769, 0, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, 0, 32768, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, 0, -32769, MOTOR_CONTROL_ALGORITHM_Q30_ONE},
        {1000, 0, 0, 32767, MOTOR_CONTROL_ALGORITHM_Q30_ONE + 1U},
        {1000, 0, 0, 0, 0U}
    };
    MotorControlAlgorithm_AlphaBetaType alphaBeta = {123, 456};
    MotorControlAlgorithm_AlphaBetaType inverseResult = {123, 456};
    MotorControlAlgorithm_DqType dq = {123, 456};
    uint32 invalidIndex;

    TEST_CHECK(MotorControlAlgorithm_Clarke(0, 0, 0, 0U, NULL_PTR) ==
               E_NOT_OK);
    for (invalidIndex = 0U;
         invalidIndex <
             (uint32)(sizeof(clarkeInvalid) / sizeof(clarkeInvalid[0]));
         invalidIndex++)
    {
        alphaBeta.alphaQ15 = 123;
        alphaBeta.betaQ15 = 456;
        TEST_CHECK(MotorControlAlgorithm_Clarke(
                       clarkeInvalid[invalidIndex].phaseA,
                       clarkeInvalid[invalidIndex].phaseB,
                       clarkeInvalid[invalidIndex].phaseC,
                       clarkeInvalid[invalidIndex].tolerance,
                       &alphaBeta) == E_NOT_OK);
        TEST_CHECK((alphaBeta.alphaQ15 == 0) &&
                   (alphaBeta.betaQ15 == 0));
    }
    TEST_CHECK(MotorControlAlgorithm_Clarke(
                   100, 100, -199, 0U, &alphaBeta) == E_NOT_OK);
    TEST_CHECK(MotorControlAlgorithm_Clarke(
                   100, 100, -199, 1U, &alphaBeta) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_Clarke(
                   0, 0, 0, 98305U, &alphaBeta) == E_NOT_OK);

    /* Valid zero-sum phases whose Clarke beta is outside Q1.15. */
    TEST_CHECK(MotorControlAlgorithm_Clarke(
                   0, 32767, -32767, 0U, &alphaBeta) == E_NOT_OK);
    TEST_CHECK((alphaBeta.alphaQ15 == 0) && (alphaBeta.betaQ15 == 0));

    alphaBeta.alphaQ15 = 1000;
    alphaBeta.betaQ15 = 2000;
    TEST_CHECK(MotorControlAlgorithm_Park(
                   NULL_PTR, 0, 32767, 65535U, &dq) == E_NOT_OK);
    TEST_CHECK((dq.dQ15 == 0) && (dq.qQ15 == 0));
    TEST_CHECK(MotorControlAlgorithm_Park(
                   &alphaBeta, 0, 32767, 65535U, NULL_PTR) == E_NOT_OK);
    for (invalidIndex = 0U;
         invalidIndex <
             (uint32)(sizeof(parkInvalid) / sizeof(parkInvalid[0]));
         invalidIndex++)
    {
        alphaBeta.alphaQ15 = parkInvalid[invalidIndex].alpha;
        alphaBeta.betaQ15 = parkInvalid[invalidIndex].beta;
        dq.dQ15 = 123;
        dq.qQ15 = 456;
        TEST_CHECK(MotorControlAlgorithm_Park(
                       &alphaBeta,
                       parkInvalid[invalidIndex].sinValue,
                       parkInvalid[invalidIndex].cosValue,
                       parkInvalid[invalidIndex].tolerance,
                       &dq) == E_NOT_OK);
        TEST_CHECK((dq.dQ15 == 0) && (dq.qQ15 == 0));
    }

    /* Unit-norm input with a mathematically unrepresentable Park output. */
    alphaBeta.alphaQ15 = 32767;
    alphaBeta.betaQ15 = 32767;
    TEST_CHECK(MotorControlAlgorithm_Park(
                   &alphaBeta, 23170, 23170, 50000U, &dq) == E_NOT_OK);
    TEST_CHECK((dq.dQ15 == 0) && (dq.qQ15 == 0));

    dq.dQ15 = 1000;
    dq.qQ15 = 2000;
    TEST_CHECK(MotorControlAlgorithm_InversePark(
                   NULL_PTR, 0, 32767, 65535U, &inverseResult) == E_NOT_OK);
    TEST_CHECK((inverseResult.alphaQ15 == 0) &&
               (inverseResult.betaQ15 == 0));
    TEST_CHECK(MotorControlAlgorithm_InversePark(
                   &dq, 0, 32767, 65535U, NULL_PTR) == E_NOT_OK);
    for (invalidIndex = 0U;
         invalidIndex <
             (uint32)(sizeof(inverseInvalid) / sizeof(inverseInvalid[0]));
         invalidIndex++)
    {
        dq.dQ15 = inverseInvalid[invalidIndex].dValue;
        dq.qQ15 = inverseInvalid[invalidIndex].qValue;
        inverseResult.alphaQ15 = 123;
        inverseResult.betaQ15 = 456;
        TEST_CHECK(MotorControlAlgorithm_InversePark(
                       &dq,
                       inverseInvalid[invalidIndex].sinValue,
                       inverseInvalid[invalidIndex].cosValue,
                       inverseInvalid[invalidIndex].tolerance,
                       &inverseResult) == E_NOT_OK);
        TEST_CHECK((inverseResult.alphaQ15 == 0) &&
                   (inverseResult.betaQ15 == 0));
    }

    dq.dQ15 = 32767;
    dq.qQ15 = -32768;
    TEST_CHECK(MotorControlAlgorithm_InversePark(
                   &dq, 23170, 23170, 50000U, &inverseResult) == E_NOT_OK);
    TEST_CHECK((inverseResult.alphaQ15 == 0) &&
               (inverseResult.betaQ15 == 0));
    return 0;
}

STATIC int Test_PiInitializationAndInvalidState(void)
{
    MotorControlAlgorithm_PiConfigType config =
        {16384, 3277, -5000, 20000};
    MotorControlAlgorithm_PiType object;
    MotorControlAlgorithm_PiResultType result = {123, TRUE};
    sint64 savedIntegrator;

    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    TEST_CHECK(object.initialized == TRUE);
    TEST_CHECK(object.integratorQ30 == 0);
    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, 20000) == E_OK);
    TEST_CHECK(object.integratorQ30 ==
               (sint64)20000 * (sint64)32768);
    savedIntegrator = object.integratorQ30;
    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, -5001) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);

    config.lowerQ15 = 1000;
    config.upperQ15 = 2000;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 1500) == E_OK);
    TEST_CHECK(object.integratorQ30 ==
               (sint64)1500 * (sint64)32768);
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_NOT_OK);
    TEST_CHECK(object.initialized == FALSE);
    TEST_CHECK(object.integratorQ30 == 0);
    TEST_CHECK(object.config.kpQ15 == 0);

    config.lowerQ15 = -1000;
    config.upperQ15 = -1000;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, -1000) ==
               E_NOT_OK);
    TEST_CHECK(object.initialized == FALSE);
    config.lowerQ15 = -1000;
    config.upperQ15 = 1000;
    config.kpQ15 = -1;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_NOT_OK);
    TEST_CHECK(MotorControlAlgorithm_PiInit(NULL_PTR, &config, 0) == E_NOT_OK);

    config.kpQ15 = 1000;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    savedIntegrator = object.integratorQ30;
    TEST_CHECK(MotorControlAlgorithm_PiStep(
                   &object, 32768, &result) == E_NOT_OK);
    TEST_CHECK((result.outputQ15 == 0) && (result.limited == FALSE));
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK(MotorControlAlgorithm_PiStep(
                   &object, 0, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    object.integratorQ30 =
        ((sint64)object.config.upperQ15 * (sint64)32768) + 1;
    TEST_CHECK(MotorControlAlgorithm_PiStep(
                   &object, 0, &result) == E_NOT_OK);
    return 0;
}

STATIC int Test_PiActionAndAntiWindup(void)
{
    MotorControlAlgorithm_PiConfigType config =
        {16384, 0, -20000, 20000};
    MotorControlAlgorithm_PiType object;
    MotorControlAlgorithm_PiResultType result;
    sint64 integratorBefore;
    uint32 iteration;

    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 10000, &result) == E_OK);
    TEST_CHECK(result.outputQ15 == 5000);
    TEST_CHECK(result.limited == FALSE);
    TEST_CHECK(object.integratorQ30 == 0);

    config.kpQ15 = 0;
    config.kiQ15 = 3277;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 1000, &result) == E_OK);
    TEST_CHECK(Test_Abs(result.outputQ15 - 100) <= 1);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 1000, &result) == E_OK);
    TEST_CHECK(Test_Abs(result.outputQ15 - 200) <= 1);

    config.kpQ15 = 32767;
    config.kiQ15 = 16384;
    config.lowerQ15 = -10000;
    config.upperQ15 = 10000;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 30000, &result) == E_OK);
    TEST_CHECK((result.outputQ15 == 10000) && (result.limited == TRUE));
    TEST_CHECK(object.integratorQ30 == 0);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, -1000, &result) == E_OK);
    TEST_CHECK(result.outputQ15 < 0);
    TEST_CHECK(object.integratorQ30 < 0);

    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, 0) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, -30000, &result) == E_OK);
    TEST_CHECK((result.outputQ15 == -10000) && (result.limited == TRUE));
    TEST_CHECK(object.integratorQ30 == 0);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 1000, &result) == E_OK);
    TEST_CHECK(result.outputQ15 > 0);
    TEST_CHECK(object.integratorQ30 > 0);

    config.kpQ15 = 32767;
    config.kiQ15 = 0;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 10000, &result) == E_OK);
    TEST_CHECK(result.outputQ15 == 10000);
    TEST_CHECK(result.limited == FALSE);

    config.kpQ15 = 0;
    config.kiQ15 = 32767;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    for (iteration = 0U; iteration < 100000U; iteration++)
    {
        sint32 error = ((iteration & 1U) == 0U) ? 32767 : -32768;
        TEST_CHECK(MotorControlAlgorithm_PiStep(
                       &object, error, &result) == E_OK);
        TEST_CHECK(object.integratorQ30 >=
                   ((sint64)config.lowerQ15 * (sint64)32768));
        TEST_CHECK(object.integratorQ30 <=
                   ((sint64)config.upperQ15 * (sint64)32768));
        TEST_CHECK((result.outputQ15 >= config.lowerQ15) &&
                   (result.outputQ15 <= config.upperQ15));
    }
    integratorBefore = object.integratorQ30;
    object.initialized = FALSE;
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 1, &result) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == integratorBefore);
    return 0;
}

STATIC int Test_PiExactRailEqualityAndZeroError(void)
{
    MotorControlAlgorithm_PiConfigType config =
        {0, 16384, -10000, 10000};
    MotorControlAlgorithm_PiType object;
    MotorControlAlgorithm_PiResultType result;
    sint64 upperQ30 = (sint64)10000 * (sint64)32768;
    sint64 lowerQ30 = (sint64)-10000 * (sint64)32768;
    sint64 savedIntegrator;

    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 0) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 20000, &result) == E_OK);
    TEST_CHECK(object.integratorQ30 == upperQ30);
    TEST_CHECK((result.outputQ15 == 10000) && (result.limited == FALSE));

    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, 0) == E_OK);
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, -20000, &result) == E_OK);
    TEST_CHECK(object.integratorQ30 == lowerQ30);
    TEST_CHECK((result.outputQ15 == -10000) && (result.limited == FALSE));

    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, 1234) == E_OK);
    savedIntegrator = object.integratorQ30;
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 0, &result) == E_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK((result.outputQ15 == 1234) && (result.limited == FALSE));
    return 0;
}

STATIC int Test_PublicInvalidMatrix(void)
{
    static const MotorControlAlgorithm_PiConfigType invalidConfigs[] =
    {
        {-1, 0, -1000, 1000},
        {32768, 0, -1000, 1000},
        {0, -1, -1000, 1000},
        {0, 32768, -1000, 1000},
        {0, 0, -32769, 1000},
        {0, 0, 32768, 1000},
        {0, 0, -1000, -32769},
        {0, 0, -1000, 32768},
        {0, 0, 1000, 1000},
        {0, 0, 1001, 1000}
    };
    static const struct
    {
        sint32 alphaQ15;
        sint32 betaQ15;
    } invalidSvpwm[] =
    {
        {32768, 0}, {-32769, 0},
        {0, 32768}, {0, -32769}
    };
    MotorControlAlgorithm_PiConfigType config = {1000, 1000, -1000, 1000};
    MotorControlAlgorithm_PiType object;
    MotorControlAlgorithm_PiResultType piResult;
    MotorControlAlgorithm_AlphaBetaType alphaBeta;
    MotorControlAlgorithm_SvpwmResultType svpwmResult;
    sint64 savedIntegrator;
    uint32 invalidIndex;

    object.config.kpQ15 = 1;
    object.config.kiQ15 = 2;
    object.config.lowerQ15 = 3;
    object.config.upperQ15 = 4;
    object.integratorQ30 = 5;
    object.initialized = TRUE;
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, NULL_PTR, 0) == E_NOT_OK);
    TEST_CHECK((object.config.kpQ15 == 0) &&
               (object.config.kiQ15 == 0) &&
               (object.config.lowerQ15 == 0) &&
               (object.config.upperQ15 == 0) &&
               (object.integratorQ30 == 0) &&
               (object.initialized == FALSE));
    for (invalidIndex = 0U;
         invalidIndex <
             (uint32)(sizeof(invalidConfigs) / sizeof(invalidConfigs[0]));
         invalidIndex++)
    {
        object.config.kpQ15 = 1;
        object.integratorQ30 = 5;
        object.initialized = TRUE;
        TEST_CHECK(MotorControlAlgorithm_PiInit(
                       &object, &invalidConfigs[invalidIndex], 0) == E_NOT_OK);
        TEST_CHECK((object.config.kpQ15 == 0) &&
                   (object.config.kiQ15 == 0) &&
                   (object.config.lowerQ15 == 0) &&
                   (object.config.upperQ15 == 0) &&
                   (object.integratorQ30 == 0) &&
                   (object.initialized == FALSE));
    }
    TEST_CHECK(MotorControlAlgorithm_PiInit(
                   &object, &config, -32769) == E_NOT_OK);
    TEST_CHECK(object.initialized == FALSE);
    TEST_CHECK(MotorControlAlgorithm_PiInit(
                   &object, &config, 32768) == E_NOT_OK);
    TEST_CHECK(object.initialized == FALSE);
    TEST_CHECK(MotorControlAlgorithm_PiInit(
                   &object, &config, -1001) == E_NOT_OK);
    TEST_CHECK(object.initialized == FALSE);
    TEST_CHECK(MotorControlAlgorithm_PiInit(
                   &object, &config, 1001) == E_NOT_OK);
    TEST_CHECK(object.initialized == FALSE);
    TEST_CHECK(MotorControlAlgorithm_PiInit(NULL_PTR, &config, 0) == E_NOT_OK);

    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 123) == E_OK);
    savedIntegrator = object.integratorQ30;
    piResult.outputQ15 = 123;
    piResult.limited = TRUE;
    TEST_CHECK(MotorControlAlgorithm_PiStep(NULL_PTR, 0, &piResult) ==
               E_NOT_OK);
    TEST_CHECK((piResult.outputQ15 == 0) && (piResult.limited == FALSE));
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 0, NULL_PTR) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK(MotorControlAlgorithm_PiStep(
                   &object, -32769, &piResult) == E_NOT_OK);
    TEST_CHECK((piResult.outputQ15 == 0) && (piResult.limited == FALSE));
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK(MotorControlAlgorithm_PiStep(
                   &object, 32768, &piResult) == E_NOT_OK);
    TEST_CHECK((piResult.outputQ15 == 0) && (piResult.limited == FALSE));
    TEST_CHECK(object.integratorQ30 == savedIntegrator);

    object.integratorQ30 =
        ((sint64)object.config.upperQ15 * (sint64)32768) + 1;
    savedIntegrator = object.integratorQ30;
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 0, &piResult) ==
               E_NOT_OK);
    TEST_CHECK((piResult.outputQ15 == 0) && (piResult.limited == FALSE));
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    object.integratorQ30 =
        ((sint64)object.config.lowerQ15 * (sint64)32768) - 1;
    savedIntegrator = object.integratorQ30;
    TEST_CHECK(MotorControlAlgorithm_PiStep(&object, 0, &piResult) ==
               E_NOT_OK);
    TEST_CHECK((piResult.outputQ15 == 0) && (piResult.limited == FALSE));
    TEST_CHECK(object.integratorQ30 == savedIntegrator);

    object.initialized = FALSE;
    savedIntegrator = object.integratorQ30;
    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, 0) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK(MotorControlAlgorithm_PiReset(NULL_PTR, 0) == E_NOT_OK);
    TEST_CHECK(MotorControlAlgorithm_PiInit(&object, &config, 123) == E_OK);
    savedIntegrator = object.integratorQ30;
    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, -32769) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, 32768) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, -1001) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);
    TEST_CHECK(MotorControlAlgorithm_PiReset(&object, 1001) == E_NOT_OK);
    TEST_CHECK(object.integratorQ30 == savedIntegrator);

    alphaBeta.alphaQ15 = 0;
    alphaBeta.betaQ15 = 0;
    svpwmResult.dutyAQ15 = 1U;
    TEST_CHECK(MotorControlAlgorithm_Svpwm(NULL_PTR, &svpwmResult) ==
               E_NOT_OK);
    TEST_CHECK((svpwmResult.dutyAQ15 == 0U) &&
               (svpwmResult.dutyBQ15 == 0U) &&
               (svpwmResult.dutyCQ15 == 0U) &&
               (svpwmResult.sector == 0U) &&
               (svpwmResult.limited == FALSE));
    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, NULL_PTR) == E_NOT_OK);
    for (invalidIndex = 0U;
         invalidIndex <
             (uint32)(sizeof(invalidSvpwm) / sizeof(invalidSvpwm[0]));
         invalidIndex++)
    {
        alphaBeta.alphaQ15 = invalidSvpwm[invalidIndex].alphaQ15;
        alphaBeta.betaQ15 = invalidSvpwm[invalidIndex].betaQ15;
        svpwmResult.dutyAQ15 = 1U;
        svpwmResult.dutyBQ15 = 2U;
        svpwmResult.dutyCQ15 = 3U;
        svpwmResult.sector = 6U;
        svpwmResult.limited = TRUE;
        TEST_CHECK(MotorControlAlgorithm_Svpwm(
                       &alphaBeta, &svpwmResult) == E_NOT_OK);
        TEST_CHECK((svpwmResult.dutyAQ15 == 0U) &&
                   (svpwmResult.dutyBQ15 == 0U) &&
                   (svpwmResult.dutyCQ15 == 0U) &&
                   (svpwmResult.sector == 0U) &&
                   (svpwmResult.limited == FALSE));
    }
    return 0;
}

STATIC int Test_SvpwmSectorsAndLimits(void)
{
    static const struct
    {
        sint32 alphaQ15;
        sint32 betaQ15;
        uint8 expectedSector;
    } cases[] =
    {
        {20000, 5000, 1U},
        {0, 20000, 2U},
        {-20000, 5000, 3U},
        {-20000, -5000, 4U},
        {0, -20000, 5U},
        {20000, -5000, 6U}
    };
    MotorControlAlgorithm_AlphaBetaType alphaBeta = {0, 0};
    MotorControlAlgorithm_SvpwmResultType result;
    uint32 caseIndex;

    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_OK);
    TEST_CHECK((result.dutyAQ15 == 16384U) &&
               (result.dutyBQ15 == 16384U) &&
               (result.dutyCQ15 == 16384U));
    TEST_CHECK((result.sector == 1U) && (result.limited == FALSE));

    for (caseIndex = 0U;
         caseIndex < (uint32)(sizeof(cases) / sizeof(cases[0]));
         caseIndex++)
    {
        alphaBeta.alphaQ15 = cases[caseIndex].alphaQ15;
        alphaBeta.betaQ15 = cases[caseIndex].betaQ15;
        TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_OK);
        TEST_CHECK(result.sector == cases[caseIndex].expectedSector);
        TEST_CHECK(result.dutyAQ15 <= 32767U);
        TEST_CHECK(result.dutyBQ15 <= 32767U);
        TEST_CHECK(result.dutyCQ15 <= 32767U);
    }

    alphaBeta.alphaQ15 = -20000;
    alphaBeta.betaQ15 = 0;
    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_OK);
    TEST_CHECK(result.sector == 3U); /* Vb == Vc > Va tie. */

    alphaBeta.alphaQ15 = 32767;
    alphaBeta.betaQ15 = 32767;
    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_OK);
    TEST_CHECK(result.limited == TRUE);
    TEST_CHECK(result.dutyAQ15 <= 32767U);
    TEST_CHECK(result.dutyBQ15 <= 32767U);
    TEST_CHECK(result.dutyCQ15 <= 32767U);

    alphaBeta.alphaQ15 = 32768;
    alphaBeta.betaQ15 = 0;
    result.sector = 6U;
    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_NOT_OK);
    TEST_CHECK((result.dutyAQ15 == 0U) && (result.dutyBQ15 == 0U) &&
               (result.dutyCQ15 == 0U) && (result.sector == 0U) &&
               (result.limited == FALSE));
    TEST_CHECK(MotorControlAlgorithm_Svpwm(NULL_PTR, &result) == E_NOT_OK);
    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, NULL_PTR) == E_NOT_OK);
    return 0;
}

STATIC int Test_SvpwmTieTableAndExactEndpoints(void)
{
    static const struct
    {
        sint32 alphaQ15;
        sint32 betaQ15;
        uint8 expectedSector;
    } tieCases[] =
    {
        {1, 2, 1U},       /* Va == Vb > Vc. */
        {-1, -2, 5U},     /* Vc > Va == Vb. */
        {1, -2, 6U},      /* Va == Vc > Vb. */
        {-1, 2, 2U},      /* Vb > Va == Vc. */
        {20000, 0, 1U},   /* Va > Vb == Vc. */
        {-20000, 0, 3U}   /* Vb == Vc > Va. */
    };
    MotorControlAlgorithm_AlphaBetaType alphaBeta;
    MotorControlAlgorithm_SvpwmResultType result;
    uint32 caseIndex;

    for (caseIndex = 0U;
         caseIndex <
             (uint32)(sizeof(tieCases) / sizeof(tieCases[0]));
         caseIndex++)
    {
        alphaBeta.alphaQ15 = tieCases[caseIndex].alphaQ15;
        alphaBeta.betaQ15 = tieCases[caseIndex].betaQ15;
        TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_OK);
        TEST_CHECK(result.sector == tieCases[caseIndex].expectedSector);
    }

    alphaBeta.alphaQ15 = 32767;
    alphaBeta.betaQ15 = 32767;
    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_OK);
    TEST_CHECK(result.limited == TRUE);
    TEST_CHECK(result.dutyAQ15 == 32767U);
    TEST_CHECK(result.dutyCQ15 == 0U);

    alphaBeta.alphaQ15 = -32768;
    alphaBeta.betaQ15 = -32768;
    TEST_CHECK(MotorControlAlgorithm_Svpwm(&alphaBeta, &result) == E_OK);
    TEST_CHECK(result.limited == TRUE);
    TEST_CHECK(result.dutyAQ15 == 0U);
    TEST_CHECK(result.dutyCQ15 == 32767U);
    return 0;
}

STATIC int Test_DeterministicPropertySweep(void)
{
    MotorControlAlgorithm_AlphaBetaType alphaBeta;
    MotorControlAlgorithm_AlphaBetaType negative;
    MotorControlAlgorithm_SvpwmResultType positiveResult;
    MotorControlAlgorithm_SvpwmResultType negativeResult;
    MotorControlAlgorithm_SvpwmResultType referenceResult;
    MotorControlAlgorithm_SvpwmResultType shiftedReferenceResult;
    sint32 alpha;
    sint32 beta;

    for (alpha = -32768; alpha <= 31744; alpha += 1024)
    {
        for (beta = -32768; beta <= 31744; beta += 1024)
        {
            alphaBeta.alphaQ15 = alpha;
            alphaBeta.betaQ15 = beta;
            TEST_CHECK(MotorControlAlgorithm_Svpwm(
                           &alphaBeta, &positiveResult) == E_OK);
            TEST_CHECK((positiveResult.sector >= 1U) &&
                       (positiveResult.sector <= 6U));
            TEST_CHECK(positiveResult.dutyAQ15 <= 32767U);
            TEST_CHECK(positiveResult.dutyBQ15 <= 32767U);
            TEST_CHECK(positiveResult.dutyCQ15 <= 32767U);

            Test_ReferenceSvpwm(alpha, beta, 0, &referenceResult);
            Test_ReferenceSvpwm(alpha, beta, 777, &shiftedReferenceResult);
            TEST_CHECK(positiveResult.dutyAQ15 == referenceResult.dutyAQ15);
            TEST_CHECK(positiveResult.dutyBQ15 == referenceResult.dutyBQ15);
            TEST_CHECK(positiveResult.dutyCQ15 == referenceResult.dutyCQ15);
            TEST_CHECK(positiveResult.sector == referenceResult.sector);
            TEST_CHECK(positiveResult.limited == referenceResult.limited);
            TEST_CHECK(Test_Abs(
                           (sint32)referenceResult.dutyAQ15 -
                           (sint32)shiftedReferenceResult.dutyAQ15) <= 1);
            TEST_CHECK(Test_Abs(
                           (sint32)referenceResult.dutyBQ15 -
                           (sint32)shiftedReferenceResult.dutyBQ15) <= 1);
            TEST_CHECK(Test_Abs(
                           (sint32)referenceResult.dutyCQ15 -
                           (sint32)shiftedReferenceResult.dutyCQ15) <= 1);
            TEST_CHECK(referenceResult.sector == shiftedReferenceResult.sector);
            TEST_CHECK(referenceResult.limited ==
                       shiftedReferenceResult.limited);

            if ((alpha != -32768) && (beta != -32768))
            {
                negative.alphaQ15 = -alpha;
                negative.betaQ15 = -beta;
                TEST_CHECK(MotorControlAlgorithm_Svpwm(
                               &negative, &negativeResult) == E_OK);
                TEST_CHECK(Test_Abs(
                               (sint32)positiveResult.dutyAQ15 +
                               (sint32)negativeResult.dutyAQ15 - 32768) <= 1);
                TEST_CHECK(Test_Abs(
                               (sint32)positiveResult.dutyBQ15 +
                               (sint32)negativeResult.dutyBQ15 - 32768) <= 1);
                TEST_CHECK(Test_Abs(
                               (sint32)positiveResult.dutyCQ15 +
                               (sint32)negativeResult.dutyCQ15 - 32768) <= 1);
            }
        }
    }
    return 0;
}

int main(void)
{
    int failures = 0;

    failures += Test_TransformsNominalAndBoundaries();
    failures += Test_TransformRejection();
    failures += Test_PiInitializationAndInvalidState();
    failures += Test_PiActionAndAntiWindup();
    failures += Test_PiExactRailEqualityAndZeroError();
    failures += Test_PublicInvalidMatrix();
    failures += Test_SvpwmSectorsAndLimits();
    failures += Test_SvpwmTieTableAndExactEndpoints();
    failures += Test_DeterministicPropertySweep();

    if (failures == 0)
    {
        (void)printf("PASS: MotorControl Algorithm Step-8A host matrix\n");
        return 0;
    }

    (void)printf("FAIL: %d MotorControl Algorithm host test group(s)\n",
                 failures);
    return 1;
}
