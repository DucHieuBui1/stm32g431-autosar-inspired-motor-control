#include <stdio.h>

#include "User_Command.h"

#include "../../../Software/Asw/UserCommand/Src/User_Command.c"

#define TEST_CHECK(condition)                                           \
    do                                                                  \
    {                                                                   \
        if (!(condition))                                               \
        {                                                               \
            (void)printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__,      \
                         #condition);                                   \
            return 1;                                                   \
        }                                                               \
    } while (0)

STATIC UserCommand_ProfileType Test_EnabledProfile(void)
{
    UserCommand_ProfileType profile;

    profile.enabled = TRUE;
    profile.currentMa = 0;
    profile.runDurationMs = 1U;
    profile.leaseMs = 1U;
    return profile;
}

STATIC int Test_DefaultProfileIsDisabled(void)
{
    UserCommand_ProfileType profile;

    profile.enabled = TRUE;
    profile.currentMa = 123;
    profile.runDurationMs = 456U;
    profile.leaseMs = 78U;

    TEST_CHECK(UserCommand_GetProfile(&profile) == E_OK);
    TEST_CHECK(profile.enabled == FALSE);
    TEST_CHECK(profile.currentMa == 0);
    TEST_CHECK(profile.runDurationMs == 0U);
    TEST_CHECK(profile.leaseMs == 0U);
    TEST_CHECK(UserCommand_GetProfile(NULL_PTR) == E_NOT_OK);
    return 0;
}

STATIC int Test_EnabledBoundaryProfiles(void)
{
    UserCommand_ProfileType profile = Test_EnabledProfile();

    profile.currentMa = -300;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == TRUE);
    profile.currentMa = 300;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == TRUE);
    profile.currentMa = 0;
    profile.runDurationMs = 5000U;
    profile.leaseMs = 100U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == TRUE);
    return 0;
}

STATIC int Test_AdjacentInvalidProfiles(void)
{
    UserCommand_ProfileType profile = Test_EnabledProfile();

    profile.currentMa = -301;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    profile.currentMa = 301;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    profile.currentMa = 0;
    profile.runDurationMs = 0U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    profile.runDurationMs = 5001U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    profile.runDurationMs = 1U;
    profile.leaseMs = 0U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    profile.leaseMs = 101U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    return 0;
}

STATIC int Test_FailClosedRuntimeValidation(void)
{
    UserCommand_ProfileType profile = Test_EnabledProfile();

    TEST_CHECK(UserCommand_ProfileIsValid(NULL_PTR) == FALSE);
    profile.enabled = (boolean)2U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);

    profile.enabled = FALSE;
    profile.currentMa = 1;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    profile.currentMa = 0;
    profile.runDurationMs = 1U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    profile.runDurationMs = 0U;
    profile.leaseMs = 1U;
    TEST_CHECK(UserCommand_ProfileIsValid(&profile) == FALSE);
    return 0;
}

int main(void)
{
    if ((Test_DefaultProfileIsDisabled() != 0) ||
        (Test_EnabledBoundaryProfiles() != 0) ||
        (Test_AdjacentInvalidProfiles() != 0) ||
        (Test_FailClosedRuntimeValidation() != 0))
    {
        return 1;
    }

    (void)printf("PASS: UserCommand SC-0 host matrix\n");
    return 0;
}
