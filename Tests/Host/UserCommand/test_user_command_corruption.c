#include <stdio.h>

#include "User_Command.h"

#define USER_COMMAND_HOST_TEST_PROFILE_INITIALIZER \
    { TRUE, 301, 1U, 1U }
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

int main(void)
{
    UserCommand_ProfileType destination;

    destination.enabled = TRUE;
    destination.currentMa = 123;
    destination.runDurationMs = 456U;
    destination.leaseMs = 78U;

    TEST_CHECK(UserCommand_GetProfile(&destination) == E_NOT_OK);
    TEST_CHECK(destination.enabled == FALSE);
    TEST_CHECK(destination.currentMa == 0);
    TEST_CHECK(destination.runDurationMs == 0U);
    TEST_CHECK(destination.leaseMs == 0U);

    (void)printf("PASS: UserCommand corrupt-profile accessor matrix\n");
    return 0;
}
