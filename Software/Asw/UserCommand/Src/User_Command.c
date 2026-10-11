#include "User_Command.h"

/* User-editable standalone commissioning profile. Disabled is the baseline. */
#define USER_COMMAND_ENABLED          0U
#define USER_COMMAND_CURRENT_MA       0
#define USER_COMMAND_RUN_DURATION_MS  0U
#define USER_COMMAND_LEASE_MS         0U

/* Safety bounds are policy, not user settings. */
#define USER_COMMAND_MAX_ABS_CURRENT_MA 300
#define USER_COMMAND_MAX_RUN_DURATION_MS 5000U
#define USER_COMMAND_MAX_LEASE_MS        100U

_Static_assert((USER_COMMAND_ENABLED == 0U) ||
               (USER_COMMAND_ENABLED == 1U),
               "USER_COMMAND_ENABLED must be 0U or 1U");
_Static_assert((USER_COMMAND_ENABLED != 0U) ||
               (USER_COMMAND_CURRENT_MA == 0),
               "disabled profile current must be zero");
_Static_assert((USER_COMMAND_ENABLED != 0U) ||
               (USER_COMMAND_RUN_DURATION_MS == 0U),
               "disabled profile duration must be zero");
_Static_assert((USER_COMMAND_ENABLED != 0U) ||
               (USER_COMMAND_LEASE_MS == 0U),
               "disabled profile lease must be zero");
_Static_assert((USER_COMMAND_ENABLED == 0U) ||
               ((USER_COMMAND_CURRENT_MA >=
                 -USER_COMMAND_MAX_ABS_CURRENT_MA) &&
                (USER_COMMAND_CURRENT_MA <=
                 USER_COMMAND_MAX_ABS_CURRENT_MA)),
               "enabled profile current exceeds 300 mA");
_Static_assert((USER_COMMAND_ENABLED == 0U) ||
               ((USER_COMMAND_RUN_DURATION_MS >= 1U) &&
                (USER_COMMAND_RUN_DURATION_MS <=
                 USER_COMMAND_MAX_RUN_DURATION_MS)),
               "enabled profile duration is outside 1..5000 ms");
_Static_assert((USER_COMMAND_ENABLED == 0U) ||
               ((USER_COMMAND_LEASE_MS >= 1U) &&
                (USER_COMMAND_LEASE_MS <= USER_COMMAND_MAX_LEASE_MS)),
               "enabled profile lease is outside 1..100 ms");

#if defined(USER_COMMAND_HOST_TEST_PROFILE_INITIALIZER)
STATIC const UserCommand_ProfileType UserCommand_Profile =
    USER_COMMAND_HOST_TEST_PROFILE_INITIALIZER;
#else
STATIC const UserCommand_ProfileType UserCommand_Profile =
{
    (boolean)USER_COMMAND_ENABLED,
    (sint32)USER_COMMAND_CURRENT_MA,
    (uint32)USER_COMMAND_RUN_DURATION_MS,
    (uint16)USER_COMMAND_LEASE_MS
};
#endif

STATIC boolean UserCommand_ProfileIsValid(
    const UserCommand_ProfileType *Profile)
{
    if ((Profile == NULL_PTR) ||
        ((Profile->enabled != FALSE) && (Profile->enabled != TRUE)))
    {
        return FALSE;
    }

    if (Profile->enabled == FALSE)
    {
        return (boolean)(((Profile->currentMa == 0) &&
                          (Profile->runDurationMs == 0U) &&
                          (Profile->leaseMs == 0U)) ? TRUE : FALSE);
    }

    if ((Profile->currentMa < -USER_COMMAND_MAX_ABS_CURRENT_MA) ||
        (Profile->currentMa > USER_COMMAND_MAX_ABS_CURRENT_MA) ||
        (Profile->runDurationMs == 0U) ||
        (Profile->runDurationMs > USER_COMMAND_MAX_RUN_DURATION_MS) ||
        (Profile->leaseMs == 0U) ||
        (Profile->leaseMs > USER_COMMAND_MAX_LEASE_MS))
    {
        return FALSE;
    }

    return TRUE;
}

Std_ReturnType UserCommand_GetProfile(UserCommand_ProfileType *Profile)
{
    if (Profile == NULL_PTR)
    {
        return E_NOT_OK;
    }

    Profile->enabled = FALSE;
    Profile->currentMa = 0;
    Profile->runDurationMs = 0U;
    Profile->leaseMs = 0U;

    if (UserCommand_ProfileIsValid(&UserCommand_Profile) != TRUE)
    {
        return E_NOT_OK;
    }

    *Profile = UserCommand_Profile;
    return E_OK;
}
