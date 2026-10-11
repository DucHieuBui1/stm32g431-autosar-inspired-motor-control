#include "Wdg.h"

#include "stm32g431xx.h"

#define WDG_KEY_START                         0x0000CCCCU
#define WDG_KEY_ENABLE_WRITE_ACCESS           0x00005555U
#define WDG_KEY_TRIGGER                       0x0000AAAAU

#define WDG_PRESCALER_DIV4                    0U
#define WDG_RELOAD_50MS_AT_32KHZ              399U
#define WDG_WINDOW_DISABLED                   IWDG_WINR_WIN

#define WDG_UPDATE_FLAGS                      \
    (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)

#define WDG_UPDATE_TIMEOUT_ITERATIONS         100000U

STATIC Std_ReturnType Wdg_WaitForRegisterUpdate(void)
{
    uint32 iterationsRemaining = WDG_UPDATE_TIMEOUT_ITERATIONS;

    while (iterationsRemaining > 0U)
    {
        if ((IWDG->SR & WDG_UPDATE_FLAGS) == 0U)
        {
            return E_OK;
        }

        iterationsRemaining--;
    }

    return E_NOT_OK;
}

STATIC boolean Wdg_IsConfigurationValid(void)
{
    if (((IWDG->PR & IWDG_PR_PR) == WDG_PRESCALER_DIV4) &&
        ((IWDG->RLR & IWDG_RLR_RL) == WDG_RELOAD_50MS_AT_32KHZ) &&
        ((IWDG->WINR & IWDG_WINR_WIN) == WDG_WINDOW_DISABLED) &&
        ((IWDG->SR & WDG_UPDATE_FLAGS) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

Std_ReturnType Wdg_Start(void)
{
    if ((FLASH->OPTR & FLASH_OPTR_IWDG_SW) == 0U)
    {
        return E_NOT_OK;
    }

    IWDG->KR = WDG_KEY_START;
    IWDG->KR = WDG_KEY_ENABLE_WRITE_ACCESS;

    IWDG->PR = WDG_PRESCALER_DIV4;
    IWDG->RLR = WDG_RELOAD_50MS_AT_32KHZ;

    if (Wdg_WaitForRegisterUpdate() != E_OK)
    {
        return E_NOT_OK;
    }

    if ((IWDG->WINR & IWDG_WINR_WIN) != WDG_WINDOW_DISABLED)
    {
        /* Writing WINR also reloads the counter. */
        IWDG->WINR = WDG_WINDOW_DISABLED;

        if (Wdg_WaitForRegisterUpdate() != E_OK)
        {
            return E_NOT_OK;
        }
    }
    else
    {
        IWDG->KR = WDG_KEY_TRIGGER;
    }

    if (Wdg_IsConfigurationValid() != TRUE)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

void Wdg_Trigger(void)
{
    IWDG->KR = WDG_KEY_TRIGGER;
}
