#ifndef WDG_H
#define WDG_H

#include "Std_Types.h"

/**
 * @brief Start IWDG with the fixed V1 supervision configuration.
 *
 * The fixed configuration uses LSI, prescaler /4, reload 399 and no window.
 * It yields 50 ms at 32 kHz and 47.06 ms to 54.24 ms over the bounded
 * 34 kHz to 29.5 kHz LSI range.
 *
 * @pre Early-safe outputs and Mcu_ClockBaselineInit have passed.
 * @pre The IWDG_SW option byte selects software start.
 * @pre Called once by the boot refresh authority.
 *
 * @return E_OK when the configuration update completes and reads back.
 * @return E_NOT_OK when the update times out or a readback mismatches. Once
 *         started, IWDG cannot be stopped except by reset.
 */
Std_ReturnType Wdg_Start(void);

/**
 * @brief Reload the running IWDG counter.
 *
 * This low-level service does not decide whether refresh is permitted. Only
 * the current supervision authority may call it after a valid checkpoint.
 *
 * @pre Wdg_Start returned E_OK and the caller owns refresh authority.
 */
void Wdg_Trigger(void);

#endif /* WDG_H */
