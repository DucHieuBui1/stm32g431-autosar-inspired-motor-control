#ifndef ECUM_H
#define ECUM_H

/**
 * @brief Execute the earliest production boot checkpoint.
 *
 * The current checkpoints enable the minimal RCC gates, establish the motor
 * power-stage safe state, capture reset/silicon information, validate the
 * reviewed Rev X profile, configure/read back the complete 170 MHz clock
 * baseline, start IWDG and initialize the static OS configuration. It returns
 * only after all pre-StartOS checkpoints have passed; an initialization
 * failure enters the non-returning safe-failure path.
 *
 * @pre Called once from main after C runtime initialization.
 * @pre Interrupts, watchdog, OS and RTE are inactive.
 *
 * @post PB5 and PB13-PB15 are confirmed low, TIM1 MOE is confirmed clear,
 *       the Boost/HSE/PLL/Flash/CSS clock baseline is active, IWDG is running,
 *       and the OS is initialized with SysTick still disabled.
 */
void EcuM_Init(void);

/** @brief Start the configured OS. This service does not return. */
void EcuM_Start(void);

/**
 * @brief Validate the first OS checkpoint, hand refresh authority to WdgM,
 *        then dispatch the 10 ms runtime supervision function.
 *
 * Called only by the configured 10 ms extended supervision task.
 */
void EcuM_BootSupervisorMainFunction(void);

#endif /* ECUM_H */
