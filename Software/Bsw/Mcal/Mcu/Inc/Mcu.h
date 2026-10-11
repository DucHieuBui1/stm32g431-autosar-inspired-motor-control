#ifndef MCU_H
#define MCU_H

#include "Std_Types.h"

typedef struct
{
    uint32 rawDbgmcuIdcode;
    uint32 rawResetFlags;
    uint16 deviceId;
    uint16 revisionId;
    uint32 bootIdentity;
} Mcu_StartupInfoType;

#define MCU_CLOCK_FAULT_EVENT_NONE            0x00000000U
#define MCU_CLOCK_FAULT_EVENT_CSS             0x00000001U

/**
 * @brief Capture the immutable reset/silicon record and validate the reviewed
 *        STM32G431 Rev X profile.
 *
 * Reset flags are captured before being cleared exactly once. Device `0x468`
 * with revision ID `0x2003` is the only currently reviewed profile; any other
 * value prevents boot progression beyond the established early-safe state.
 *
 * @pre MotorControlCdd_EarlySafeInit returned E_OK.
 * @pre Called once before full clock configuration, IWDG and OS startup.
 *
 * @param StartupInfo Destination for the captured immutable startup record.
 * @return E_OK when the record is captured, reset flags clear successfully,
 *         and the device/revision matches the reviewed profile.
 * @return E_NOT_OK for a null pointer, repeated call, reset-flag clear
 *         failure, or unsupported device/revision.
 */
Std_ReturnType Mcu_StartupValidationInit(Mcu_StartupInfoType *StartupInfo);

/**
 * @brief Enable and read back only the GPIOB and TIM1 peripheral clock gates
 *        required by the first power-stage safe action.
 *
 * @pre Reset HSI is active and full clock initialization has not started.
 * @pre No interrupt or module initialization may progress concurrently.
 *
 * @return E_OK when both clock gates read back enabled.
 * @return E_NOT_OK when a required clock gate does not read back enabled; the
 *         caller shall keep boot in the safe failure path.
 */
Std_ReturnType Mcu_EarlyClockGateInit(void);

/**
 * @brief Establish the fixed 170 MHz production clock baseline.
 *
 * The sequence configures Voltage Range 1 Boost, Flash latency/cache/prefetch,
 * the 24 MHz HSE, PLLR at 170 MHz, PLLP at 42.5 MHz, the ADC12 kernel-clock
 * source, bus prescalers and HSE CSS. Every hardware transition has a bounded
 * wait and the complete configuration is read back before success is returned.
 *
 * @pre Mcu_EarlyClockGateInit and MotorControlCdd_EarlySafeInit have passed.
 * @pre Called once from the reset HSI baseline with HSE and PLL disabled.
 * @pre Interrupts, watchdog, OS and RTE are inactive.
 *
 * @return E_OK when the complete clock baseline reads back correctly.
 * @return E_NOT_OK on a precondition mismatch, timeout or readback mismatch;
 *         the caller shall keep the motor outputs safe and stop boot.
 */
Std_ReturnType Mcu_ClockBaselineInit(void);

/**
 * @brief Enable the GPIOA/GPIOC/ADC12 clocks and reset only the shared ADC12
 *        block before either ADC owner initializes its instance.
 *
 * GPIOA and GPIOC are clock-enabled but never whole-port reset. The service
 * verifies the production ADC12 kernel selection, requires ADC1/ADC2 inactive,
 * pulses only ADC12RST, and reads back the released/reset state.
 *
 * @pre Mcu_ClockBaselineInit and Wdg_Start returned E_OK.
 * @pre ADC1 and ADC2 are disabled and no ADC owner has configured registers.
 * @pre Called exactly once before Adc_Init and MotorControlCdd full init.
 *
 * @return E_OK when all gates, reset and kernel-source checks pass.
 * @return E_NOT_OK on a repeated call or any precondition/readback failure;
 *         failure is latched until reset and the caller shall stop boot.
 */
Std_ReturnType Mcu_Adc12PeripheralClockInit(void);

/**
 * @brief Enable and reset only the Mcu-owned USART2 clock boundary.
 *
 * GPIOA is clock-enabled but never whole-port reset. USART2 uses PCLK1 and
 * must still be disabled when this one-shot service runs.
 *
 * @return E_OK when GPIOA/USART2 clocks, reset release and kernel source read
 *         back correctly; E_NOT_OK on repeat or precondition failure.
 */
Std_ReturnType Mcu_SerialPeripheralClockInit(void);

/**
 * @brief Read the producer-owned clock-fault event word.
 *
 * The NMI path release-publishes CSS faults after emergency-off and RCC flag
 * clearing. Critical fault bits are reset-latched and are not cleared by this
 * accessor.
 *
 * @return Current clock-fault event bits, acquired atomically.
 */
uint32 Mcu_GetClockFaultEvents(void);

#endif /* MCU_H */
