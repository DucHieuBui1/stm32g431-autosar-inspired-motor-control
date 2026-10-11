#ifndef ADC_H
#define ADC_H

#include "Std_Types.h"

typedef enum
{
    ADC_SAFETY_GROUP_IDLE = 0,
    ADC_SAFETY_GROUP_PENDING,
    ADC_SAFETY_GROUP_COMPLETED,
    ADC_SAFETY_GROUP_FAILED,
    ADC_SAFETY_GROUP_INVALID_ARGUMENT
} Adc_SafetyGroupStatusType;

typedef struct
{
    uint16 vbusRaw;
    uint16 temperatureRaw;
    uint32 acquisitionSequence;
} Adc_SafetyGroupResultType;

/**
 * @brief Initialize the ADC12 common block, ADC2, PA0 and PC4 once.
 *
 * @pre Mcu_Adc12PeripheralClockInit has completed successfully.
 * @pre The watchdog is running, motor outputs remain early-safe, and the OS
 *      and full MotorControlCdd initialization have not started.
 *
 * @return E_OK after bounded calibration/enable and complete readback.
 * @return E_NOT_OK on a repeated call, precondition failure, timeout, or
 *         configuration mismatch. A failed attempt remains latched until reset.
 */
Std_ReturnType Adc_Init(void);

/**
 * @brief Check the frozen ADC12 common configuration without writing it.
 *
 * @return E_OK only after successful initialization while the complete owned
 *         common configuration still matches the reviewed value.
 */
Std_ReturnType Adc_CheckCommonConfiguration(void);

/**
 * @brief Start the fixed three-rank ADC2 injected safety group.
 *
 * @return E_OK when a new group was started from idle/completed state.
 * @return E_NOT_OK before initialization, while pending, after failure, or
 *         when a hardware/configuration precondition is invalid.
 */
Std_ReturnType Adc_StartSafetyGroup(void);

/**
 * @brief Poll the ADC2 safety group without waiting.
 *
 * Rank one is deliberately discarded. On a valid completion, rank two is
 * published as VBUS and rank three as temperature.
 *
 * @param Result Destination updated only for a completed cached/new result.
 * @return Current group state or ADC_SAFETY_GROUP_INVALID_ARGUMENT for a null
 *         destination. Null polling does not consume a hardware completion.
 */
Adc_SafetyGroupStatusType Adc_PollSafetyGroup(
    Adc_SafetyGroupResultType *Result);

#endif /* ADC_H */
