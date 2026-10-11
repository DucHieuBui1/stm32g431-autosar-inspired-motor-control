#include "Adc.h"

#include "stm32g431xx.h"

#define ADC_REQUIRED_RCC_GATES               \
    (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_ADC12EN)

#define ADC_GPIO_PA0_MODE_MASK               (3UL << 0U)
#define ADC_GPIO_PC4_MODE_MASK               (3UL << 8U)
#define ADC_GPIO_ANALOG_MODE_PA0             (3UL << 0U)
#define ADC_GPIO_ANALOG_MODE_PC4             (3UL << 8U)

#define ADC_COMMON_OWNED_MASK                \
    (ADC_CCR_DUAL | ADC_CCR_DELAY | ADC_CCR_DMACFG | ADC_CCR_MDMA | \
     ADC_CCR_CKMODE | ADC_CCR_PRESC | ADC_CCR_VREFEN | \
     ADC_CCR_VSENSESEL | ADC_CCR_VBATSEL)
#define ADC_COMMON_EXPECTED_VALUE             0U

#define ADC_ACTIVITY_MASK                    \
    (ADC_CR_ADDIS | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP | \
     ADC_CR_JADSTP | ADC_CR_ADCAL)

#define ADC_CFGR_EXPECTED_VALUE               ADC_CFGR_JQDIS

#define ADC_SAMPLE_47_5_CYCLES                4U
#define ADC_SMPR1_SAFETY_MASK                \
    (ADC_SMPR1_SMP1 | ADC_SMPR1_SMP5)
#define ADC_SMPR1_SAFETY_VALUE               \
    ((ADC_SAMPLE_47_5_CYCLES << ADC_SMPR1_SMP1_Pos) | \
     (ADC_SAMPLE_47_5_CYCLES << ADC_SMPR1_SMP5_Pos))

#define ADC_JSQR_SAFETY_MASK                 \
    (ADC_JSQR_JL | ADC_JSQR_JEXTSEL | ADC_JSQR_JEXTEN | \
     ADC_JSQR_JSQ1 | ADC_JSQR_JSQ2 | ADC_JSQR_JSQ3 | ADC_JSQR_JSQ4)
#define ADC_JSQR_SAFETY_VALUE                \
    ((2UL << ADC_JSQR_JL_Pos) | \
     (1UL << ADC_JSQR_JSQ1_Pos) | \
     (1UL << ADC_JSQR_JSQ2_Pos) | \
     (5UL << ADC_JSQR_JSQ3_Pos))

#define ADC_SAFETY_CHANNEL_DIFSEL_MASK       \
    (ADC_DIFSEL_DIFSEL_1 | ADC_DIFSEL_DIFSEL_5)

#define ADC_STATUS_CLEAR_AT_INIT             \
    (ADC_ISR_ADRDY | ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOS | \
     ADC_ISR_OVR | ADC_ISR_JEOC | ADC_ISR_JEOS | ADC_ISR_AWD1 | \
     ADC_ISR_AWD2 | ADC_ISR_AWD3 | ADC_ISR_JQOVF)

#define ADC_REGULATOR_DELAY_HCLK_CYCLES      3400U
#define ADC_POST_CALIBRATION_HCLK_CYCLES     16U
#define ADC_CALIBRATION_TIMEOUT_OBSERVATIONS 100000U
#define ADC_READY_TIMEOUT_OBSERVATIONS       100000U
#define ADC_RAW_12BIT_MASK                   0x0FFFU

#define ADC_TEST_EVENT_GPIO_CONFIGURED       1U
#define ADC_TEST_EVENT_COMMON_CONFIGURED     2U
#define ADC_TEST_EVENT_FINAL_READBACK        3U
#define ADC_TEST_WAIT_CALIBRATION            1U
#define ADC_TEST_WAIT_READY                  2U

#ifndef ADC_TEST_EVENT_HOOK
#define ADC_TEST_EVENT_HOOK(event)           ((void)(event))
#endif

#ifndef ADC_TEST_WAIT_HOOK
#define ADC_TEST_WAIT_HOOK(stage, observation) \
    do                                           \
    {                                            \
        (void)(stage);                           \
        (void)(observation);                     \
    } while (0)
#endif

#ifndef ADC_TEST_STATUS_CLEAR_HOOK
#define ADC_TEST_STATUS_CLEAR_HOOK(mask)      ((void)(mask))
#endif

#ifndef ADC_READ_INJECTED_DATA
#define ADC_READ_INJECTED_DATA(registerValue, rank) \
    ((void)(rank), (registerValue))
#endif

typedef enum
{
    ADC_INTERNAL_UNINITIALIZED = 0,
    ADC_INTERNAL_INITIALIZING,
    ADC_INTERNAL_IDLE,
    ADC_INTERNAL_PENDING,
    ADC_INTERNAL_COMPLETED,
    ADC_INTERNAL_FAILED
} Adc_InternalStateType;

STATIC Adc_InternalStateType Adc_State = ADC_INTERNAL_UNINITIALIZED;
STATIC boolean Adc_CommonFrozen = FALSE;
STATIC Adc_SafetyGroupResultType Adc_CachedResult;
STATIC uint32 Adc_AcquisitionSequence = 0U;

STATIC void Adc_ClearStatus(uint32 mask)
{
    ADC2->ISR = mask;
    ADC_TEST_STATUS_CLEAR_HOOK(mask);
}

STATIC void Adc_DelayAtLeastHclkCycles(uint32 cycleCount)
{
    volatile uint32 cyclesRemaining = cycleCount;

    while (cyclesRemaining > 0U)
    {
        __NOP();
        cyclesRemaining--;
    }
}

STATIC boolean Adc_AreRccPreconditionsValid(void)
{
    if (((RCC->AHB2ENR & ADC_REQUIRED_RCC_GATES) == ADC_REQUIRED_RCC_GATES) &&
        ((RCC->CCIPR & RCC_CCIPR_ADC12SEL) == RCC_CCIPR_ADC12SEL_0) &&
        ((RCC->AHB2RSTR & RCC_AHB2RSTR_ADC12RST) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean Adc_IsCommonConfigurationValid(void)
{
    if ((Adc_CommonFrozen == TRUE) &&
        ((ADC12_COMMON->CCR & ADC_COMMON_OWNED_MASK) ==
         ADC_COMMON_EXPECTED_VALUE))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean Adc_IsGpioConfigurationValid(void)
{
    if (((GPIOA->MODER & ADC_GPIO_PA0_MODE_MASK) ==
         ADC_GPIO_ANALOG_MODE_PA0) &&
        ((GPIOA->PUPDR & ADC_GPIO_PA0_MODE_MASK) == 0U) &&
        ((GPIOC->MODER & ADC_GPIO_PC4_MODE_MASK) ==
         ADC_GPIO_ANALOG_MODE_PC4) &&
        ((GPIOC->PUPDR & ADC_GPIO_PC4_MODE_MASK) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean Adc_IsInstanceConfigurationValid(void)
{
    if ((ADC2->IER == 0U) &&
        (ADC2->CFGR == ADC_CFGR_EXPECTED_VALUE) &&
        (ADC2->CFGR2 == 0U) &&
        ((ADC2->SMPR1 & ADC_SMPR1_SAFETY_MASK) ==
         ADC_SMPR1_SAFETY_VALUE) &&
        ((ADC2->JSQR & ADC_JSQR_SAFETY_MASK) ==
         ADC_JSQR_SAFETY_VALUE) &&
        ((ADC2->DIFSEL & ADC_SAFETY_CHANNEL_DIFSEL_MASK) == 0U) &&
        ((ADC2->CR & ADC_CR_ADEN) != 0U) &&
        ((ADC2->CR & ADC_CR_ADVREGEN) != 0U) &&
        ((ADC2->CR & ADC_CR_DEEPPWD) == 0U) &&
        ((ADC2->CR & (ADC_CR_ADDIS | ADC_CR_ADSTP | ADC_CR_JADSTP |
                      ADC_CR_ADCAL | ADC_CR_ADCALDIF)) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean Adc_IsCompleteConfigurationValid(void)
{
    if ((Adc_AreRccPreconditionsValid() == TRUE) &&
        (Adc_IsGpioConfigurationValid() == TRUE) &&
        (Adc_IsCommonConfigurationValid() == TRUE) &&
        (Adc_IsInstanceConfigurationValid() == TRUE))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC Std_ReturnType Adc_WaitForCalibration(void)
{
    uint32 observation;

    for (observation = 0U;
         observation < ADC_CALIBRATION_TIMEOUT_OBSERVATIONS;
         observation++)
    {
        ADC_TEST_WAIT_HOOK(ADC_TEST_WAIT_CALIBRATION, observation);
        if ((ADC2->CR & ADC_CR_ADCAL) == 0U)
        {
            return E_OK;
        }
    }

    return E_NOT_OK;
}

STATIC Std_ReturnType Adc_WaitForReady(void)
{
    uint32 observation;

    for (observation = 0U;
         observation < ADC_READY_TIMEOUT_OBSERVATIONS;
         observation++)
    {
        ADC_TEST_WAIT_HOOK(ADC_TEST_WAIT_READY, observation);
        if ((ADC2->ISR & ADC_ISR_ADRDY) != 0U)
        {
            return E_OK;
        }
    }

    return E_NOT_OK;
}

STATIC void Adc_ConfigureSafetyPins(void)
{
    GPIOA->MODER = ((GPIOA->MODER & ~ADC_GPIO_PA0_MODE_MASK) |
                    ADC_GPIO_ANALOG_MODE_PA0);
    GPIOA->PUPDR &= ~ADC_GPIO_PA0_MODE_MASK;

    GPIOC->MODER = ((GPIOC->MODER & ~ADC_GPIO_PC4_MODE_MASK) |
                    ADC_GPIO_ANALOG_MODE_PC4);
    GPIOC->PUPDR &= ~ADC_GPIO_PC4_MODE_MASK;
}

STATIC void Adc_ConfigureCommon(void)
{
    ADC12_COMMON->CCR =
        ((ADC12_COMMON->CCR & ~ADC_COMMON_OWNED_MASK) |
         ADC_COMMON_EXPECTED_VALUE);
}

STATIC void Adc_ConfigureInstance(void)
{
    ADC2->IER = 0U;
    ADC2->CFGR = ADC_CFGR_EXPECTED_VALUE;
    ADC2->CFGR2 = 0U;
    ADC2->SMPR1 = ((ADC2->SMPR1 & ~ADC_SMPR1_SAFETY_MASK) |
                   ADC_SMPR1_SAFETY_VALUE);
    ADC2->JSQR = ((ADC2->JSQR & ~ADC_JSQR_SAFETY_MASK) |
                  ADC_JSQR_SAFETY_VALUE);
    ADC2->DIFSEL &= ~ADC_SAFETY_CHANNEL_DIFSEL_MASK;
}

Std_ReturnType Adc_Init(void)
{
    if (Adc_State != ADC_INTERNAL_UNINITIALIZED)
    {
        return E_NOT_OK;
    }
    Adc_State = ADC_INTERNAL_INITIALIZING;

    if ((Adc_AreRccPreconditionsValid() != TRUE) ||
        ((ADC2->CR & (ADC_CR_ADEN | ADC_ACTIVITY_MASK)) != 0U))
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }

    Adc_ConfigureSafetyPins();
    ADC_TEST_EVENT_HOOK(ADC_TEST_EVENT_GPIO_CONFIGURED);
    if (Adc_IsGpioConfigurationValid() != TRUE)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }

    Adc_ConfigureCommon();
    ADC_TEST_EVENT_HOOK(ADC_TEST_EVENT_COMMON_CONFIGURED);
    if ((ADC12_COMMON->CCR & ADC_COMMON_OWNED_MASK) !=
        ADC_COMMON_EXPECTED_VALUE)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }
    Adc_CommonFrozen = TRUE;

    ADC2->CR &= ~ADC_CR_DEEPPWD;
    ADC2->CR |= ADC_CR_ADVREGEN;
    Adc_DelayAtLeastHclkCycles(ADC_REGULATOR_DELAY_HCLK_CYCLES);

    ADC2->CR &= ~ADC_CR_ADCALDIF;
    ADC2->CR |= ADC_CR_ADCAL;
    if (Adc_WaitForCalibration() != E_OK)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }

    Adc_DelayAtLeastHclkCycles(ADC_POST_CALIBRATION_HCLK_CYCLES);
    Adc_ConfigureInstance();
    Adc_ClearStatus(ADC_STATUS_CLEAR_AT_INIT);

    ADC2->CR |= ADC_CR_ADEN;
    if (Adc_WaitForReady() != E_OK)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }
    Adc_ClearStatus(ADC_ISR_ADRDY);

    ADC_TEST_EVENT_HOOK(ADC_TEST_EVENT_FINAL_READBACK);
    if (Adc_IsCompleteConfigurationValid() != TRUE)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }

    Adc_State = ADC_INTERNAL_IDLE;
    return E_OK;
}

Std_ReturnType Adc_CheckCommonConfiguration(void)
{
    if ((Adc_State != ADC_INTERNAL_UNINITIALIZED) &&
        (Adc_State != ADC_INTERNAL_INITIALIZING) &&
        (Adc_State != ADC_INTERNAL_FAILED) &&
        (Adc_IsCommonConfigurationValid() == TRUE))
    {
        return E_OK;
    }

    return E_NOT_OK;
}

Std_ReturnType Adc_StartSafetyGroup(void)
{
    if (Adc_State == ADC_INTERNAL_PENDING)
    {
        return E_NOT_OK;
    }

    if (Adc_State == ADC_INTERNAL_UNINITIALIZED)
    {
        return E_NOT_OK;
    }

    if ((Adc_State != ADC_INTERNAL_IDLE) &&
        (Adc_State != ADC_INTERNAL_COMPLETED))
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }

    if ((Adc_IsCompleteConfigurationValid() != TRUE) ||
        ((ADC2->CR & ADC_CR_JADSTART) != 0U) ||
        ((ADC2->ISR & (ADC_ISR_JEOS | ADC_ISR_JQOVF)) != 0U))
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return E_NOT_OK;
    }

    Adc_State = ADC_INTERNAL_PENDING;
    ADC2->CR |= ADC_CR_JADSTART;
    return E_OK;
}

Adc_SafetyGroupStatusType Adc_PollSafetyGroup(
    Adc_SafetyGroupResultType *Result)
{
    volatile uint32 discardedRank;
    uint32 vbusRank;
    uint32 temperatureRank;

    if (Result == NULL_PTR)
    {
        return ADC_SAFETY_GROUP_INVALID_ARGUMENT;
    }

    if (Adc_State == ADC_INTERNAL_IDLE)
    {
        if (Adc_IsCompleteConfigurationValid() != TRUE)
        {
            Adc_State = ADC_INTERNAL_FAILED;
            return ADC_SAFETY_GROUP_FAILED;
        }
        return ADC_SAFETY_GROUP_IDLE;
    }

    if (Adc_State == ADC_INTERNAL_COMPLETED)
    {
        if (Adc_IsCompleteConfigurationValid() != TRUE)
        {
            Adc_State = ADC_INTERNAL_FAILED;
            return ADC_SAFETY_GROUP_FAILED;
        }
        *Result = Adc_CachedResult;
        return ADC_SAFETY_GROUP_COMPLETED;
    }

    if (Adc_State == ADC_INTERNAL_FAILED)
    {
        return ADC_SAFETY_GROUP_FAILED;
    }

    if (Adc_State == ADC_INTERNAL_UNINITIALIZED)
    {
        return ADC_SAFETY_GROUP_FAILED;
    }

    if (Adc_State != ADC_INTERNAL_PENDING)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return ADC_SAFETY_GROUP_FAILED;
    }

    if ((ADC2->ISR & ADC_ISR_JQOVF) != 0U)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return ADC_SAFETY_GROUP_FAILED;
    }

    if (Adc_IsCompleteConfigurationValid() != TRUE)
    {
        Adc_State = ADC_INTERNAL_FAILED;
        return ADC_SAFETY_GROUP_FAILED;
    }

    if ((ADC2->ISR & ADC_ISR_JEOS) == 0U)
    {
        return ADC_SAFETY_GROUP_PENDING;
    }

    discardedRank = ADC_READ_INJECTED_DATA(ADC2->JDR1, 1U);
    vbusRank = ADC_READ_INJECTED_DATA(ADC2->JDR2, 2U);
    temperatureRank = ADC_READ_INJECTED_DATA(ADC2->JDR3, 3U);
    (void)discardedRank;

    Adc_AcquisitionSequence++;
    Adc_CachedResult.vbusRaw = (uint16)(vbusRank & ADC_RAW_12BIT_MASK);
    Adc_CachedResult.temperatureRaw =
        (uint16)(temperatureRank & ADC_RAW_12BIT_MASK);
    Adc_CachedResult.acquisitionSequence = Adc_AcquisitionSequence;

    Adc_ClearStatus(ADC_ISR_JEOS);
    Adc_State = ADC_INTERNAL_COMPLETED;
    *Result = Adc_CachedResult;

    return ADC_SAFETY_GROUP_COMPLETED;
}
