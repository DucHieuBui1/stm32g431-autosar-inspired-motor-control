#include "Mcu.h"

#include "stm32g431xx.h"

#define MCU_ADC12_EXPECTED_HCLK_HZ             170000000U

#define MCU_ADC12_GATE_MASK                    \
    (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_ADC12EN)

#define MCU_ADC12_PROHIBITED_GPIO_RESET_MASK   \
    (RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOCRST)

#define MCU_ADC12_ACTIVITY_MASK                \
    (ADC_CR_ADEN | ADC_CR_ADDIS | ADC_CR_ADSTART | ADC_CR_JADSTART | \
     ADC_CR_ADSTP | ADC_CR_JADSTP | ADC_CR_ADCAL)

#define MCU_ADC12_CFGR_RESET_VALUE             ADC_CFGR_JQDIS

#define MCU_ADC12_TEST_STAGE_GATES_ENABLED     1U
#define MCU_ADC12_TEST_STAGE_RESET_ASSERTED    2U
#define MCU_ADC12_TEST_STAGE_RESET_RELEASED    3U

#ifndef MCU_ADC12_TEST_HOOK
#define MCU_ADC12_TEST_HOOK(stage)             ((void)(stage))
#endif

STATIC boolean Mcu_Adc12ClockInitAttempted = FALSE;

STATIC boolean Mcu_AreAdcInstancesInactive(void)
{
    if (((ADC1->CR & MCU_ADC12_ACTIVITY_MASK) == 0U) &&
        ((ADC2->CR & MCU_ADC12_ACTIVITY_MASK) == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC boolean Mcu_IsAdc12ResetStateValid(void)
{
    if ((Mcu_AreAdcInstancesInactive() == TRUE) &&
        ((ADC1->CR & ADC_CR_DEEPPWD) != 0U) &&
        ((ADC2->CR & ADC_CR_DEEPPWD) != 0U) &&
        (ADC1->IER == 0U) &&
        (ADC2->IER == 0U) &&
        (ADC1->CFGR == MCU_ADC12_CFGR_RESET_VALUE) &&
        (ADC2->CFGR == MCU_ADC12_CFGR_RESET_VALUE) &&
        (ADC1->JSQR == 0U) &&
        (ADC2->JSQR == 0U) &&
        (ADC12_COMMON->CCR == 0U))
    {
        return TRUE;
    }

    return FALSE;
}

Std_ReturnType Mcu_Adc12PeripheralClockInit(void)
{
    uint32 gateState;
    uint32 resetState;

    if (Mcu_Adc12ClockInitAttempted == TRUE)
    {
        return E_NOT_OK;
    }
    Mcu_Adc12ClockInitAttempted = TRUE;

    if ((SystemCoreClock != MCU_ADC12_EXPECTED_HCLK_HZ) ||
        ((RCC->CCIPR & RCC_CCIPR_ADC12SEL) != RCC_CCIPR_ADC12SEL_0) ||
        ((RCC->AHB2RSTR & (MCU_ADC12_PROHIBITED_GPIO_RESET_MASK |
                          RCC_AHB2RSTR_ADC12RST)) != 0U))
    {
        return E_NOT_OK;
    }

    RCC->AHB2ENR |= MCU_ADC12_GATE_MASK;
    MCU_ADC12_TEST_HOOK(MCU_ADC12_TEST_STAGE_GATES_ENABLED);
    gateState = RCC->AHB2ENR;
    if ((gateState & MCU_ADC12_GATE_MASK) != MCU_ADC12_GATE_MASK)
    {
        return E_NOT_OK;
    }

    if (Mcu_AreAdcInstancesInactive() != TRUE)
    {
        return E_NOT_OK;
    }

    RCC->AHB2RSTR |= RCC_AHB2RSTR_ADC12RST;
    MCU_ADC12_TEST_HOOK(MCU_ADC12_TEST_STAGE_RESET_ASSERTED);
    resetState = RCC->AHB2RSTR;
    if (((resetState & RCC_AHB2RSTR_ADC12RST) == 0U) ||
        ((resetState & MCU_ADC12_PROHIBITED_GPIO_RESET_MASK) != 0U))
    {
        return E_NOT_OK;
    }

    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADC12RST;
    MCU_ADC12_TEST_HOOK(MCU_ADC12_TEST_STAGE_RESET_RELEASED);
    resetState = RCC->AHB2RSTR;
    if ((resetState & (RCC_AHB2RSTR_ADC12RST |
                       MCU_ADC12_PROHIBITED_GPIO_RESET_MASK)) != 0U)
    {
        return E_NOT_OK;
    }

    if (((RCC->AHB2ENR & MCU_ADC12_GATE_MASK) != MCU_ADC12_GATE_MASK) ||
        ((RCC->CCIPR & RCC_CCIPR_ADC12SEL) != RCC_CCIPR_ADC12SEL_0) ||
        (Mcu_IsAdc12ResetStateValid() != TRUE))
    {
        return E_NOT_OK;
    }

    return E_OK;
}
