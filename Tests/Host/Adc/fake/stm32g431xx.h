#ifndef STM32G431XX_H
#define STM32G431XX_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t ISR;
    volatile uint32_t IER;
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CFGR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    uint32_t RESERVED1[12];
    volatile uint32_t JSQR;
    uint32_t RESERVED2[12];
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    uint32_t RESERVED3[8];
    volatile uint32_t DIFSEL;
} ADC_TypeDef;

typedef struct
{
    volatile uint32_t CSR;
    uint32_t RESERVED1;
    volatile uint32_t CCR;
    volatile uint32_t CDR;
} ADC_Common_TypeDef;

typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
} GPIO_TypeDef;

typedef struct
{
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t CCIPR;
} RCC_TypeDef;

extern ADC_TypeDef AdcHost_Adc1;
extern ADC_TypeDef AdcHost_Adc2;
extern ADC_Common_TypeDef AdcHost_Adc12Common;
extern GPIO_TypeDef AdcHost_GpioA;
extern GPIO_TypeDef AdcHost_GpioC;
extern RCC_TypeDef AdcHost_Rcc;
extern uint32_t SystemCoreClock;

#define ADC1          (&AdcHost_Adc1)
#define ADC2          (&AdcHost_Adc2)
#define ADC12_COMMON  (&AdcHost_Adc12Common)
#define GPIOA         (&AdcHost_GpioA)
#define GPIOC         (&AdcHost_GpioC)
#define RCC           (&AdcHost_Rcc)

#define ADC_ISR_ADRDY   (1UL << 0U)
#define ADC_ISR_EOSMP   (1UL << 1U)
#define ADC_ISR_EOC     (1UL << 2U)
#define ADC_ISR_EOS     (1UL << 3U)
#define ADC_ISR_OVR     (1UL << 4U)
#define ADC_ISR_JEOC    (1UL << 5U)
#define ADC_ISR_JEOS    (1UL << 6U)
#define ADC_ISR_AWD1    (1UL << 7U)
#define ADC_ISR_AWD2    (1UL << 8U)
#define ADC_ISR_AWD3    (1UL << 9U)
#define ADC_ISR_JQOVF   (1UL << 10U)

#define ADC_CR_ADEN      (1UL << 0U)
#define ADC_CR_ADDIS     (1UL << 1U)
#define ADC_CR_ADSTART   (1UL << 2U)
#define ADC_CR_JADSTART  (1UL << 3U)
#define ADC_CR_ADSTP     (1UL << 4U)
#define ADC_CR_JADSTP    (1UL << 5U)
#define ADC_CR_ADVREGEN  (1UL << 28U)
#define ADC_CR_DEEPPWD   (1UL << 29U)
#define ADC_CR_ADCALDIF  (1UL << 30U)
#define ADC_CR_ADCAL     (1UL << 31U)

#define ADC_CFGR_JQM     (1UL << 21U)
#define ADC_CFGR_JQDIS   (1UL << 31U)

#define ADC_SMPR1_SMP1_Pos 3U
#define ADC_SMPR1_SMP1     (7UL << ADC_SMPR1_SMP1_Pos)
#define ADC_SMPR1_SMP5_Pos 15U
#define ADC_SMPR1_SMP5     (7UL << ADC_SMPR1_SMP5_Pos)

#define ADC_JSQR_JL_Pos       0U
#define ADC_JSQR_JL           (3UL << ADC_JSQR_JL_Pos)
#define ADC_JSQR_JEXTSEL      (31UL << 2U)
#define ADC_JSQR_JEXTEN       (3UL << 7U)
#define ADC_JSQR_JSQ1_Pos     9U
#define ADC_JSQR_JSQ1         (31UL << ADC_JSQR_JSQ1_Pos)
#define ADC_JSQR_JSQ2_Pos     15U
#define ADC_JSQR_JSQ2         (31UL << ADC_JSQR_JSQ2_Pos)
#define ADC_JSQR_JSQ3_Pos     21U
#define ADC_JSQR_JSQ3         (31UL << ADC_JSQR_JSQ3_Pos)
#define ADC_JSQR_JSQ4         (31UL << 27U)

#define ADC_DIFSEL_DIFSEL_1   (1UL << 1U)
#define ADC_DIFSEL_DIFSEL_5   (1UL << 5U)

#define ADC_CCR_DUAL          (31UL << 0U)
#define ADC_CCR_DELAY         (15UL << 8U)
#define ADC_CCR_DMACFG        (1UL << 13U)
#define ADC_CCR_MDMA          (3UL << 14U)
#define ADC_CCR_CKMODE        (3UL << 16U)
#define ADC_CCR_PRESC         (15UL << 18U)
#define ADC_CCR_VREFEN        (1UL << 22U)
#define ADC_CCR_VSENSESEL     (1UL << 23U)
#define ADC_CCR_VBATSEL       (1UL << 24U)

#define RCC_AHB2ENR_GPIOAEN   (1UL << 0U)
#define RCC_AHB2ENR_GPIOCEN   (1UL << 2U)
#define RCC_AHB2ENR_ADC12EN   (1UL << 13U)
#define RCC_AHB2RSTR_GPIOARST (1UL << 0U)
#define RCC_AHB2RSTR_GPIOCRST (1UL << 2U)
#define RCC_AHB2RSTR_ADC12RST (1UL << 13U)
#define RCC_CCIPR_ADC12SEL    (3UL << 28U)
#define RCC_CCIPR_ADC12SEL_0  (1UL << 28U)

void AdcHost_EventHook(uint32_t event);
void AdcHost_WaitHook(uint32_t stage, uint32_t observation);
void AdcHost_StatusClearHook(uint32_t mask);
uint32_t AdcHost_ReadInjectedData(volatile uint32_t *registerAddress,
                                  uint32_t rank);
void McuAdc12Host_Hook(uint32_t stage);

#define ADC_TEST_EVENT_HOOK(event) \
    AdcHost_EventHook((uint32_t)(event))
#define ADC_TEST_WAIT_HOOK(stage, observation) \
    AdcHost_WaitHook((uint32_t)(stage), (uint32_t)(observation))
#define ADC_TEST_STATUS_CLEAR_HOOK(mask) \
    AdcHost_StatusClearHook((uint32_t)(mask))
#define ADC_READ_INJECTED_DATA(registerValue, rank) \
    AdcHost_ReadInjectedData(&(registerValue), (uint32_t)(rank))
#define MCU_ADC12_TEST_HOOK(stage) \
    McuAdc12Host_Hook((uint32_t)(stage))

#define __NOP() ((void)0)

#endif /* STM32G431XX_H */
