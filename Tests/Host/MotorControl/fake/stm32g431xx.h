#ifndef STM32G431XX_H
#define STM32G431XX_H

#include <stdint.h>

/* MinGW GCC 6 does not expose this target builtin as a C constant expression. */
#define __atomic_always_lock_free(size, pointer) 1

typedef struct
{
    volatile uint32_t ISR;
    volatile uint32_t IER;
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CFGR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    uint32_t RESERVED1;
    volatile uint32_t TR1;
    volatile uint32_t TR2;
    volatile uint32_t TR3;
    uint32_t RESERVED2;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t SQR4;
    volatile uint32_t DR;
    uint32_t RESERVED3[2];
    volatile uint32_t JSQR;
    uint32_t RESERVED4[12];
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    uint32_t RESERVED5[8];
    volatile uint32_t DIFSEL;
} ADC_TypeDef;

typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
    volatile uint32_t BRR;
} GPIO_TypeDef;

typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
} TIM_TypeDef;

typedef struct
{
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t CCIPR;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;

typedef enum
{
    ADC1_2_IRQn = 18,
    TIM1_BRK_TIM15_IRQn = 24,
    TIM1_UP_TIM16_IRQn = 25
} IRQn_Type;

extern ADC_TypeDef MotorHost_Adc1;
extern GPIO_TypeDef MotorHost_GpioA;
extern GPIO_TypeDef MotorHost_GpioB;
extern TIM_TypeDef MotorHost_Tim1;
extern RCC_TypeDef MotorHost_Rcc;

#define ADC1  (&MotorHost_Adc1)
#define GPIOA (&MotorHost_GpioA)
#define GPIOB (&MotorHost_GpioB)
#define TIM1  (&MotorHost_Tim1)
#define RCC   (&MotorHost_Rcc)

#define GPIO_ODR_OD5  (1UL << 5U)
#define GPIO_ODR_OD13 (1UL << 13U)
#define GPIO_ODR_OD14 (1UL << 14U)
#define GPIO_ODR_OD15 (1UL << 15U)
#define GPIO_BSRR_BR5  (1UL << 21U)
#define GPIO_BSRR_BR13 (1UL << 29U)
#define GPIO_BSRR_BR14 (1UL << 30U)
#define GPIO_BSRR_BR15 (1UL << 31U)

#define GPIO_MODER_MODE0      (3UL << 0U)
#define GPIO_MODER_MODE1      (3UL << 2U)
#define GPIO_MODER_MODE1_0    (1UL << 2U)
#define GPIO_MODER_MODE1_1    (2UL << 2U)
#define GPIO_MODER_MODE5      (3UL << 10U)
#define GPIO_MODER_MODE5_0    (1UL << 10U)
#define GPIO_MODER_MODE8      (3UL << 16U)
#define GPIO_MODER_MODE8_1    (2UL << 16U)
#define GPIO_MODER_MODE9      (3UL << 18U)
#define GPIO_MODER_MODE9_1    (2UL << 18U)
#define GPIO_MODER_MODE10     (3UL << 20U)
#define GPIO_MODER_MODE10_1   (2UL << 20U)
#define GPIO_MODER_MODE12     (3UL << 24U)
#define GPIO_MODER_MODE12_1   (2UL << 24U)
#define GPIO_MODER_MODE13     (3UL << 26U)
#define GPIO_MODER_MODE13_0   (1UL << 26U)
#define GPIO_MODER_MODE14     (3UL << 28U)
#define GPIO_MODER_MODE14_0   (1UL << 28U)
#define GPIO_MODER_MODE15     (3UL << 30U)
#define GPIO_MODER_MODE15_0   (1UL << 30U)

#define GPIO_OTYPER_OT5  (1UL << 5U)
#define GPIO_OTYPER_OT8  (1UL << 8U)
#define GPIO_OTYPER_OT9  (1UL << 9U)
#define GPIO_OTYPER_OT10 (1UL << 10U)
#define GPIO_OTYPER_OT12 (1UL << 12U)
#define GPIO_OTYPER_OT13 (1UL << 13U)
#define GPIO_OTYPER_OT14 (1UL << 14U)
#define GPIO_OTYPER_OT15 (1UL << 15U)

#define GPIO_OSPEEDR_OSPEED5  (3UL << 10U)
#define GPIO_OSPEEDR_OSPEED8  (3UL << 16U)
#define GPIO_OSPEEDR_OSPEED9  (3UL << 18U)
#define GPIO_OSPEEDR_OSPEED10 (3UL << 20U)
#define GPIO_OSPEEDR_OSPEED13 (3UL << 26U)
#define GPIO_OSPEEDR_OSPEED14 (3UL << 28U)
#define GPIO_OSPEEDR_OSPEED15 (3UL << 30U)

#define GPIO_PUPDR_PUPD0    (3UL << 0U)
#define GPIO_PUPDR_PUPD1    (3UL << 2U)
#define GPIO_PUPDR_PUPD5    (3UL << 10U)
#define GPIO_PUPDR_PUPD5_1  (2UL << 10U)
#define GPIO_PUPDR_PUPD8    (3UL << 16U)
#define GPIO_PUPDR_PUPD9    (3UL << 18U)
#define GPIO_PUPDR_PUPD10   (3UL << 20U)
#define GPIO_PUPDR_PUPD12   (3UL << 24U)
#define GPIO_PUPDR_PUPD12_0 (1UL << 24U)
#define GPIO_PUPDR_PUPD13   (3UL << 26U)
#define GPIO_PUPDR_PUPD13_1 (2UL << 26U)
#define GPIO_PUPDR_PUPD14   (3UL << 28U)
#define GPIO_PUPDR_PUPD14_1 (2UL << 28U)
#define GPIO_PUPDR_PUPD15   (3UL << 30U)
#define GPIO_PUPDR_PUPD15_1 (2UL << 30U)

#define GPIO_AFRH_AFSEL8_Pos  0U
#define GPIO_AFRH_AFSEL8      (15UL << GPIO_AFRH_AFSEL8_Pos)
#define GPIO_AFRH_AFSEL9_Pos  4U
#define GPIO_AFRH_AFSEL9      (15UL << GPIO_AFRH_AFSEL9_Pos)
#define GPIO_AFRH_AFSEL10_Pos 8U
#define GPIO_AFRH_AFSEL10     (15UL << GPIO_AFRH_AFSEL10_Pos)
#define GPIO_AFRH_AFSEL12_Pos 16U
#define GPIO_AFRH_AFSEL12     (15UL << GPIO_AFRH_AFSEL12_Pos)

#define RCC_AHB2ENR_GPIOAEN (1UL << 0U)
#define RCC_AHB2ENR_GPIOBEN (1UL << 1U)
#define RCC_AHB2ENR_ADC12EN (1UL << 13U)
#define RCC_AHB2RSTR_ADC12RST (1UL << 13U)
#define RCC_APB2ENR_TIM1EN (1UL << 11U)
#define RCC_CCIPR_ADC12SEL   (3UL << 28U)
#define RCC_CCIPR_ADC12SEL_0 (1UL << 28U)

#define ADC_ISR_ADRDY (1UL << 0U)
#define ADC_ISR_EOSMP (1UL << 1U)
#define ADC_ISR_EOC   (1UL << 2U)
#define ADC_ISR_EOS   (1UL << 3U)
#define ADC_ISR_OVR   (1UL << 4U)
#define ADC_ISR_JEOC  (1UL << 5U)
#define ADC_ISR_JEOS  (1UL << 6U)
#define ADC_ISR_AWD1  (1UL << 7U)
#define ADC_ISR_AWD2  (1UL << 8U)
#define ADC_ISR_AWD3  (1UL << 9U)
#define ADC_ISR_JQOVF (1UL << 10U)
#define ADC_IER_JEOSIE (1UL << 6U)

#define ADC_CR_ADEN     (1UL << 0U)
#define ADC_CR_ADDIS    (1UL << 1U)
#define ADC_CR_ADSTART  (1UL << 2U)
#define ADC_CR_JADSTART (1UL << 3U)
#define ADC_CR_ADSTP    (1UL << 4U)
#define ADC_CR_JADSTP   (1UL << 5U)
#define ADC_CR_ADVREGEN (1UL << 28U)
#define ADC_CR_DEEPPWD  (1UL << 29U)
#define ADC_CR_ADCALDIF (1UL << 30U)
#define ADC_CR_ADCAL    (1UL << 31U)
#define ADC_CFGR_JQM    (1UL << 21U)
#define ADC_CFGR_DMAEN  (1UL << 0U)
#define ADC_CFGR_DMACFG (1UL << 1U)
#define ADC_CFGR_EXTSEL (31UL << 5U)
#define ADC_CFGR_EXTEN  (3UL << 10U)
#define ADC_CFGR_OVRMOD (1UL << 12U)
#define ADC_CFGR_CONT   (1UL << 13U)
#define ADC_CFGR_AUTDLY (1UL << 14U)
#define ADC_CFGR_DISCEN (1UL << 16U)
#define ADC_CFGR_DISCNUM (7UL << 17U)
#define ADC_CFGR_DISCNUM_0 (1UL << 17U)
#define ADC_SQR1_L_Pos   0U
#define ADC_SQR1_SQ1_Pos 6U
#define ADC_SQR1_SQ2_Pos 12U
#define ADC_SQR1_SQ3_Pos 18U
#define ADC_SQR1_SQ4_Pos 24U
#define ADC_SQR2_SQ5_Pos 0U
#define ADC_SQR2_SQ6_Pos 6U
#define ADC_JSQR_JL_Pos 0U
#define ADC_JSQR_JEXTEN_Pos 7U
#define ADC_JSQR_JEXTEN (3UL << ADC_JSQR_JEXTEN_Pos)
#define ADC_JSQR_JSQ1_Pos 9U
#define ADC_JSQR_JSQ2_Pos 15U

#define ADC_SMPR1_SMP2_Pos 6U
#define ADC_SMPR1_SMP2     (7UL << ADC_SMPR1_SMP2_Pos)
#define ADC_SMPR2_SMP12_Pos 6U
#define ADC_SMPR2_SMP12     (7UL << ADC_SMPR2_SMP12_Pos)
#define ADC_SMPR2_SMP15_Pos 15U
#define ADC_SMPR2_SMP15     (7UL << ADC_SMPR2_SMP15_Pos)
#define ADC_DIFSEL_DIFSEL_2  (1UL << 2U)
#define ADC_DIFSEL_DIFSEL_12 (1UL << 12U)
#define ADC_DIFSEL_DIFSEL_15 (1UL << 15U)

#define TIM_CR1_CEN   (1UL << 0U)
#define TIM_CR1_DIR   (1UL << 4U)
#define TIM_CR1_CMS_0 (1UL << 5U)
#define TIM_CR1_CKD_0 (1UL << 8U)
#define TIM_CR2_MMS_Pos 4U
#define TIM_CR2_MMS   (0x02000070UL)
#define TIM_DIER_UIE  (1UL << 0U)
#define TIM_DIER_BIE  (1UL << 7U)
#define TIM_SR_UIF    (1UL << 0U)
#define TIM_SR_BIF    (1UL << 7U)
#define TIM_EGR_UG    (1UL << 0U)
#define TIM_CCMR1_OC1PE (1UL << 3U)
#define TIM_CCMR1_OC1M_Pos 4U
#define TIM_CCMR1_OC2PE (1UL << 11U)
#define TIM_CCMR1_OC2M_Pos 12U
#define TIM_CCMR2_OC3PE (1UL << 3U)
#define TIM_CCMR2_OC3M_Pos 4U
#define TIM_CCMR2_OC4PE (1UL << 11U)
#define TIM_CCMR2_OC4M_Pos 12U
#define TIM_CCER_CC1E (1UL << 0U)
#define TIM_CCER_CC2E (1UL << 4U)
#define TIM_CCER_CC3E (1UL << 8U)
#define TIM_BDTR_OSSI (1UL << 10U)
#define TIM_BDTR_OSSR (1UL << 11U)
#define TIM_BDTR_BKE  (1UL << 12U)
#define TIM_BDTR_AOE  (1UL << 14U)
#define TIM_BDTR_MOE  (1UL << 15U)
#define TIM_BDTR_BKF_Pos 16U

void MotorHost_NvicSetPriority(IRQn_Type irq, uint32_t priority);
void MotorHost_NvicClearPending(IRQn_Type irq);
void MotorHost_NvicEnable(IRQn_Type irq);
void MotorHost_WaitHook(uint32_t stage, uint32_t observation);
void MotorHost_AdcStatusClearHook(uint32_t mask);
void MotorHost_TimStatusClearHook(uint32_t mask);
void MotorHost_NextGenerationHook(void *generation);
void MotorHost_CommitHook(void);
void MotorHost_MailboxHook(uint32_t stage, uint32_t slot);

#define NVIC_SetPriority(irq, priority) \
    MotorHost_NvicSetPriority((irq), (priority))
#define NVIC_ClearPendingIRQ(irq) MotorHost_NvicClearPending((irq))
#define NVIC_EnableIRQ(irq) MotorHost_NvicEnable((irq))
#define MOTOR_CONTROL_CDD_TEST_WAIT_HOOK(stage, observation) \
    MotorHost_WaitHook((stage), (observation))
#define MOTOR_CONTROL_CDD_TEST_ADC_STATUS_CLEAR_HOOK(mask) \
    MotorHost_AdcStatusClearHook((mask))
#define MOTOR_CONTROL_CDD_TEST_TIM_STATUS_CLEAR_HOOK(mask) \
    MotorHost_TimStatusClearHook((mask))
#define MOTOR_CONTROL_CDD_TEST_NEXT_GENERATION_HOOK(generation) \
    MotorHost_NextGenerationHook((void *)(generation))
#define MOTOR_CONTROL_CDD_TEST_COMMIT_HOOK() MotorHost_CommitHook()
#define MOTOR_CONTROL_CDD_TEST_MAILBOX_HOOK(stage, slot) \
    MotorHost_MailboxHook((stage), (slot))

#define __NOP() ((void)0)
#define __DSB() ((void)0)
#define __DMB() ((void)0)

#endif /* STM32G431XX_H */
