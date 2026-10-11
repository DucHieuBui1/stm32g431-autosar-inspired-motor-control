#ifndef STM32G431XX_H
#define STM32G431XX_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t APB1RSTR1;
    volatile uint32_t APB1ENR1;
    volatile uint32_t CCIPR;
} RCC_TypeDef;

typedef struct
{
    volatile uint32_t CR1;
} USART_TypeDef;

extern RCC_TypeDef McuCommunicationHost_Rcc;
extern USART_TypeDef McuCommunicationHost_Usart2;
extern uint32_t McuCommunicationHost_UidWords[3];
extern uint32_t SystemCoreClock;

#define RCC     (&McuCommunicationHost_Rcc)
#define USART2  (&McuCommunicationHost_Usart2)

#define RCC_CSR_OBLRSTF             (1UL << 25U)
#define RCC_CSR_PINRSTF             (1UL << 26U)
#define RCC_CSR_BORRSTF             (1UL << 27U)
#define RCC_CSR_SFTRSTF             (1UL << 28U)
#define RCC_CSR_IWDGRSTF            (1UL << 29U)
#define RCC_CSR_WWDGRSTF            (1UL << 30U)
#define RCC_CSR_LPWRRSTF            (1UL << 31U)

#define RCC_AHB2ENR_GPIOAEN         (1UL << 0U)
#define RCC_AHB2RSTR_GPIOARST       (1UL << 0U)
#define RCC_APB1ENR1_USART2EN       (1UL << 17U)
#define RCC_APB1RSTR1_USART2RST     (1UL << 17U)
#define RCC_CCIPR_USART2SEL         (3UL << 2U)
#define USART_CR1_UE                (1UL << 0U)

void McuCommunicationHost_TestHook(uint32_t stage);

#define MCU_COMMUNICATION_READ_UID_WORD(index) \
    (McuCommunicationHost_UidWords[(index)])
#define MCU_COMMUNICATION_TEST_HOOK(stage) \
    McuCommunicationHost_TestHook((uint32_t)(stage))

#endif /* STM32G431XX_H */
