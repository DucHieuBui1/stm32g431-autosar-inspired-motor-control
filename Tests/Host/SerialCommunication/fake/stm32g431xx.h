#ifndef STM32G431XX_H
#define STM32G431XX_H

#include <stdint.h>

/* MinGW GCC 6 does not expose this target builtin as a C constant expression. */
#define __atomic_always_lock_free(size, pointer) 1

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
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t GTPR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
    volatile uint32_t PRESC;
} USART_TypeDef;

typedef enum
{
    USART2_IRQn = 38
} IRQn_Type;

extern GPIO_TypeDef SerialHost_GpioA;
extern USART_TypeDef SerialHost_Usart2;

#define GPIOA  (&SerialHost_GpioA)
#define USART2 (&SerialHost_Usart2)

void SerialHost_NvicSetPriority(IRQn_Type irq, uint32_t priority);
void SerialHost_NvicClearPending(IRQn_Type irq);
void SerialHost_NvicEnable(IRQn_Type irq);
uint32_t SerialHost_NvicGetPriority(IRQn_Type irq);
uint32_t SerialHost_NvicGetEnable(IRQn_Type irq);
void SerialHost_InitHook(void);
void SerialHost_RxClearHook(uint32_t mask);
void SerialHost_TxHook(uint32_t value);
void SerialHost_ConsumeHook(uint32_t length);
void SerialHost_DsbHook(void);

#define NVIC_SetPriority(irq, priority) \
    SerialHost_NvicSetPriority((irq), (priority))
#define NVIC_ClearPendingIRQ(irq) SerialHost_NvicClearPending((irq))
#define NVIC_EnableIRQ(irq) SerialHost_NvicEnable((irq))
#define NVIC_GetPriority(irq) SerialHost_NvicGetPriority((irq))
#define NVIC_GetEnableIRQ(irq) SerialHost_NvicGetEnable((irq))
#define SERIAL_COMMUNICATION_CDD_TEST_INIT_HOOK() SerialHost_InitHook()
#define SERIAL_COMMUNICATION_CDD_TEST_RX_CLEAR_HOOK(mask) \
    SerialHost_RxClearHook((mask))
#define SERIAL_COMMUNICATION_CDD_TEST_TX_HOOK(value) \
    SerialHost_TxHook((value))
#define SERIAL_COMMUNICATION_CDD_TEST_CONSUME_HOOK(length) \
    SerialHost_ConsumeHook((length))
#define __DSB() SerialHost_DsbHook()

#endif /* STM32G431XX_H */
