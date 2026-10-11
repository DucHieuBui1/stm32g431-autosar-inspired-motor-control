#ifndef STM32G431XX_H
#define STM32G431XX_H

#include <stdint.h>

#define __NVIC_PRIO_BITS 4U

typedef enum
{
    SVCall_IRQn = -5,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1
} IRQn_Type;

typedef struct
{
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
} SysTick_Type;

typedef struct
{
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
} SCB_Type;

extern SysTick_Type OsHost_SysTick;
extern SCB_Type OsHost_Scb;
extern uint32_t SystemCoreClock;

#define SysTick (&OsHost_SysTick)
#define SCB     (&OsHost_Scb)

#define SysTick_CTRL_ENABLE_Msk       (1UL << 0U)
#define SysTick_CTRL_TICKINT_Msk      (1UL << 1U)
#define SysTick_CTRL_CLKSOURCE_Msk    (1UL << 2U)
#define SysTick_LOAD_RELOAD_Msk       0x00FFFFFFUL
#define SCB_ICSR_PENDSVSET_Msk        (1UL << 28U)
#define SCB_ICSR_PENDSVCLR_Msk        (1UL << 27U)
#define SCB_CCR_STKALIGN_Msk          (1UL << 9U)

void OsHost_NvicSetPriorityGrouping(uint32_t grouping);
uint32_t OsHost_NvicGetPriorityGrouping(void);
void OsHost_NvicSetPriority(IRQn_Type irq, uint32_t priority);
uint32_t OsHost_NvicGetPriority(IRQn_Type irq);
uint32_t OsHost_GetBasePri(void);
void OsHost_SetBasePri(uint32_t value);
void OsHost_SetBasePriMax(uint32_t value);

#define NVIC_SetPriorityGrouping(grouping) \
    OsHost_NvicSetPriorityGrouping(grouping)
#define NVIC_GetPriorityGrouping() OsHost_NvicGetPriorityGrouping()
#define NVIC_SetPriority(irq, priority) \
    OsHost_NvicSetPriority((irq), (priority))
#define NVIC_GetPriority(irq) OsHost_NvicGetPriority((irq))
#define __get_BASEPRI() OsHost_GetBasePri()
#define __set_BASEPRI(value) OsHost_SetBasePri(value)
#define __set_BASEPRI_MAX(value) OsHost_SetBasePriMax(value)
#define __DSB() do { } while (0)
#define __ISB() do { } while (0)
#define __WFI() do { } while (0)

#endif /* STM32G431XX_H */
