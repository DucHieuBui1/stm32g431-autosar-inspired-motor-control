#include "Mcu.h"
#include "Mcu_Communication.h"

#include "MotorControlCdd.h"
#include "StateManager.h"
#include "stm32g431xx.h"

#define MCU_EXPECTED_HCLK_HZ                  170000000U

#define MCU_SUPPORTED_DEVICE_ID               0x0468U
#define MCU_SUPPORTED_REVISION_X_ID           0x2003U

#define MCU_RESET_FLAG_MASK                   \
    (RCC_CSR_OBLRSTF | RCC_CSR_PINRSTF | RCC_CSR_BORRSTF | \
     RCC_CSR_SFTRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | \
     RCC_CSR_LPWRRSTF)

#define MCU_PLLM_DIVIDER                      6U
#define MCU_PLLN_MULTIPLIER                   85U
#define MCU_PLLP_DIVIDER                      8U

#define MCU_REGISTER_TIMEOUT_ITERATIONS       2000000U
#define MCU_BOOST_SETTLING_CYCLES             256U
#define MCU_CSS_CLEAR_TIMEOUT_ITERATIONS      32U

#define MCU_FAULT_SNAPSHOT_MAGIC              0x4D435546U
#define MCU_FAULT_SNAPSHOT_FORMAT_VERSION     1U

#define MCU_FAULT_REASON_NMI_UNKNOWN          1U
#define MCU_FAULT_REASON_HARDFAULT            2U
#define MCU_FAULT_REASON_MEMMANAGE            3U
#define MCU_FAULT_REASON_BUSFAULT             4U
#define MCU_FAULT_REASON_USAGEFAULT           5U
#define MCU_FAULT_REASON_UNEXPECTED_VECTOR    6U
#define MCU_FAULT_REASON_CSS_CLEAR_FAILED     7U

#ifndef MCU_TEST_CLOCK_FAULT_READ_HOOK
#define MCU_TEST_CLOCK_FAULT_READ_HOOK() ((void)0)
#endif

#define MCU_FLASH_CONFIG_MASK                 \
    (FLASH_ACR_LATENCY | FLASH_ACR_PRFTEN |   \
     FLASH_ACR_ICEN | FLASH_ACR_DCEN)

#define MCU_FLASH_CONFIG_VALUE                \
    (FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | \
     FLASH_ACR_ICEN | FLASH_ACR_DCEN)

#define MCU_PLL_CONFIG_MASK                   \
    (RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM |  \
     RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLPEN |  \
     RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQEN |  \
     RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLREN |  \
     RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLPDIV)

#define MCU_PLL_CONFIG_VALUE                  \
    (RCC_PLLCFGR_PLLSRC_HSE |                 \
     (((MCU_PLLM_DIVIDER - 1U) << RCC_PLLCFGR_PLLM_Pos) & \
      RCC_PLLCFGR_PLLM) |                     \
     ((MCU_PLLN_MULTIPLIER << RCC_PLLCFGR_PLLN_Pos) & \
      RCC_PLLCFGR_PLLN) |                     \
     RCC_PLLCFGR_PLLPEN |                     \
     RCC_PLLCFGR_PLLREN |                     \
     ((MCU_PLLP_DIVIDER << RCC_PLLCFGR_PLLPDIV_Pos) & \
      RCC_PLLCFGR_PLLPDIV))

#define MCU_FINAL_CR_MASK                     \
    (RCC_CR_HSEON | RCC_CR_HSERDY | RCC_CR_HSEBYP | \
     RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_PLLRDY)

#define MCU_FINAL_CR_VALUE                    \
    (RCC_CR_HSEON | RCC_CR_HSERDY | RCC_CR_CSSON | \
     RCC_CR_PLLON | RCC_CR_PLLRDY)

#define MCU_FINAL_CFGR_MASK                   \
    (RCC_CFGR_SW | RCC_CFGR_SWS | RCC_CFGR_HPRE | \
     RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)

#define MCU_FINAL_CFGR_VALUE                  \
    (RCC_CFGR_SW_PLL | RCC_CFGR_SWS_PLL |     \
     RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | \
     RCC_CFGR_PPRE2_DIV1)

typedef struct
{
    uint32 magic;
    uint32 formatVersion;
    uint32 lengthBytes;
    uint32 faultReason;
    uint32 ipsr;
    uint32 icsr;
    uint32 cfsr;
    uint32 hfsr;
    uint32 mmfar;
    uint32 bfar;
    uint32 rccCifr;
    uint32 gpioBOutput;
    uint32 tim1Bdtr;
    uint32 crc32;
} Mcu_FaultSnapshotType;

STATIC boolean Mcu_StartupRecordCaptured = FALSE;
STATIC uint32 Mcu_ClockFaultEvents = MCU_CLOCK_FAULT_EVENT_NONE;

volatile Mcu_FaultSnapshotType Mcu_FaultSnapshot
    __attribute__((section(".noinit.mcu_fault_snapshot"), used, aligned(4)));

_Static_assert(__atomic_always_lock_free(sizeof(uint32), 0),
               "32-bit clock-fault events must be lock-free");

STATIC uint32 Mcu_CalculateFaultSnapshotCrc32(void)
{
    volatile const uint32 *snapshotWords =
        (volatile const uint32 *)&Mcu_FaultSnapshot;
    uint32 crc = 0xFFFFFFFFU;
    uint32 wordIndex;

    for (wordIndex = 0U;
         wordIndex < ((sizeof(Mcu_FaultSnapshotType) / sizeof(uint32)) - 1U);
         wordIndex++)
    {
        uint32 data = snapshotWords[wordIndex];
        uint32 bitIndex;

        for (bitIndex = 0U; bitIndex < 32U; bitIndex++)
        {
            uint32 polynomialMask = 0U - ((crc ^ data) & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & polynomialMask);
            data >>= 1U;
        }
    }

    return ~crc;
}

STATIC void Mcu_StoreFatalSnapshot(uint32 faultReason)
{
    Mcu_FaultSnapshot.magic = MCU_FAULT_SNAPSHOT_MAGIC;
    Mcu_FaultSnapshot.formatVersion = MCU_FAULT_SNAPSHOT_FORMAT_VERSION;
    Mcu_FaultSnapshot.lengthBytes = sizeof(Mcu_FaultSnapshotType);
    Mcu_FaultSnapshot.faultReason = faultReason;
    Mcu_FaultSnapshot.ipsr = __get_IPSR();
    Mcu_FaultSnapshot.icsr = SCB->ICSR;
    Mcu_FaultSnapshot.cfsr = SCB->CFSR;
    Mcu_FaultSnapshot.hfsr = SCB->HFSR;
    Mcu_FaultSnapshot.mmfar = SCB->MMFAR;
    Mcu_FaultSnapshot.bfar = SCB->BFAR;
    Mcu_FaultSnapshot.rccCifr = RCC->CIFR;
    Mcu_FaultSnapshot.gpioBOutput = GPIOB->ODR;
    Mcu_FaultSnapshot.tim1Bdtr = TIM1->BDTR;
    Mcu_FaultSnapshot.crc32 = Mcu_CalculateFaultSnapshotCrc32();
}

STATIC __attribute__((noreturn)) void Mcu_EnterFatalException(uint32 faultReason)
{
    __disable_irq();
    MotorControlCdd_EmergencyOff();
    Mcu_StoreFatalSnapshot(faultReason);
    __DSB();
    __ISB();

    for (;;)
    {
        /* Stop watchdog refresh and wait for IWDG or an external reset. */
        __NOP();
    }
}

STATIC boolean Mcu_ClearCssFaultFlag(void)
{
    uint32 iterationsRemaining = MCU_CSS_CLEAR_TIMEOUT_ITERATIONS;

    RCC->CICR = RCC_CICR_CSSC;
    __DSB();

    while (iterationsRemaining > 0U)
    {
        if ((RCC->CIFR & RCC_CIFR_CSSF) == 0U)
        {
            return TRUE;
        }

        iterationsRemaining--;
    }

    return FALSE;
}

STATIC Std_ReturnType Mcu_WaitForRegisterValue(
    volatile uint32 *registerAddress,
    uint32 mask,
    uint32 expectedValue)
{
    uint32 iterationsRemaining = MCU_REGISTER_TIMEOUT_ITERATIONS;

    while (iterationsRemaining > 0U)
    {
        if ((*registerAddress & mask) == expectedValue)
        {
            return E_OK;
        }

        iterationsRemaining--;
    }

    return E_NOT_OK;
}

STATIC void Mcu_WaitForBoostSettlingTime(void)
{
    volatile uint32 cyclesRemaining = MCU_BOOST_SETTLING_CYCLES;

    /* At the temporary 85 MHz HCLK, 256 retained iterations exceed 1 us. */
    while (cyclesRemaining > 0U)
    {
        __NOP();
        cyclesRemaining--;
    }
}

STATIC boolean Mcu_IsResetClockBaselineValid(void)
{
    uint32 oscillatorState = RCC->CR;
    uint32 systemClockState = RCC->CFGR;

    if (((oscillatorState & (RCC_CR_HSION | RCC_CR_HSIRDY)) ==
         (RCC_CR_HSION | RCC_CR_HSIRDY)) &&
        ((oscillatorState & (RCC_CR_HSEON | RCC_CR_HSERDY |
                             RCC_CR_PLLON | RCC_CR_PLLRDY)) == 0U) &&
        ((systemClockState & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI))
    {
        return TRUE;
    }

    return FALSE;
}

STATIC Std_ReturnType Mcu_ConfigureVoltageAndFlash(void)
{
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0;

    if (Mcu_WaitForRegisterValue(&PWR->SR2, PWR_SR2_VOSF, 0U) != E_OK)
    {
        return E_NOT_OK;
    }

    FLASH->ACR =
        (FLASH->ACR & ~MCU_FLASH_CONFIG_MASK) | MCU_FLASH_CONFIG_VALUE;

    if ((FLASH->ACR & MCU_FLASH_CONFIG_MASK) != MCU_FLASH_CONFIG_VALUE)
    {
        return E_NOT_OK;
    }

    /* Keep HCLK at or below 85 MHz while entering Range 1 Boost. */
    RCC->CFGR =
        (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
        RCC_CFGR_HPRE_DIV2 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1;

    if ((RCC->CFGR & (RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) !=
        (RCC_CFGR_HPRE_DIV2 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1))
    {
        return E_NOT_OK;
    }

    PWR->CR5 &= ~PWR_CR5_R1MODE;

    if (((PWR->CR1 & PWR_CR1_VOS) != PWR_CR1_VOS_0) ||
        ((PWR->CR5 & PWR_CR5_R1MODE) != 0U))
    {
        return E_NOT_OK;
    }

    return E_OK;
}

STATIC Std_ReturnType Mcu_StartHseAndPll(void)
{
    RCC->CR &= ~RCC_CR_HSEBYP;

    if ((RCC->CR & RCC_CR_HSEBYP) != 0U)
    {
        return E_NOT_OK;
    }

    RCC->CR |= RCC_CR_HSEON;

    if (Mcu_WaitForRegisterValue(&RCC->CR, RCC_CR_HSERDY,
                                 RCC_CR_HSERDY) != E_OK)
    {
        return E_NOT_OK;
    }

    RCC->PLLCFGR =
        (RCC->PLLCFGR & ~MCU_PLL_CONFIG_MASK) | MCU_PLL_CONFIG_VALUE;

    if ((RCC->PLLCFGR & MCU_PLL_CONFIG_MASK) != MCU_PLL_CONFIG_VALUE)
    {
        return E_NOT_OK;
    }

    RCC->CR |= RCC_CR_PLLON;

    if (Mcu_WaitForRegisterValue(&RCC->CR, RCC_CR_PLLRDY,
                                 RCC_CR_PLLRDY) != E_OK)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

STATIC Std_ReturnType Mcu_SwitchToProductionClock(void)
{
    RCC->CCIPR =
        (RCC->CCIPR & ~RCC_CCIPR_ADC12SEL) | RCC_CCIPR_ADC12SEL_0;

    if ((RCC->CCIPR & RCC_CCIPR_ADC12SEL) != RCC_CCIPR_ADC12SEL_0)
    {
        return E_NOT_OK;
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;

    if (Mcu_WaitForRegisterValue(&RCC->CFGR, RCC_CFGR_SWS,
                                 RCC_CFGR_SWS_PLL) != E_OK)
    {
        return E_NOT_OK;
    }

    Mcu_WaitForBoostSettlingTime();

    RCC->CFGR =
        (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
        RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1;

    RCC->CR |= RCC_CR_CSSON;
    SystemCoreClockUpdate();

    return E_OK;
}

STATIC boolean Mcu_IsProductionClockValid(void)
{
    uint32 gpioBClockState = RCC->AHB2ENR;
    uint32 tim1ClockState = RCC->APB2ENR;

    if (((PWR->CR1 & PWR_CR1_VOS) == PWR_CR1_VOS_0) &&
        ((PWR->CR5 & PWR_CR5_R1MODE) == 0U) &&
        ((PWR->SR2 & PWR_SR2_VOSF) == 0U) &&
        ((FLASH->ACR & MCU_FLASH_CONFIG_MASK) == MCU_FLASH_CONFIG_VALUE) &&
        ((RCC->CR & MCU_FINAL_CR_MASK) == MCU_FINAL_CR_VALUE) &&
        ((RCC->PLLCFGR & MCU_PLL_CONFIG_MASK) == MCU_PLL_CONFIG_VALUE) &&
        ((RCC->CFGR & MCU_FINAL_CFGR_MASK) == MCU_FINAL_CFGR_VALUE) &&
        ((RCC->CCIPR & RCC_CCIPR_ADC12SEL) == RCC_CCIPR_ADC12SEL_0) &&
        ((gpioBClockState & RCC_AHB2ENR_GPIOBEN) != 0U) &&
        ((tim1ClockState & RCC_APB2ENR_TIM1EN) != 0U) &&
        (SystemCoreClock == MCU_EXPECTED_HCLK_HZ))
    {
        return TRUE;
    }

    return FALSE;
}

Std_ReturnType Mcu_EarlyClockGateInit(void)
{
    uint32 gpioBClockState;
    uint32 tim1ClockState;

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    gpioBClockState = RCC->AHB2ENR;

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    tim1ClockState = RCC->APB2ENR;

    if (((gpioBClockState & RCC_AHB2ENR_GPIOBEN) != 0U) &&
        ((tim1ClockState & RCC_APB2ENR_TIM1EN) != 0U))
    {
        return E_OK;
    }

    return E_NOT_OK;
}

Std_ReturnType Mcu_StartupValidationInit(Mcu_StartupInfoType *StartupInfo)
{
    uint32 rawDbgmcuIdcode;
    uint32 rawResetFlags;
    uint16 deviceId;
    uint16 revisionId;

    if ((StartupInfo == NULL_PTR) ||
        (Mcu_StartupRecordCaptured == TRUE))
    {
        return E_NOT_OK;
    }

    rawDbgmcuIdcode = DBGMCU->IDCODE;
    rawResetFlags = RCC->CSR & MCU_RESET_FLAG_MASK;
    deviceId = (uint16)((rawDbgmcuIdcode & DBGMCU_IDCODE_DEV_ID) >>
                        DBGMCU_IDCODE_DEV_ID_Pos);
    revisionId = (uint16)((rawDbgmcuIdcode & DBGMCU_IDCODE_REV_ID) >>
                          DBGMCU_IDCODE_REV_ID_Pos);

    StartupInfo->rawDbgmcuIdcode = rawDbgmcuIdcode;
    StartupInfo->rawResetFlags = rawResetFlags;
    StartupInfo->deviceId = deviceId;
    StartupInfo->revisionId = revisionId;
    StartupInfo->bootIdentity = 0U;
    Mcu_StartupRecordCaptured = TRUE;

    RCC->CSR |= RCC_CSR_RMVF;

    if (((RCC->CSR & MCU_RESET_FLAG_MASK) != 0U) ||
        (deviceId != MCU_SUPPORTED_DEVICE_ID) ||
        (revisionId != MCU_SUPPORTED_REVISION_X_ID))
    {
        return E_NOT_OK;
    }

    if (Mcu_BootIdentityPrepare(rawResetFlags,
                                &StartupInfo->bootIdentity) != E_OK)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

Std_ReturnType Mcu_ClockBaselineInit(void)
{
    if (Mcu_IsResetClockBaselineValid() != TRUE)
    {
        return E_NOT_OK;
    }

    if (Mcu_ConfigureVoltageAndFlash() != E_OK)
    {
        return E_NOT_OK;
    }

    if (Mcu_StartHseAndPll() != E_OK)
    {
        return E_NOT_OK;
    }

    if (Mcu_SwitchToProductionClock() != E_OK)
    {
        return E_NOT_OK;
    }

    if (Mcu_IsProductionClockValid() != TRUE)
    {
        return E_NOT_OK;
    }

    return E_OK;
}

uint32 Mcu_GetClockFaultEvents(void)
{
    uint32 events = __atomic_load_n(&Mcu_ClockFaultEvents,
                                    __ATOMIC_ACQUIRE);

    MCU_TEST_CLOCK_FAULT_READ_HOOK();
    return events;
}

void NMI_Handler(void)
{
    MotorControlCdd_EmergencyOff();

    if ((RCC->CIFR & RCC_CIFR_CSSF) != 0U)
    {
        if (Mcu_ClearCssFaultFlag() != TRUE)
        {
            Mcu_EnterFatalException(MCU_FAULT_REASON_CSS_CLEAR_FAILED);
        }

        (void)__atomic_fetch_or(&Mcu_ClockFaultEvents,
                                MCU_CLOCK_FAULT_EVENT_CSS,
                                __ATOMIC_RELEASE);
        StateManager_NotifyCriticalFault(
            STATE_MANAGER_CRITICAL_CLASS_CLOCK);
        __DSB();
        return;
    }

    Mcu_EnterFatalException(MCU_FAULT_REASON_NMI_UNKNOWN);
}

void HardFault_Handler(void)
{
    Mcu_EnterFatalException(MCU_FAULT_REASON_HARDFAULT);
}

void MemManage_Handler(void)
{
    Mcu_EnterFatalException(MCU_FAULT_REASON_MEMMANAGE);
}

void BusFault_Handler(void)
{
    Mcu_EnterFatalException(MCU_FAULT_REASON_BUSFAULT);
}

void UsageFault_Handler(void)
{
    Mcu_EnterFatalException(MCU_FAULT_REASON_USAGEFAULT);
}

void Mcu_UnexpectedInterruptHandler(void)
{
    Mcu_EnterFatalException(MCU_FAULT_REASON_UNEXPECTED_VECTOR);
}
