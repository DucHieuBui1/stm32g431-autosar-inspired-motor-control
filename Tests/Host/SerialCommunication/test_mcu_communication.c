#include <stdio.h>
#include <string.h>

#include "stm32g431xx.h"

RCC_TypeDef McuCommunicationHost_Rcc;
USART_TypeDef McuCommunicationHost_Usart2;
uint32_t McuCommunicationHost_UidWords[3];
uint32_t SystemCoreClock;

static uint32_t Test_HookStage;
static uint32_t Test_CorruptAction;
static uint32_t Test_ResetAssertObserved;

#include "../../../Software/Bsw/Mcal/Mcu/Src/Mcu_Communication.c"

static unsigned int Test_Failures;

#define TEST_CHECK(condition)                                                   \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            (void)printf("FAIL:%u: %s\n", (unsigned int)__LINE__, #condition); \
            Test_Failures++;                                                    \
        }                                                                       \
    } while (0)

void McuCommunicationHost_TestHook(uint32_t stage)
{
    Test_HookStage = stage;
    if (stage == MCU_COMMUNICATION_TEST_RESET_ASSERTED)
    {
        Test_ResetAssertObserved =
            RCC->APB1RSTR1 & RCC_APB1RSTR1_USART2RST;
    }
    else if (stage == MCU_COMMUNICATION_TEST_AFTER_RECORD_WRITE)
    {
        if (Test_CorruptAction == 1U)
        {
            Mcu_BootIdentityRecord.lengthBytes++;
        }
        else if (Test_CorruptAction == 2U)
        {
            Mcu_BootIdentityRecord.rawResetFlags ^= RCC_CSR_PINRSTF;
        }
        else if (Test_CorruptAction == 3U)
        {
            Mcu_BootIdentityRecord.bootCounterInverse ^= 1U;
        }
        else if (Test_CorruptAction == 4U)
        {
            Mcu_BootIdentityRecord.crc32 ^= 1U;
        }
        else
        {
            /* No injected corruption. */
        }
    }
    else if ((stage == MCU_COMMUNICATION_TEST_AFTER_CLOCK_WRITE) &&
             (Test_CorruptAction == 5U))
    {
        RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN;
    }
    else
    {
        /* No action for other stages. */
    }
}

static void Test_Reset(void)
{
    (void)memset(&McuCommunicationHost_Rcc, 0,
                 sizeof(McuCommunicationHost_Rcc));
    (void)memset(&McuCommunicationHost_Usart2, 0,
                 sizeof(McuCommunicationHost_Usart2));
    (void)memset((void *)&Mcu_BootIdentityRecord, 0,
                 sizeof(Mcu_BootIdentityRecord));
    McuCommunicationHost_UidWords[0] = 0x11223344U;
    McuCommunicationHost_UidWords[1] = 0x55667788U;
    McuCommunicationHost_UidWords[2] = 0x99AABBCCU;
    SystemCoreClock = MCU_EXPECTED_HCLK_HZ;
    Mcu_SerialClockInitialized = FALSE;
    Test_HookStage = 0U;
    Test_CorruptAction = 0U;
    Test_ResetAssertObserved = 0U;
}

static void Test_WriteValidPriorRecord(
    uint32_t counter,
    uint32_t resetFlags)
{
    Mcu_BootIdentityRecordType record;

    record.magic = MCU_BOOT_IDENTITY_MAGIC;
    record.formatVersion = MCU_BOOT_IDENTITY_FORMAT_VERSION;
    record.lengthBytes = sizeof(Mcu_BootIdentityRecordType);
    record.rawResetFlags = resetFlags;
    record.bootCounter = counter;
    record.bootCounterInverse = ~counter;
    record.crc32 = Mcu_CommunicationRecordCrc32(&record);
    Mcu_BootIdentityRecord = record;
}

static void Test_BootIdentityLifecycle(void)
{
    uint32 firstIdentity = 0U;
    uint32 secondIdentity = 0U;
    uint32 firstCounter;

    Test_Reset();
    TEST_CHECK(Mcu_BootIdentityPrepare(RCC_CSR_PINRSTF,
                                       &firstIdentity) == E_OK);
    TEST_CHECK(firstIdentity != 0U);
    TEST_CHECK(Mcu_BootIdentityRecord.lengthBytes ==
               sizeof(Mcu_BootIdentityRecordType));
    TEST_CHECK(Mcu_BootIdentityRecord.rawResetFlags == RCC_CSR_PINRSTF);
    TEST_CHECK(Mcu_BootIdentityRecord.bootCounterInverse ==
               ~Mcu_BootIdentityRecord.bootCounter);
    firstCounter = Mcu_BootIdentityRecord.bootCounter;

    TEST_CHECK(Mcu_BootIdentityPrepare(RCC_CSR_IWDGRSTF,
                                       &secondIdentity) == E_OK);
    TEST_CHECK(Mcu_BootIdentityRecord.bootCounter == (firstCounter + 1U));
    TEST_CHECK(Mcu_BootIdentityRecord.rawResetFlags == RCC_CSR_IWDGRSTF);
    TEST_CHECK(secondIdentity != firstIdentity);

    Test_WriteValidPriorRecord(0xFFFFFFFFU, RCC_CSR_SFTRSTF);
    TEST_CHECK(Mcu_BootIdentityPrepare(RCC_CSR_SFTRSTF,
                                       &secondIdentity) == E_OK);
    TEST_CHECK(Mcu_BootIdentityRecord.bootCounter == 1U);
}

static void Test_BootIdentityRejectsInvalidInputsAndReadback(void)
{
    uint32 identity = 0U;
    uint32 action;

    Test_Reset();
    TEST_CHECK(Mcu_BootIdentityPrepare(RCC_CSR_PINRSTF,
                                       NULL_PTR) == E_NOT_OK);
    TEST_CHECK(Mcu_BootIdentityPrepare(1U, &identity) == E_NOT_OK);

    for (action = 1U; action <= 4U; action++)
    {
        Test_Reset();
        Test_CorruptAction = action;
        TEST_CHECK(Mcu_BootIdentityPrepare(RCC_CSR_BORRSTF,
                                           &identity) == E_NOT_OK);
    }

    Test_Reset();
    Test_WriteValidPriorRecord(41U, RCC_CSR_PINRSTF);
    Mcu_BootIdentityRecord.lengthBytes++;
    TEST_CHECK(Mcu_BootIdentityPrepare(RCC_CSR_PINRSTF,
                                       &identity) == E_OK);
    TEST_CHECK(Mcu_BootIdentityRecord.bootCounter != 42U);
}

static void Test_SerialClockOwnership(void)
{
    Test_Reset();
    RCC->AHB2RSTR = 0xA5A5A5A5U;
    RCC->CCIPR = RCC_CCIPR_USART2SEL | 0x80000000U;
    TEST_CHECK(Mcu_SerialPeripheralClockInit() == E_OK);
    TEST_CHECK((RCC->AHB2ENR & RCC_AHB2ENR_GPIOAEN) != 0U);
    TEST_CHECK((RCC->APB1ENR1 & RCC_APB1ENR1_USART2EN) != 0U);
    TEST_CHECK((RCC->CCIPR & RCC_CCIPR_USART2SEL) == 0U);
    TEST_CHECK((RCC->CCIPR & 0x80000000U) != 0U);
    TEST_CHECK(RCC->AHB2RSTR == 0xA5A5A5A5U);
    TEST_CHECK(Test_ResetAssertObserved == RCC_APB1RSTR1_USART2RST);
    TEST_CHECK(Mcu_SerialPeripheralClockInit() == E_NOT_OK);

    Test_Reset();
    SystemCoreClock--;
    TEST_CHECK(Mcu_SerialPeripheralClockInit() == E_NOT_OK);

    Test_Reset();
    USART2->CR1 = USART_CR1_UE;
    TEST_CHECK(Mcu_SerialPeripheralClockInit() == E_NOT_OK);

    Test_Reset();
    Test_CorruptAction = 5U;
    TEST_CHECK(Mcu_SerialPeripheralClockInit() == E_NOT_OK);
    TEST_CHECK(Test_HookStage == MCU_COMMUNICATION_TEST_AFTER_CLOCK_WRITE);
}

int main(void)
{
    Test_BootIdentityLifecycle();
    Test_BootIdentityRejectsInvalidInputsAndReadback();
    Test_SerialClockOwnership();

    if (Test_Failures == 0U)
    {
        (void)printf("PASS: Mcu serial clock and boot identity exact-source matrix\n");
        return 0;
    }

    return 1;
}
