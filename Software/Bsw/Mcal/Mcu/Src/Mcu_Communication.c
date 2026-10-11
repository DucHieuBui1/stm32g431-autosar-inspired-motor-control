#include "Mcu.h"
#include "Mcu_Communication.h"

#include "stm32g431xx.h"

#define MCU_BOOT_IDENTITY_MAGIC              0x4D425449U
#define MCU_BOOT_IDENTITY_FORMAT_VERSION     1U
#define MCU_EXPECTED_HCLK_HZ                 170000000U

#define MCU_COMMUNICATION_RESET_FLAG_MASK    \
    (RCC_CSR_OBLRSTF | RCC_CSR_PINRSTF | RCC_CSR_BORRSTF | \
     RCC_CSR_SFTRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF | \
     RCC_CSR_LPWRRSTF)

#define MCU_COMMUNICATION_UID_WORD_COUNT     3U

#ifndef MCU_COMMUNICATION_READ_UID_WORD
#define MCU_COMMUNICATION_READ_UID_WORD(index) \
    (((volatile const uint32 *)UID_BASE)[(index)])
#endif

#ifndef MCU_COMMUNICATION_TEST_HOOK
#define MCU_COMMUNICATION_TEST_HOOK(stage) ((void)(stage))
#endif

#define MCU_COMMUNICATION_TEST_AFTER_RECORD_WRITE  1U
#define MCU_COMMUNICATION_TEST_RESET_ASSERTED      2U
#define MCU_COMMUNICATION_TEST_AFTER_CLOCK_WRITE   3U

typedef struct
{
    uint32 magic;
    uint32 formatVersion;
    uint32 lengthBytes;
    uint32 rawResetFlags;
    uint32 bootCounter;
    uint32 bootCounterInverse;
    uint32 crc32;
} Mcu_BootIdentityRecordType;

STATIC boolean Mcu_SerialClockInitialized = FALSE;

volatile Mcu_BootIdentityRecordType Mcu_BootIdentityRecord
    __attribute__((section(".noinit.mcu_boot_identity"), used, aligned(4)));

_Static_assert(sizeof(Mcu_BootIdentityRecordType) == (7U * sizeof(uint32)),
               "boot identity record layout must remain seven words");

STATIC uint32 Mcu_CommunicationCrc32(
    const uint32 *words,
    uint32 wordCount)
{
    uint32 crc = 0xFFFFFFFFU;
    uint32 wordIndex;

    for (wordIndex = 0U; wordIndex < wordCount; wordIndex++)
    {
        uint32 data = words[wordIndex];
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

STATIC uint32 Mcu_CommunicationRecordCrc32(
    const Mcu_BootIdentityRecordType *record)
{
    uint32 words[6];

    words[0] = record->magic;
    words[1] = record->formatVersion;
    words[2] = record->lengthBytes;
    words[3] = record->rawResetFlags;
    words[4] = record->bootCounter;
    words[5] = record->bootCounterInverse;
    return Mcu_CommunicationCrc32(words, 6U);
}

STATIC void Mcu_CopyBootIdentityRecord(
    Mcu_BootIdentityRecordType *destination)
{
    destination->magic = Mcu_BootIdentityRecord.magic;
    destination->formatVersion = Mcu_BootIdentityRecord.formatVersion;
    destination->lengthBytes = Mcu_BootIdentityRecord.lengthBytes;
    destination->rawResetFlags = Mcu_BootIdentityRecord.rawResetFlags;
    destination->bootCounter = Mcu_BootIdentityRecord.bootCounter;
    destination->bootCounterInverse =
        Mcu_BootIdentityRecord.bootCounterInverse;
    destination->crc32 = Mcu_BootIdentityRecord.crc32;
}

STATIC boolean Mcu_IsBootIdentityRecordStructurallyValid(
    const Mcu_BootIdentityRecordType *record)
{
    if ((record->magic != MCU_BOOT_IDENTITY_MAGIC) ||
        (record->formatVersion != MCU_BOOT_IDENTITY_FORMAT_VERSION) ||
        (record->lengthBytes != sizeof(Mcu_BootIdentityRecordType)) ||
        ((record->rawResetFlags & ~MCU_COMMUNICATION_RESET_FLAG_MASK) != 0U) ||
        (record->bootCounter == 0U) ||
        (record->bootCounterInverse != ~record->bootCounter) ||
        (record->crc32 != Mcu_CommunicationRecordCrc32(record)))
    {
        return FALSE;
    }

    return TRUE;
}

STATIC uint32 Mcu_SeedBootCounter(void)
{
    uint32 seed = MCU_COMMUNICATION_READ_UID_WORD(0U) ^
                  (MCU_COMMUNICATION_READ_UID_WORD(1U) << 11U) ^
                  (MCU_COMMUNICATION_READ_UID_WORD(1U) >> 21U) ^
                  (MCU_COMMUNICATION_READ_UID_WORD(2U) << 22U) ^
                  (MCU_COMMUNICATION_READ_UID_WORD(2U) >> 10U);

    if (seed == 0U)
    {
        seed = 1U;
    }

    return seed;
}

Std_ReturnType Mcu_BootIdentityPrepare(
    uint32 RawResetFlags,
    uint32 *BootIdentity)
{
    Mcu_BootIdentityRecordType priorRecord;
    Mcu_BootIdentityRecordType currentRecord;
    uint32 identityWords[5];
    uint32 nextCounter;
    uint32 identity;

    if ((BootIdentity == NULL_PTR) ||
        ((RawResetFlags & ~MCU_COMMUNICATION_RESET_FLAG_MASK) != 0U))
    {
        return E_NOT_OK;
    }

    Mcu_CopyBootIdentityRecord(&priorRecord);
    if (Mcu_IsBootIdentityRecordStructurallyValid(&priorRecord) == TRUE)
    {
        nextCounter = priorRecord.bootCounter + 1U;
        if (nextCounter == 0U)
        {
            nextCounter = 1U;
        }
    }
    else
    {
        nextCounter = Mcu_SeedBootCounter();
    }

    currentRecord.magic = MCU_BOOT_IDENTITY_MAGIC;
    currentRecord.formatVersion = MCU_BOOT_IDENTITY_FORMAT_VERSION;
    currentRecord.lengthBytes = sizeof(Mcu_BootIdentityRecordType);
    currentRecord.rawResetFlags = RawResetFlags;
    currentRecord.bootCounter = nextCounter;
    currentRecord.bootCounterInverse = ~nextCounter;
    currentRecord.crc32 = Mcu_CommunicationRecordCrc32(&currentRecord);

    Mcu_BootIdentityRecord.magic = currentRecord.magic;
    Mcu_BootIdentityRecord.formatVersion = currentRecord.formatVersion;
    Mcu_BootIdentityRecord.lengthBytes = currentRecord.lengthBytes;
    Mcu_BootIdentityRecord.rawResetFlags = currentRecord.rawResetFlags;
    Mcu_BootIdentityRecord.bootCounter = currentRecord.bootCounter;
    Mcu_BootIdentityRecord.bootCounterInverse =
        currentRecord.bootCounterInverse;
    Mcu_BootIdentityRecord.crc32 = currentRecord.crc32;

    MCU_COMMUNICATION_TEST_HOOK(MCU_COMMUNICATION_TEST_AFTER_RECORD_WRITE);
    Mcu_CopyBootIdentityRecord(&currentRecord);
    if ((Mcu_IsBootIdentityRecordStructurallyValid(&currentRecord) != TRUE) ||
        (currentRecord.rawResetFlags != RawResetFlags) ||
        (currentRecord.bootCounter != nextCounter))
    {
        return E_NOT_OK;
    }

    identityWords[0] = MCU_COMMUNICATION_READ_UID_WORD(0U);
    identityWords[1] = MCU_COMMUNICATION_READ_UID_WORD(1U);
    identityWords[2] = MCU_COMMUNICATION_READ_UID_WORD(2U);
    identityWords[3] = currentRecord.bootCounter;
    identityWords[4] = currentRecord.rawResetFlags;
    identity = Mcu_CommunicationCrc32(identityWords, 5U);
    if (identity == 0U)
    {
        identity = MCU_BOOT_IDENTITY_MAGIC ^ currentRecord.bootCounter;
        if (identity == 0U)
        {
            identity = 1U;
        }
    }

    *BootIdentity = identity;
    return E_OK;
}

Std_ReturnType Mcu_SerialPeripheralClockInit(void)
{
    if ((Mcu_SerialClockInitialized == TRUE) ||
        (SystemCoreClock != MCU_EXPECTED_HCLK_HZ) ||
        ((USART2->CR1 & USART_CR1_UE) != 0U))
    {
        return E_NOT_OK;
    }

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL;

    RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST;
    MCU_COMMUNICATION_TEST_HOOK(MCU_COMMUNICATION_TEST_RESET_ASSERTED);
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;

    MCU_COMMUNICATION_TEST_HOOK(MCU_COMMUNICATION_TEST_AFTER_CLOCK_WRITE);
    if (((RCC->AHB2ENR & RCC_AHB2ENR_GPIOAEN) == 0U) ||
        ((RCC->APB1ENR1 & RCC_APB1ENR1_USART2EN) == 0U) ||
        ((RCC->APB1RSTR1 & RCC_APB1RSTR1_USART2RST) != 0U) ||
        ((RCC->CCIPR & RCC_CCIPR_USART2SEL) != 0U))
    {
        return E_NOT_OK;
    }

    Mcu_SerialClockInitialized = TRUE;
    return E_OK;
}
