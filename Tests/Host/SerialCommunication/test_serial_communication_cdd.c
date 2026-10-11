#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "SerialCommunicationCdd.h"
#include "stm32g431xx.h"

GPIO_TypeDef SerialHost_GpioA;
USART_TypeDef SerialHost_Usart2;

#define TEST_BOOT_IDENTITY 0x13579BDFUL
#define TEST_SESSION       0x2468ACE1UL
#define TEST_ISR_RXNE      (1UL << 5U)
#define TEST_ISR_TXE       (1UL << 7U)
#define TEST_ISR_ORE       (1UL << 3U)

static uint32 Test_NvicPriority;
static uint32 Test_NvicClearCount;
static uint32 Test_NvicEnableCount;
static uint32 Test_InitCorruption;
static uint32 Test_DsbCount;
static uint32 Test_ConsumeErrorAtLength;
static uint32 Test_ConsumeErrorCount;
static uint8 Test_TxBytes[128];
static uint32 Test_TxCount;

#define TEST_CHECK(condition)                                             \
    do                                                                    \
    {                                                                     \
        if (!(condition))                                                 \
        {                                                                 \
            (void)printf("FAIL line %d: %s\n", __LINE__, #condition);     \
            return 1;                                                     \
        }                                                                 \
    } while (0)

void SerialHost_NvicSetPriority(IRQn_Type irq, uint32_t priority)
{
    if (irq == USART2_IRQn)
    {
        Test_NvicPriority = priority;
    }
}

void SerialHost_NvicClearPending(IRQn_Type irq)
{
    if (irq == USART2_IRQn)
    {
        Test_NvicClearCount++;
    }
}

void SerialHost_NvicEnable(IRQn_Type irq)
{
    if (irq == USART2_IRQn)
    {
        Test_NvicEnableCount++;
    }
}

uint32_t SerialHost_NvicGetPriority(IRQn_Type irq)
{
    return (irq == USART2_IRQn) ? Test_NvicPriority : UINT_MAX;
}

uint32_t SerialHost_NvicGetEnable(IRQn_Type irq)
{
    return ((irq == USART2_IRQn) && (Test_NvicEnableCount != 0U)) ? 1U : 0U;
}

void SerialHost_InitHook(void)
{
    if (Test_InitCorruption == 1U)
    {
        GPIOA->AFR[0] ^= (1UL << 8U);
    }
    else if (Test_InitCorruption == 2U)
    {
        USART2->BRR++;
    }
    else if (Test_InitCorruption == 3U)
    {
        Test_NvicPriority = 4U;
    }
    else if (Test_InitCorruption == 4U)
    {
        Test_NvicEnableCount = 0U;
    }
}

void SerialHost_RxClearHook(uint32_t mask)
{
    USART2->ISR &= ~mask;
}

void SerialHost_TxHook(uint32_t value)
{
    if (Test_TxCount < (uint32)sizeof(Test_TxBytes))
    {
        Test_TxBytes[Test_TxCount] = (uint8)value;
    }
    Test_TxCount++;
}

void SerialHost_ConsumeHook(uint32_t length)
{
    if ((Test_ConsumeErrorAtLength != 0U) &&
        (length == Test_ConsumeErrorAtLength) &&
        (Test_ConsumeErrorCount == 0U))
    {
        USART2->RDR = 0xEEU;
        USART2->ISR |= TEST_ISR_RXNE | TEST_ISR_ORE;
        USART2_IRQHandler();
        Test_ConsumeErrorCount++;
    }
}

void SerialHost_DsbHook(void)
{
    Test_DsbCount++;
}

#include "../../../Software/Bsw/ComplexDrivers/SerialCommunication/Src/SerialCommunicationCdd.c"

static void Test_WriteU16(uint8 *bytes, uint16 value)
{
    bytes[0] = (uint8)value;
    bytes[1] = (uint8)(value >> 8U);
}

static void Test_WriteU32(uint8 *bytes, uint32 value)
{
    bytes[0] = (uint8)value;
    bytes[1] = (uint8)(value >> 8U);
    bytes[2] = (uint8)(value >> 16U);
    bytes[3] = (uint8)(value >> 24U);
}

static uint16 Test_ReadU16(const uint8 *bytes)
{
    return (uint16)((uint16)bytes[0] | ((uint16)bytes[1] << 8U));
}

static uint32 Test_ReadU32(const uint8 *bytes)
{
    return (uint32)bytes[0] |
           ((uint32)bytes[1] << 8U) |
           ((uint32)bytes[2] << 16U) |
           ((uint32)bytes[3] << 24U);
}

static uint16 Test_Crc16(const uint8 *bytes, uint32 length)
{
    uint16 crc = 0xFFFFU;
    uint32 i;

    for (i = 0U; i < length; i++)
    {
        uint32 bit;
        crc ^= (uint16)((uint16)bytes[i] << 8U);
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16)((uint16)(crc << 1U) ^ 0x1021U) :
                  (uint16)(crc << 1U);
        }
    }
    return crc;
}

static void Test_BuildRequest(uint8 *frame,
                              uint8 command,
                              uint32 session,
                              uint32 sequence,
                              uint16 lease,
                              uint16 selector,
                              sint32 current,
                              uint32 bootIdentity)
{
    (void)memset(frame, 0, SERIAL_COMMUNICATION_CDD_FRAME_LENGTH);
    frame[0] = SERIAL_COMMUNICATION_CDD_REQUEST_MAGIC;
    frame[1] = SERIAL_COMMUNICATION_CDD_PROTOCOL_VERSION;
    frame[2] = command;
    frame[3] = SERIAL_COMMUNICATION_CDD_FRAME_LENGTH;
    Test_WriteU32(&frame[4], session);
    Test_WriteU32(&frame[8], sequence);
    Test_WriteU16(&frame[12], lease);
    Test_WriteU16(&frame[14], selector);
    Test_WriteU32(&frame[16], (uint32)current);
    Test_WriteU32(&frame[20], bootIdentity);
    Test_WriteU16(&frame[26], Test_Crc16(frame, 26U));
}

static void Test_InjectByte(uint8 byte)
{
    USART2->RDR = byte;
    USART2->ISR |= TEST_ISR_RXNE;
    USART2_IRQHandler();
}

static void Test_InjectFrame(const uint8 *frame, uint32 tick)
{
    uint32 i;
    for (i = 0U; i < SERIAL_COMMUNICATION_CDD_FRAME_LENGTH; i++)
    {
        Test_InjectByte(frame[i]);
    }
    SerialCommunicationCdd_MainFunction(tick);
}

static int Test_ResetHardwareAndInit(void)
{
    (void)memset(&SerialHost_GpioA, 0, sizeof(SerialHost_GpioA));
    (void)memset(&SerialHost_Usart2, 0, sizeof(SerialHost_Usart2));
    GPIOA->MODER = 0xA5A5A5A5UL;
    GPIOA->OTYPER = 0xFFFFFFFFUL;
    GPIOA->OSPEEDR = 0xFFFFFFFFUL;
    GPIOA->PUPDR = 0xFFFFFFFFUL;
    GPIOA->AFR[0] = 0x5A5A5A5AUL;
    USART2->ISR = TEST_ISR_TXE;
    Test_NvicPriority = UINT_MAX;
    Test_NvicClearCount = 0U;
    Test_NvicEnableCount = 0U;
    Test_InitCorruption = 0U;
    Test_DsbCount = 0U;
    Test_ConsumeErrorAtLength = 0U;
    Test_ConsumeErrorCount = 0U;
    Test_TxCount = 0U;
    (void)memset(Test_TxBytes, 0, sizeof(Test_TxBytes));
    return (SerialCommunicationCdd_Init(TEST_BOOT_IDENTITY) == E_OK) ? 0 : 1;
}

static int Test_ResetAndInit(void)
{
    if (Test_ResetHardwareAndInit() != 0)
    {
        return 1;
    }
    SerialCommunicationCdd_MainFunction(0U);
    SerialCommunicationCdd_MainFunction(100U);
    SerialCommunicationCdd_MainFunction(110U);
    return (SerialCdd_QuarantineReleased == TRUE) ? 0 : 1;
}

static int Test_BindSession(uint32 tick)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;

    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0, 0U);
    Test_InjectFrame(frame, tick);
    TEST_CHECK(SerialCdd_ResponseState == SERIAL_CDD_RESPONSE_READY);
    USART2->ISR |= TEST_ISR_TXE;
    SerialCommunicationCdd_MainFunction(tick + 1U);
    TEST_CHECK(SerialCdd_ResponseState == SERIAL_CDD_RESPONSE_FREE);
    Test_TxCount = 0U;
    (void)memset(Test_TxBytes, 0, sizeof(Test_TxBytes));

    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, tick + 2U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 0U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    return 0;
}

static int Test_ReceiveQuarantine(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;

    TEST_CHECK(Test_ResetHardwareAndInit() == 0);
    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0,
                      0U);
    {
        uint32 i;
        for (i = 0U; i < 28U; i++) { Test_InjectByte(frame[i]); }
    }
    SerialCommunicationCdd_MainFunction(1000U);
    TEST_CHECK(SerialCdd_QuarantineReleased == FALSE);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    TEST_CHECK(Test_TxCount == 0U);
    SerialCommunicationCdd_MainFunction(1099U);
    TEST_CHECK(SerialCdd_IdleStarted == FALSE);
    SerialCommunicationCdd_MainFunction(1100U);
    TEST_CHECK(SerialCdd_IdleStarted == TRUE);
    TEST_CHECK(SerialCdd_QuarantineReleased == FALSE);
    SerialCommunicationCdd_MainFunction(1109U);
    TEST_CHECK(SerialCdd_QuarantineReleased == FALSE);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    TEST_CHECK(Test_TxCount == 0U);
    SerialCommunicationCdd_MainFunction(1110U);
    TEST_CHECK(SerialCdd_QuarantineReleased == TRUE);

    Test_InjectFrame(frame, 1111U);
    TEST_CHECK(Test_TxCount == 0U);
    TEST_CHECK(SerialCdd_ParserLength == 0U);
    TEST_CHECK(SerialCdd_FrameRejects == 0U);
    TEST_CHECK(SerialCdd_IdentityRejects == 0U);
    TEST_CHECK(SerialCdd_ResponseState == SERIAL_CDD_RESPONSE_READY);
    SerialCommunicationCdd_MainFunction(1112U);
    TEST_CHECK(Test_TxCount == 28U);
    TEST_CHECK(Test_TxBytes[0] == SERIAL_COMMUNICATION_CDD_RESPONSE_MAGIC);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[12]) ==
               SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE);

    TEST_CHECK(Test_ResetHardwareAndInit() == 0);
    SerialCommunicationCdd_MainFunction(0U);
    SerialCommunicationCdd_MainFunction(99U);
    TEST_CHECK(SerialCdd_IdleStarted == FALSE);
    SerialCommunicationCdd_MainFunction(100U);
    TEST_CHECK(SerialCdd_IdleStarted == TRUE);
    SerialCommunicationCdd_MainFunction(105U);
    Test_InjectByte(0x55U);
    SerialCommunicationCdd_MainFunction(105U);
    TEST_CHECK(SerialCdd_IdleStartTick == 105U);
    SerialCommunicationCdd_MainFunction(114U);
    TEST_CHECK(SerialCdd_QuarantineReleased == FALSE);
    USART2->ISR |= TEST_ISR_ORE;
    USART2_IRQHandler();
    SerialCommunicationCdd_MainFunction(114U);
    TEST_CHECK(SerialCdd_IdleStartTick == 114U);
    SerialCommunicationCdd_MainFunction(123U);
    TEST_CHECK(SerialCdd_QuarantineReleased == FALSE);
    SerialCommunicationCdd_MainFunction(124U);
    TEST_CHECK(SerialCdd_QuarantineReleased == TRUE);
    return 0;
}

static int Test_InitAndReadback(void)
{
    uint32 moder;
    uint32 otyper;
    uint32 speed;
    uint32 pull;
    uint32 afr;

    (void)memset(&SerialHost_GpioA, 0, sizeof(SerialHost_GpioA));
    (void)memset(&SerialHost_Usart2, 0, sizeof(SerialHost_Usart2));
    TEST_CHECK(Test_Crc16((const uint8 *)"123456789", 9U) == 0x29B1U);
    TEST_CHECK(SerialCommunicationCdd_Init(0U) == E_NOT_OK);

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_NvicPriority == 3U);
    TEST_CHECK(Test_NvicClearCount == 1U);
    TEST_CHECK(Test_NvicEnableCount == 1U);
    TEST_CHECK(USART2->BRR == 1476U);
    TEST_CHECK(USART2->CR1 == 0x12DUL);
    TEST_CHECK(USART2->CR2 == 0U);
    TEST_CHECK(USART2->CR3 == 1U);
    TEST_CHECK(USART2->PRESC == 0U);
    TEST_CHECK(SerialCommunicationCdd_CheckStaticConfiguration() == E_OK);

    moder = GPIOA->MODER;
    otyper = GPIOA->OTYPER;
    speed = GPIOA->OSPEEDR;
    pull = GPIOA->PUPDR;
    afr = GPIOA->AFR[0];
    TEST_CHECK((moder & ~SERIAL_CDD_GPIO_MODE_MASK) ==
               (0xA5A5A5A5UL & ~SERIAL_CDD_GPIO_MODE_MASK));
    TEST_CHECK((otyper & ~SERIAL_CDD_GPIO_TYPE_MASK) ==
               (0xFFFFFFFFUL & ~SERIAL_CDD_GPIO_TYPE_MASK));
    TEST_CHECK((speed & ~SERIAL_CDD_GPIO_SPEED_MASK) ==
               (0xFFFFFFFFUL & ~SERIAL_CDD_GPIO_SPEED_MASK));
    TEST_CHECK((pull & ~SERIAL_CDD_GPIO_PULL_MASK) ==
               (0xFFFFFFFFUL & ~SERIAL_CDD_GPIO_PULL_MASK));
    TEST_CHECK((afr & ~SERIAL_CDD_GPIO_AF_MASK) ==
               (0x5A5A5A5AUL & ~SERIAL_CDD_GPIO_AF_MASK));

    GPIOA->PUPDR |= (1UL << 4U);
    TEST_CHECK(SerialCommunicationCdd_CheckStaticConfiguration() == E_NOT_OK);
    TEST_CHECK(Test_ResetAndInit() == 0);
    USART2->CR3 = 0U;
    TEST_CHECK(SerialCommunicationCdd_CheckStaticConfiguration() == E_NOT_OK);

    Test_InitCorruption = 1U;
    TEST_CHECK(SerialCommunicationCdd_Init(TEST_BOOT_IDENTITY) == E_NOT_OK);
    TEST_CHECK(USART2->CR1 == 0U);
    Test_InitCorruption = 2U;
    TEST_CHECK(SerialCommunicationCdd_Init(TEST_BOOT_IDENTITY) == E_NOT_OK);
    Test_InitCorruption = 3U;
    TEST_CHECK(SerialCommunicationCdd_Init(TEST_BOOT_IDENTITY) == E_NOT_OK);
    Test_InitCorruption = 4U;
    TEST_CHECK(SerialCommunicationCdd_Init(TEST_BOOT_IDENTITY) == E_NOT_OK);
    return 0;
}

static int Test_ChallengeBindAndResponse(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;
    SerialCommunicationCdd_ResponseType response;

    TEST_CHECK(Test_ResetAndInit() == 0);
    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 9U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0, 0U);
    Test_InjectFrame(frame, 10U);
    TEST_CHECK(Test_TxCount == 0U);
    SerialCommunicationCdd_MainFunction(11U);
    TEST_CHECK(Test_TxCount == SERIAL_COMMUNICATION_CDD_FRAME_LENGTH);
    TEST_CHECK(Test_TxBytes[0] == SERIAL_COMMUNICATION_CDD_RESPONSE_MAGIC);
    TEST_CHECK(Test_TxBytes[1] == SERIAL_COMMUNICATION_CDD_PROTOCOL_VERSION);
    TEST_CHECK(Test_TxBytes[2] == 0x84U);
    TEST_CHECK(Test_TxBytes[3] == SERIAL_COMMUNICATION_CDD_FRAME_LENGTH);
    TEST_CHECK(Test_ReadU32(&Test_TxBytes[4]) == TEST_SESSION);
    TEST_CHECK(Test_ReadU32(&Test_TxBytes[8]) == 0U);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[12]) ==
               SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[14]) == 0U);
    TEST_CHECK(Test_ReadU32(&Test_TxBytes[16]) == TEST_BOOT_IDENTITY);
    TEST_CHECK(Test_ReadU32(&Test_TxBytes[20]) == 0U);
    TEST_CHECK(Test_TxBytes[24] == 0U && Test_TxBytes[25] == 0U);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[26]) ==
               Test_Crc16(Test_TxBytes, 26U));
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);

    Test_BuildRequest(frame, 4U, TEST_SESSION + 1U, 0U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 12U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY + 1U);
    Test_InjectFrame(frame, 12U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    Test_BuildRequest(frame, 4U, TEST_SESSION, 1U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 12U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    Test_BuildRequest(frame, 1U, TEST_SESSION, 0U, 1U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 12U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    Test_BuildRequest(frame, 4U, 0U, 0U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 12U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);

    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 13U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request != NULL_PTR);
    TEST_CHECK(request->command == SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ);
    TEST_CHECK(request->sessionId == TEST_SESSION);
    TEST_CHECK(request->sequence == 0U);
    TEST_CHECK(request->receiveTick == 13U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_NOT_OK);

    response.command = SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ;
    response.sessionId = TEST_SESSION;
    response.sequence = 0U;
    response.status = SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED;
    response.detail = SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_STATE;
    response.data0 = 3U;
    response.data1 = 0xA5U;
    Test_TxCount = 0U;
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_OK);
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_NOT_OK);
    TEST_CHECK(SerialCdd_ResponseFrame[0] == 0x5AU);
    SerialCommunicationCdd_MainFunction(14U);
    TEST_CHECK(Test_TxCount == 28U);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[12]) == 0U);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[14]) == 1U);
    TEST_CHECK(Test_ReadU32(&Test_TxBytes[16]) == 3U);
    TEST_CHECK(Test_ReadU32(&Test_TxBytes[20]) == 0xA5U);

    response.status = SERIAL_COMMUNICATION_CDD_STATUS_BUSY;
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_NOT_OK);
    response.status = SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE;
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_NOT_OK);
    response.status = (SerialCommunicationCdd_StatusType)6;
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_NOT_OK);
    response.status = SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED;
    response.detail = (SerialCommunicationCdd_DetailType)5;
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_NOT_OK);
    response.detail = SERIAL_COMMUNICATION_CDD_DETAIL_NONE;
    response.command = (SerialCommunicationCdd_CommandType)5;
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_NOT_OK);
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(NULL_PTR) == E_NOT_OK);
    return 0;
}

static int Test_RejectedFramesDoNotAdvance(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;
    SerialCommunicationCdd_DiagnosticsType diagnostics;
    uint32 caseIndex;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);

    for (caseIndex = 0U; caseIndex < 7U; caseIndex++)
    {
        Test_BuildRequest(frame, 2U, TEST_SESSION, 1U, 0U, 0U, 0,
                          TEST_BOOT_IDENTITY);
        if (caseIndex == 0U) { frame[0] = 0xA4U; }
        if (caseIndex == 1U) { frame[1] = 2U; }
        if (caseIndex == 2U) { frame[2] = 5U; }
        if (caseIndex == 3U) { frame[3] = 27U; }
        if (caseIndex == 4U) { frame[24] = 1U; }
        if (caseIndex == 5U) { frame[25] = 1U; }
        Test_WriteU16(&frame[26], Test_Crc16(frame, 26U));
        if (caseIndex == 6U) { frame[26] ^= 1U; }
        Test_InjectFrame(frame, 2U + caseIndex);
        TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    }

    Test_BuildRequest(frame, 2U, 0U, 1U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 10U);
    Test_BuildRequest(frame, 2U, TEST_SESSION + 1U, 1U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 11U);
    Test_BuildRequest(frame, 2U, TEST_SESSION, 1U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY + 1U);
    Test_InjectFrame(frame, 12U);
    Test_BuildRequest(frame, 2U, TEST_SESSION, 2U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 13U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);

    Test_BuildRequest(frame, 2U, TEST_SESSION, 1U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 14U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 1U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);

    Test_BuildRequest(frame, 2U, TEST_SESSION, 1U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 15U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    Test_BuildRequest(frame, 2U, TEST_SESSION, 2U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 16U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);

    TEST_CHECK(SerialCommunicationCdd_GetDiagnostics(&diagnostics) == E_OK);
    TEST_CHECK(diagnostics.frameOrCrcRejects >= 7U);
    TEST_CHECK(diagnostics.identityOrSequenceRejects >= 5U);
    TEST_CHECK(SerialCommunicationCdd_GetDiagnostics(NULL_PTR) == E_NOT_OK);
    return 0;
}

static int Test_CommandFieldMatrix(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;
    uint32 sequence = 0U;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);

#define TEST_REJECT(commandValue, leaseValue, selectorValue, currentValue)    \
    do                                                                        \
    {                                                                         \
        Test_BuildRequest(frame, (commandValue), TEST_SESSION, sequence + 1U, \
                          (leaseValue), (selectorValue), (currentValue),       \
                          TEST_BOOT_IDENTITY);                                 \
        Test_InjectFrame(frame, 20U);                                          \
        TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) ==          \
                   E_NOT_OK);                                                  \
    } while (0)
#define TEST_ACCEPT(commandValue, leaseValue, selectorValue, currentValue)    \
    do                                                                        \
    {                                                                         \
        sequence++;                                                           \
        Test_BuildRequest(frame, (commandValue), TEST_SESSION, sequence,      \
                          (leaseValue), (selectorValue), (currentValue),       \
                          TEST_BOOT_IDENTITY);                                 \
        Test_InjectFrame(frame, 20U + sequence);                               \
        TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);   \
        TEST_CHECK(request->sequence == sequence);                             \
        TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);    \
    } while (0)

    TEST_REJECT(1U, 0U, 0U, 0);
    TEST_ACCEPT(1U, 1U, 0U, 0);
    TEST_ACCEPT(3U, 100U, 0U, 0);
    TEST_REJECT(3U, 101U, 0U, 0);
    TEST_REJECT(1U, 1U, 1U, 0);
    TEST_REJECT(1U, 1U, 0U, 1);
    TEST_REJECT(1U, 1U, 0U, -1);
    TEST_REJECT(3U, 1U, 0U, INT32_MIN);
    TEST_REJECT(3U, 1U, 0U, INT32_MAX);
    TEST_REJECT(2U, 1U, 0U, 0);
    TEST_REJECT(2U, 0U, 0U, 1);
    TEST_REJECT(2U, 0U, 1U, 0);
    TEST_ACCEPT(2U, 0U, 0U, 0);
    TEST_REJECT(4U, 1U, 0U, 0);
    TEST_REJECT(4U, 0U, 0U, 1);
    TEST_REJECT(4U, 0U, 2U, 0);
    TEST_ACCEPT(4U, 0U, 0U, 0);
    TEST_ACCEPT(4U, 0U, 1U, 0);

#undef TEST_REJECT
#undef TEST_ACCEPT
    return 0;
}

static int Test_MotorCommandRateBoundary(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);

    Test_BuildRequest(frame, 1U, TEST_SESSION, 1U, 1U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 50U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 1U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    TEST_CHECK(SerialCdd_MotorCommandRateEstablished == TRUE);
    TEST_CHECK(SerialCdd_LastAcceptedMotorCommandTick == 50U);

    Test_BuildRequest(frame, 3U, TEST_SESSION, 2U, 1U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 50U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    TEST_CHECK(SerialCdd_LastSequence == 1U);
    TEST_CHECK(SerialCdd_ResponseState == SERIAL_CDD_RESPONSE_READY);
    USART2->ISR |= TEST_ISR_TXE;
    SerialCommunicationCdd_MainFunction(51U);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[12]) ==
               SERIAL_COMMUNICATION_CDD_STATUS_INVALID_STATE);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[14]) ==
               SERIAL_COMMUNICATION_CDD_DETAIL_RATE_LIMIT);

    Test_TxCount = 0U;
    (void)memset(Test_TxBytes, 0, sizeof(Test_TxBytes));
    Test_InjectFrame(frame, 51U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 2U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    TEST_CHECK(SerialCdd_LastAcceptedMotorCommandTick == 51U);
    return 0;
}

static int Test_MotorCommandRateWrap(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);
    SerialCdd_MotorCommandRateEstablished = TRUE;
    SerialCdd_LastAcceptedMotorCommandTick = UINT32_MAX;
    Test_BuildRequest(frame, 1U, TEST_SESSION, 1U, 1U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 0U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 1U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    TEST_CHECK(SerialCdd_LastAcceptedMotorCommandTick == 0U);
    return 0;
}

static int Test_IsrRingAndParserBounds(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;
    SerialCommunicationCdd_DiagnosticsType diagnostics;
    uint32 i;

    TEST_CHECK(Test_ResetAndInit() == 0);
    USART2->ISR = 0U;
    USART2_IRQHandler();
    TEST_CHECK(Test_DsbCount == 1U);

    for (i = 0U; i < 63U; i++)
    {
        Test_InjectByte(0x11U);
    }
    TEST_CHECK(SerialCdd_RingHead == 63U);
    Test_InjectByte(0x22U);
    TEST_CHECK(SerialCdd_RingHead == 63U);
    SerialCommunicationCdd_MainFunction(1U);
    TEST_CHECK(SerialCdd_RingTail == SerialCdd_RingHead);
    TEST_CHECK(SerialCdd_ParserLength == 0U);

    for (i = 0U; i < 40U; i++)
    {
        Test_InjectByte(0x33U);
    }
    SerialCommunicationCdd_MainFunction(2U);
    TEST_CHECK(((SerialCdd_RingTail + 8U) & 63U) == SerialCdd_RingHead);
    SerialCommunicationCdd_MainFunction(3U);
    TEST_CHECK(SerialCdd_RingTail == SerialCdd_RingHead);

    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U, 0U, 0U, 0, 0U);
    for (i = 0U; i < 10U; i++)
    {
        Test_InjectByte(frame[i]);
    }
    SerialCommunicationCdd_MainFunction(4U);
    TEST_CHECK(SerialCdd_ParserLength == 10U);
    USART2->RDR = 0xEEU;
    USART2->ISR |= TEST_ISR_RXNE | TEST_ISR_ORE;
    USART2_IRQHandler();
    SerialCommunicationCdd_MainFunction(5U);
    TEST_CHECK(SerialCdd_ParserLength == 0U);
    for (i = 10U; i < 28U; i++)
    {
        Test_InjectByte(frame[i]);
    }
    SerialCommunicationCdd_MainFunction(6U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);

    TEST_CHECK(Test_BindSession(7U) == 0);
    TEST_CHECK(SerialCommunicationCdd_GetDiagnostics(&diagnostics) == E_OK);
    TEST_CHECK(diagnostics.usartErrorOrRingDrops == 2U);
    TEST_CHECK(Test_DsbCount ==
               (1U + 63U + 1U + 40U + 10U + 1U + 18U + 28U + 28U));
    return 0;
}

static int Test_MailboxBusyRetryAndOrdering(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *held[3];
    const SerialCommunicationCdd_RequestType *request;
    SerialCommunicationCdd_DiagnosticsType diagnostics;
    uint32 i;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(100U) == 0);

    for (i = 0U; i < 3U; i++)
    {
        Test_BuildRequest(frame, 2U, TEST_SESSION, i + 1U, 0U, 0U, 0,
                          TEST_BOOT_IDENTITY);
        Test_InjectFrame(frame, 103U + i);
        TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&held[i]) == E_OK);
    }
    Test_BuildRequest(frame, 2U, TEST_SESSION, 4U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 106U);
    TEST_CHECK(SerialCdd_LastSequence == 3U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    SerialCommunicationCdd_MainFunction(107U);
    TEST_CHECK(Test_ReadU16(&Test_TxBytes[12]) ==
               SERIAL_COMMUNICATION_CDD_STATUS_BUSY);
    TEST_CHECK(Test_ReadU32(&Test_TxBytes[8]) == 4U);

    TEST_CHECK(SerialCommunicationCdd_RequestRelease(held[1]) == E_OK);
    Test_InjectFrame(frame, 108U);
    TEST_CHECK(SerialCdd_LastSequence == 4U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 4U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(held[0]) == E_OK);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(held[2]) == E_OK);
    TEST_CHECK(SerialCommunicationCdd_GetDiagnostics(&diagnostics) == E_OK);
    TEST_CHECK(diagnostics.mailboxBusyOrResponseDrops == 1U);
    return 0;
}

static int Test_MultipleReadySlotsUseSequenceOrder(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *heldOne;
    const SerialCommunicationCdd_RequestType *heldTwo;
    const SerialCommunicationCdd_RequestType *heldThree;
    const SerialCommunicationCdd_RequestType *heldFour;
    const SerialCommunicationCdd_RequestType *request;
    uint32 sequence;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);
    for (sequence = 1U; sequence <= 3U; sequence++)
    {
        Test_BuildRequest(frame, 2U, TEST_SESSION, sequence,
                          0U, 0U, 0, TEST_BOOT_IDENTITY);
        Test_InjectFrame(frame, 10U + sequence);
    }

    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&heldOne) == E_OK);
    TEST_CHECK(heldOne->sequence == 1U);
    Test_BuildRequest(frame, 2U, TEST_SESSION, 4U,
                      0U, 0U, 0, TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 14U);
    TEST_CHECK(SerialCdd_LastSequence == 3U);
    TEST_CHECK(heldOne->sessionId == TEST_SESSION);
    TEST_CHECK(heldOne->sequence == 1U);
    TEST_CHECK(heldOne->command == SERIAL_COMMUNICATION_CDD_COMMAND_STOP);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 2U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    USART2->ISR |= TEST_ISR_TXE;
    SerialCommunicationCdd_MainFunction(15U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(heldOne) == E_OK);

    Test_InjectFrame(frame, 16U);
    TEST_CHECK(SerialCdd_LastSequence == 4U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&heldThree) == E_OK);
    TEST_CHECK(heldThree->sequence == 3U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&heldFour) == E_OK);
    TEST_CHECK(heldFour->sequence == 4U);

    TEST_CHECK(SerialCommunicationCdd_RequestRelease(heldThree) == E_OK);
    Test_BuildRequest(frame, 2U, TEST_SESSION, 5U,
                      0U, 0U, 0, TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 17U);
    TEST_CHECK(heldFour->sequence == 4U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&heldTwo) == E_OK);
    TEST_CHECK(heldTwo->sequence == 5U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(heldTwo) == E_OK);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(heldFour) == E_OK);
    return 0;
}

static int Test_WritingSlotIsExcludedAndPreserved(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);
    SerialCdd_RequestMailbox[0].request.sessionId = 0xDEADBEEFU;
    SerialCdd_RequestMailbox[0].request.sequence = 0xA5A5A5A5U;
    __atomic_store_n(&SerialCdd_RequestMailbox[0].state,
                     SERIAL_CDD_SLOT_WRITING,
                     __ATOMIC_RELEASE);

    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    Test_BuildRequest(frame, 2U, TEST_SESSION, 1U,
                      0U, 0U, 0, TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 10U);
    TEST_CHECK(SerialCdd_RequestMailbox[0].state == SERIAL_CDD_SLOT_WRITING);
    TEST_CHECK(SerialCdd_RequestMailbox[0].request.sessionId == 0xDEADBEEFU);
    TEST_CHECK(SerialCdd_RequestMailbox[0].request.sequence == 0xA5A5A5A5U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 1U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    __atomic_store_n(&SerialCdd_RequestMailbox[0].state,
                     SERIAL_CDD_SLOT_FREE,
                     __ATOMIC_RELEASE);
    return 0;
}

static int Test_IsrDuringConsumeInvalidatesFrame(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;

    TEST_CHECK(Test_ResetAndInit() == 0);
    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U,
                      0U, 0U, 0, 0U);
    Test_InjectFrame(frame, 1U);
    USART2->ISR |= TEST_ISR_TXE;
    SerialCommunicationCdd_MainFunction(2U);
    TEST_CHECK(SerialCdd_ChallengePending == TRUE);

    Test_BuildRequest(frame, 4U, TEST_SESSION, 0U,
                      0U, 0U, 0, TEST_BOOT_IDENTITY);
    {
        uint32 index;
        for (index = 0U; index < SERIAL_COMMUNICATION_CDD_FRAME_LENGTH;
             index++)
        {
            Test_InjectByte(frame[index]);
        }
    }
    Test_ConsumeErrorAtLength = 10U;
    SerialCommunicationCdd_MainFunction(3U);
    TEST_CHECK(Test_ConsumeErrorCount == 1U);
    TEST_CHECK(SerialCdd_Bound == FALSE);
    TEST_CHECK(SerialCdd_LastSequence == 0U);
    TEST_CHECK(SerialCdd_ParserLength == 0U);
    TEST_CHECK(SerialCdd_RingTail == SerialCdd_RingHead);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    return 0;
}

static int Test_ResponseBusyDefersReceive(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    SerialCommunicationCdd_ResponseType response;
    const SerialCommunicationCdd_RequestType *request;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);
    response.command = SERIAL_COMMUNICATION_CDD_COMMAND_STOP;
    response.sessionId = TEST_SESSION;
    response.sequence = 9U;
    response.status = SERIAL_COMMUNICATION_CDD_STATUS_INTERNAL;
    response.detail = SERIAL_COMMUNICATION_CDD_DETAIL_LINK_DIAGNOSTICS;
    response.data0 = 0x11223344UL;
    response.data1 = 0x55667788UL;
    TEST_CHECK(SerialCommunicationCdd_ResponseQueue(&response) == E_OK);
    Test_BuildRequest(frame, 2U, TEST_SESSION, 1U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    {
        uint32 i;
        for (i = 0U; i < 28U; i++) { Test_InjectByte(frame[i]); }
    }
    USART2->ISR &= ~TEST_ISR_TXE;
    SerialCommunicationCdd_MainFunction(1U);
    TEST_CHECK(Test_TxCount == 0U);
    TEST_CHECK(SerialCdd_RingTail != SerialCdd_RingHead);
    USART2->ISR |= TEST_ISR_TXE;
    SerialCommunicationCdd_MainFunction(2U);
    TEST_CHECK(Test_TxCount == 28U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);
    SerialCommunicationCdd_MainFunction(3U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_OK);
    TEST_CHECK(request->sequence == 1U);
    TEST_CHECK(SerialCommunicationCdd_RequestRelease(request) == E_OK);
    return 0;
}

static int Test_NoWrapAndCounterSaturation(void)
{
    uint8 frame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
    const SerialCommunicationCdd_RequestType *request;
    SerialCommunicationCdd_DiagnosticsType diagnostics;

    TEST_CHECK(Test_ResetAndInit() == 0);
    TEST_CHECK(Test_BindSession(0U) == 0);
    SerialCdd_LastSequence = UINT32_MAX;
    Test_BuildRequest(frame, 2U, TEST_SESSION, 0U, 0U, 0U, 0,
                      TEST_BOOT_IDENTITY);
    Test_InjectFrame(frame, 1U);
    TEST_CHECK(SerialCommunicationCdd_RequestAcquire(&request) == E_NOT_OK);

    __atomic_store_n(&SerialCdd_FrameRejects, 65535U, __ATOMIC_RELAXED);
    Test_InjectByte(0x00U);
    SerialCommunicationCdd_MainFunction(2U);
    __atomic_store_n(&SerialCdd_IdentityRejects, 65535U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_UsartDrops, 65535U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_MailboxDrops, 65535U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_ResponseDrops, 65535U, __ATOMIC_RELAXED);
    SerialCdd_SaturatingIncrement(&SerialCdd_IdentityRejects);
    SerialCdd_SaturatingIncrement(&SerialCdd_UsartDrops);
    TEST_CHECK(SerialCommunicationCdd_GetDiagnostics(&diagnostics) == E_OK);
    TEST_CHECK(diagnostics.frameOrCrcRejects == 65535U);
    TEST_CHECK(diagnostics.identityOrSequenceRejects == 65535U);
    TEST_CHECK(diagnostics.usartErrorOrRingDrops == 65535U);
    TEST_CHECK(diagnostics.mailboxBusyOrResponseDrops == 65535U);
    return 0;
}

int main(void)
{
    int result = 0;

    result |= Test_InitAndReadback();
    result |= Test_ReceiveQuarantine();
    result |= Test_ChallengeBindAndResponse();
    result |= Test_RejectedFramesDoNotAdvance();
    result |= Test_CommandFieldMatrix();
    result |= Test_MotorCommandRateBoundary();
    result |= Test_MotorCommandRateWrap();
    result |= Test_IsrRingAndParserBounds();
    result |= Test_MailboxBusyRetryAndOrdering();
    result |= Test_MultipleReadySlotsUseSequenceOrder();
    result |= Test_WritingSlotIsExcludedAndPreserved();
    result |= Test_IsrDuringConsumeInvalidatesFrame();
    result |= Test_ResponseBusyDefersReceive();
    result |= Test_NoWrapAndCounterSaturation();
    if (result == 0)
    {
        (void)printf("PASS: SerialCommunication CDD host matrix\n");
    }
    return result;
}
