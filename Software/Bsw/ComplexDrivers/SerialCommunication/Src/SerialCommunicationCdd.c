#include "SerialCommunicationCdd.h"
#include "stm32g431xx.h"

#include <limits.h>

#define SERIAL_CDD_GPIO_MODE_MASK   ((3UL << 4U) | (3UL << 6U))
#define SERIAL_CDD_GPIO_MODE_AF     ((2UL << 4U) | (2UL << 6U))
#define SERIAL_CDD_GPIO_TYPE_MASK   ((1UL << 2U) | (1UL << 3U))
#define SERIAL_CDD_GPIO_SPEED_MASK  ((3UL << 4U) | (3UL << 6U))
#define SERIAL_CDD_GPIO_PULL_MASK   ((3UL << 4U) | (3UL << 6U))
#define SERIAL_CDD_GPIO_AF_MASK     ((15UL << 8U) | (15UL << 12U))
#define SERIAL_CDD_GPIO_AF7         ((7UL << 8U) | (7UL << 12U))

#define SERIAL_CDD_USART_CR1_UE       (1UL << 0U)
#define SERIAL_CDD_USART_CR1_RE       (1UL << 2U)
#define SERIAL_CDD_USART_CR1_TE       (1UL << 3U)
#define SERIAL_CDD_USART_CR1_RXNEIE   (1UL << 5U)
#define SERIAL_CDD_USART_CR1_PEIE     (1UL << 8U)
#define SERIAL_CDD_USART_CR1_EXPECTED (SERIAL_CDD_USART_CR1_UE | \
                                        SERIAL_CDD_USART_CR1_RE | \
                                        SERIAL_CDD_USART_CR1_TE | \
                                        SERIAL_CDD_USART_CR1_RXNEIE | \
                                        SERIAL_CDD_USART_CR1_PEIE)
#define SERIAL_CDD_USART_CR2_EXPECTED 0UL
#define SERIAL_CDD_USART_CR3_EIE      (1UL << 0U)
#define SERIAL_CDD_USART_BRR          1476UL
#define SERIAL_CDD_USART_PRESC        0UL
#define SERIAL_CDD_USART_ISR_PE       (1UL << 0U)
#define SERIAL_CDD_USART_ISR_FE       (1UL << 1U)
#define SERIAL_CDD_USART_ISR_NE       (1UL << 2U)
#define SERIAL_CDD_USART_ISR_ORE      (1UL << 3U)
#define SERIAL_CDD_USART_ISR_RXNE     (1UL << 5U)
#define SERIAL_CDD_USART_ISR_TXE      (1UL << 7U)
#define SERIAL_CDD_USART_ERROR_MASK   (SERIAL_CDD_USART_ISR_PE | \
                                       SERIAL_CDD_USART_ISR_FE | \
                                       SERIAL_CDD_USART_ISR_NE | \
                                       SERIAL_CDD_USART_ISR_ORE)

#define SERIAL_CDD_RING_SIZE          64U
#define SERIAL_CDD_RING_MASK          (SERIAL_CDD_RING_SIZE - 1U)
#define SERIAL_CDD_TASK_BYTE_LIMIT    32U
#define SERIAL_CDD_MAILBOX_SLOTS      3U
#define SERIAL_CDD_COUNTER_MAX        65535U
#define SERIAL_CDD_QUARANTINE_MS      100U
#define SERIAL_CDD_IDLE_RELEASE_MS    10U

#define SERIAL_CDD_SLOT_FREE          0U
#define SERIAL_CDD_SLOT_WRITING       1U
#define SERIAL_CDD_SLOT_READY         2U
#define SERIAL_CDD_SLOT_READING       3U

#define SERIAL_CDD_RESPONSE_FREE      0U
#define SERIAL_CDD_RESPONSE_WRITING   1U
#define SERIAL_CDD_RESPONSE_READY     2U

#ifndef SERIAL_COMMUNICATION_CDD_TEST_INIT_HOOK
#define SERIAL_COMMUNICATION_CDD_TEST_INIT_HOOK() ((void)0)
#endif
#ifndef SERIAL_COMMUNICATION_CDD_TEST_RX_CLEAR_HOOK
#define SERIAL_COMMUNICATION_CDD_TEST_RX_CLEAR_HOOK(mask) ((void)(mask))
#endif
#ifndef SERIAL_COMMUNICATION_CDD_TEST_TX_HOOK
#define SERIAL_COMMUNICATION_CDD_TEST_TX_HOOK(value) ((void)(value))
#endif
#ifndef SERIAL_COMMUNICATION_CDD_TEST_CONSUME_HOOK
#define SERIAL_COMMUNICATION_CDD_TEST_CONSUME_HOOK(length) ((void)(length))
#endif

typedef struct
{
    _Alignas(4) volatile uint32 state;
    SerialCommunicationCdd_RequestType request;
} SerialCdd_RequestSlotType;

static _Alignas(4) volatile uint32 SerialCdd_RingHead;
static _Alignas(4) volatile uint32 SerialCdd_RingTail;
static uint8 SerialCdd_Ring[SERIAL_CDD_RING_SIZE];
static _Alignas(4) volatile uint32 SerialCdd_ParserResetEvent;
static _Alignas(4) volatile uint32 SerialCdd_RxActivity;
static uint8 SerialCdd_Parser[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
static uint32 SerialCdd_ParserLength;

static SerialCdd_RequestSlotType
    SerialCdd_RequestMailbox[SERIAL_CDD_MAILBOX_SLOTS];
static _Alignas(4) volatile uint32 SerialCdd_ResponseState;
static uint8 SerialCdd_ResponseFrame[SERIAL_COMMUNICATION_CDD_FRAME_LENGTH];
static uint32 SerialCdd_ResponseIndex;

static _Alignas(4) volatile uint32 SerialCdd_FrameRejects;
static _Alignas(4) volatile uint32 SerialCdd_IdentityRejects;
static _Alignas(4) volatile uint32 SerialCdd_UsartDrops;
static _Alignas(4) volatile uint32 SerialCdd_MailboxDrops;
static _Alignas(4) volatile uint32 SerialCdd_ResponseDrops;

static uint32 SerialCdd_BootIdentity;
static uint32 SerialCdd_ChallengeSession;
static uint32 SerialCdd_HostSession;
static uint32 SerialCdd_LastSequence;
static uint32 SerialCdd_LastAcceptedMotorCommandTick;
static boolean SerialCdd_ChallengePending;
static boolean SerialCdd_Bound;
static boolean SerialCdd_MotorCommandRateEstablished;
static boolean SerialCdd_Initialized;
static boolean SerialCdd_QuarantineStarted;
static boolean SerialCdd_IdleStarted;
static boolean SerialCdd_QuarantineReleased;
static uint32 SerialCdd_QuarantineStartTick;
static uint32 SerialCdd_IdleStartTick;
static uint32 SerialCdd_ObservedRxActivity;

_Static_assert((SERIAL_CDD_RING_SIZE & (SERIAL_CDD_RING_SIZE - 1U)) == 0U,
               "serial ring must remain a power of two");
_Static_assert(__atomic_always_lock_free(sizeof(uint32), 0),
               "32-bit atomics must be always lock-free");

static void SerialCdd_SaturatingIncrement(volatile uint32 *Counter)
{
    uint32 value = __atomic_load_n(Counter, __ATOMIC_RELAXED);

    if (value < SERIAL_CDD_COUNTER_MAX)
    {
        __atomic_store_n(Counter, value + 1U, __ATOMIC_RELEASE);
    }
}

static uint16 SerialCdd_LoadCounter(const volatile uint32 *Counter)
{
    uint32 value = __atomic_load_n(Counter, __ATOMIC_ACQUIRE);
    return (uint16)((value > SERIAL_CDD_COUNTER_MAX) ?
                    SERIAL_CDD_COUNTER_MAX : value);
}

static void SerialCdd_RecordRxActivity(void)
{
    uint32 activity = __atomic_load_n(&SerialCdd_RxActivity,
                                      __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_RxActivity,
                     activity + 1U,
                     __ATOMIC_RELEASE);
}

static boolean SerialCdd_QuarantineMainFunction(uint32 CurrentTick)
{
    uint32 head = __atomic_load_n(&SerialCdd_RingHead, __ATOMIC_ACQUIRE);
    uint32 activity = __atomic_load_n(&SerialCdd_RxActivity,
                                      __ATOMIC_ACQUIRE);

    SerialCdd_ParserLength = 0U;
    __atomic_store_n(&SerialCdd_RingTail, head, __ATOMIC_RELEASE);
    (void)__atomic_exchange_n(&SerialCdd_ParserResetEvent,
                              0U,
                              __ATOMIC_ACQUIRE);

    if (SerialCdd_QuarantineStarted == FALSE)
    {
        SerialCdd_QuarantineStartTick = CurrentTick;
        SerialCdd_ObservedRxActivity = activity;
        SerialCdd_QuarantineStarted = TRUE;
        return FALSE;
    }
    if ((uint32)(CurrentTick - SerialCdd_QuarantineStartTick) <
        SERIAL_CDD_QUARANTINE_MS)
    {
        SerialCdd_ObservedRxActivity = activity;
        return FALSE;
    }
    if (SerialCdd_IdleStarted == FALSE)
    {
        SerialCdd_IdleStartTick = CurrentTick;
        SerialCdd_ObservedRxActivity = activity;
        SerialCdd_IdleStarted = TRUE;
        return FALSE;
    }
    if (activity != SerialCdd_ObservedRxActivity)
    {
        SerialCdd_IdleStartTick = CurrentTick;
        SerialCdd_ObservedRxActivity = activity;
        return FALSE;
    }
    if ((uint32)(CurrentTick - SerialCdd_IdleStartTick) <
        SERIAL_CDD_IDLE_RELEASE_MS)
    {
        return FALSE;
    }
    SerialCdd_QuarantineReleased = TRUE;
    return TRUE;
}

static uint16 SerialCdd_ReadU16(const uint8 *Bytes)
{
    return (uint16)((uint16)Bytes[0] | ((uint16)Bytes[1] << 8U));
}

static uint32 SerialCdd_ReadU32(const uint8 *Bytes)
{
    return (uint32)Bytes[0] |
           ((uint32)Bytes[1] << 8U) |
           ((uint32)Bytes[2] << 16U) |
           ((uint32)Bytes[3] << 24U);
}

static sint32 SerialCdd_ReadS32(const uint8 *Bytes)
{
    uint32 raw = SerialCdd_ReadU32(Bytes);

    if (raw <= (uint32)INT32_MAX)
    {
        return (sint32)raw;
    }
    return (sint32)(raw - 0x80000000UL) + INT32_MIN;
}

static void SerialCdd_WriteU16(uint8 *Bytes, uint16 Value)
{
    Bytes[0] = (uint8)Value;
    Bytes[1] = (uint8)(Value >> 8U);
}

static void SerialCdd_WriteU32(uint8 *Bytes, uint32 Value)
{
    Bytes[0] = (uint8)Value;
    Bytes[1] = (uint8)(Value >> 8U);
    Bytes[2] = (uint8)(Value >> 16U);
    Bytes[3] = (uint8)(Value >> 24U);
}

static uint16 SerialCdd_Crc16(const uint8 *Bytes, uint32 Length)
{
    uint16 crc = 0xFFFFU;
    uint32 byteIndex;

    for (byteIndex = 0U; byteIndex < Length; byteIndex++)
    {
        uint32 bitIndex;
        crc ^= (uint16)((uint16)Bytes[byteIndex] << 8U);
        for (bitIndex = 0U; bitIndex < 8U; bitIndex++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16)((uint16)(crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16)(crc << 1U);
            }
        }
    }
    return crc;
}

static boolean SerialCdd_CommandValid(uint8 Command)
{
    return (boolean)(((Command >=
                       (uint8)SERIAL_COMMUNICATION_CDD_COMMAND_START) &&
                      (Command <=
                       (uint8)SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ)) ?
                     TRUE : FALSE);
}

static boolean SerialCdd_StatusValid(SerialCommunicationCdd_StatusType Status)
{
    return (boolean)(((uint32)Status <=
                      (uint32)SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE) ?
                     TRUE : FALSE);
}

static boolean SerialCdd_DetailValid(SerialCommunicationCdd_DetailType Detail)
{
    return (boolean)(((uint32)Detail <=
                      (uint32)SERIAL_COMMUNICATION_CDD_DETAIL_RATE_LIMIT) ?
                     TRUE : FALSE);
}

static Std_ReturnType SerialCdd_QueueResponseInternal(
    const SerialCommunicationCdd_ResponseType *Response,
    boolean TransportOwned)
{
    uint32 expected = SERIAL_CDD_RESPONSE_FREE;
    uint16 crc;

    if ((Response == NULL_PTR) ||
        (SerialCdd_CommandValid((uint8)Response->command) == FALSE) ||
        (SerialCdd_StatusValid(Response->status) == FALSE) ||
        (SerialCdd_DetailValid(Response->detail) == FALSE))
    {
        return E_NOT_OK;
    }
    if (TransportOwned == FALSE)
    {
        if ((Response->status == SERIAL_COMMUNICATION_CDD_STATUS_BUSY) ||
            (Response->status == SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE) ||
            (Response->detail ==
             SERIAL_COMMUNICATION_CDD_DETAIL_RATE_LIMIT))
        {
            return E_NOT_OK;
        }
    }
    else if ((Response->status != SERIAL_COMMUNICATION_CDD_STATUS_BUSY) &&
             (Response->status != SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE) &&
             ((Response->status !=
               SERIAL_COMMUNICATION_CDD_STATUS_INVALID_STATE) ||
              (Response->detail !=
               SERIAL_COMMUNICATION_CDD_DETAIL_RATE_LIMIT)))
    {
        return E_NOT_OK;
    }

    if (!__atomic_compare_exchange_n(&SerialCdd_ResponseState,
                                     &expected,
                                     SERIAL_CDD_RESPONSE_WRITING,
                                     FALSE,
                                     __ATOMIC_ACQUIRE,
                                     __ATOMIC_RELAXED))
    {
        SerialCdd_SaturatingIncrement(&SerialCdd_ResponseDrops);
        return E_NOT_OK;
    }

    SerialCdd_ResponseFrame[0] = SERIAL_COMMUNICATION_CDD_RESPONSE_MAGIC;
    SerialCdd_ResponseFrame[1] = SERIAL_COMMUNICATION_CDD_PROTOCOL_VERSION;
    SerialCdd_ResponseFrame[2] = (uint8)((uint8)Response->command | 0x80U);
    SerialCdd_ResponseFrame[3] = SERIAL_COMMUNICATION_CDD_FRAME_LENGTH;
    SerialCdd_WriteU32(&SerialCdd_ResponseFrame[4], Response->sessionId);
    SerialCdd_WriteU32(&SerialCdd_ResponseFrame[8], Response->sequence);
    SerialCdd_WriteU16(&SerialCdd_ResponseFrame[12], (uint16)Response->status);
    SerialCdd_WriteU16(&SerialCdd_ResponseFrame[14], (uint16)Response->detail);
    SerialCdd_WriteU32(&SerialCdd_ResponseFrame[16], Response->data0);
    SerialCdd_WriteU32(&SerialCdd_ResponseFrame[20], Response->data1);
    SerialCdd_ResponseFrame[24] = 0U;
    SerialCdd_ResponseFrame[25] = 0U;
    crc = SerialCdd_Crc16(SerialCdd_ResponseFrame, 26U);
    SerialCdd_WriteU16(&SerialCdd_ResponseFrame[26], crc);
    SerialCdd_ResponseIndex = 0U;
    __atomic_store_n(&SerialCdd_ResponseState,
                     SERIAL_CDD_RESPONSE_READY,
                     __ATOMIC_RELEASE);
    return E_OK;
}

static Std_ReturnType SerialCdd_QueueTransportResponse(
    const SerialCommunicationCdd_RequestType *Request,
    SerialCommunicationCdd_StatusType Status,
    SerialCommunicationCdd_DetailType Detail,
    uint32 Data0)
{
    SerialCommunicationCdd_ResponseType response;

    response.command = Request->command;
    response.sessionId = Request->sessionId;
    response.sequence = Request->sequence;
    response.status = Status;
    response.detail = Detail;
    response.data0 = Data0;
    response.data1 = 0U;
    return SerialCdd_QueueResponseInternal(&response, TRUE);
}

static SerialCdd_RequestSlotType *SerialCdd_ClaimRequestSlot(void)
{
    uint32 slotIndex;

    for (slotIndex = 0U; slotIndex < SERIAL_CDD_MAILBOX_SLOTS; slotIndex++)
    {
        uint32 expected = SERIAL_CDD_SLOT_FREE;
        if (__atomic_compare_exchange_n(
                &SerialCdd_RequestMailbox[slotIndex].state,
                &expected,
                SERIAL_CDD_SLOT_WRITING,
                FALSE,
                __ATOMIC_ACQUIRE,
                __ATOMIC_RELAXED))
        {
            return &SerialCdd_RequestMailbox[slotIndex];
        }
    }
    return NULL_PTR;
}

static boolean SerialCdd_AcceptRequest(
    const SerialCommunicationCdd_RequestType *Request,
    boolean Bind)
{
    SerialCdd_RequestSlotType *slot = SerialCdd_ClaimRequestSlot();

    if (slot == NULL_PTR)
    {
        SerialCdd_SaturatingIncrement(&SerialCdd_MailboxDrops);
        (void)SerialCdd_QueueTransportResponse(
            Request, SERIAL_COMMUNICATION_CDD_STATUS_BUSY,
            SERIAL_COMMUNICATION_CDD_DETAIL_NONE, 0U);
        return FALSE;
    }

    slot->request = *Request;
    if (Bind == TRUE)
    {
        SerialCdd_HostSession = Request->sessionId;
        SerialCdd_LastSequence = 0U;
        SerialCdd_Bound = TRUE;
    }
    else
    {
        SerialCdd_LastSequence = Request->sequence;
    }
    __atomic_store_n(&slot->state, SERIAL_CDD_SLOT_READY, __ATOMIC_RELEASE);
    return TRUE;
}

static boolean SerialCdd_ValidateCommandFields(
    const SerialCommunicationCdd_RequestType *Request)
{
    const sint32 limit = SERIAL_COMMUNICATION_CDD_CONFIGURED_LIMIT_MA;

    if ((Request->command == SERIAL_COMMUNICATION_CDD_COMMAND_START) ||
        (Request->command == SERIAL_COMMUNICATION_CDD_COMMAND_SET_CURRENT))
    {
        if ((Request->diagnosticSelector != 0U) ||
            (Request->leaseMs == 0U) ||
            (Request->leaseMs > SERIAL_COMMUNICATION_CDD_MAX_LEASE_MS) ||
            (Request->currentMa < -limit) ||
            (Request->currentMa > limit))
        {
            return FALSE;
        }
    }
    else if (Request->command == SERIAL_COMMUNICATION_CDD_COMMAND_STOP)
    {
        if ((Request->diagnosticSelector != 0U) ||
            (Request->leaseMs != 0U) || (Request->currentMa != 0))
        {
            return FALSE;
        }
    }
    else
    {
        if ((Request->diagnosticSelector > 1U) ||
            (Request->leaseMs != 0U) || (Request->currentMa != 0))
        {
            return FALSE;
        }
    }
    return TRUE;
}

static void SerialCdd_ProcessFrame(const uint8 *Frame, uint32 CurrentTick)
{
    SerialCommunicationCdd_RequestType request;
    uint16 receivedCrc;
    uint16 calculatedCrc;

    receivedCrc = SerialCdd_ReadU16(&Frame[26]);
    calculatedCrc = SerialCdd_Crc16(Frame, 26U);
    if ((Frame[0] != SERIAL_COMMUNICATION_CDD_REQUEST_MAGIC) ||
        (Frame[1] != SERIAL_COMMUNICATION_CDD_PROTOCOL_VERSION) ||
        (SerialCdd_CommandValid(Frame[2]) == FALSE) ||
        (Frame[3] != SERIAL_COMMUNICATION_CDD_FRAME_LENGTH) ||
        (Frame[24] != 0U) || (Frame[25] != 0U) ||
        (receivedCrc != calculatedCrc))
    {
        SerialCdd_SaturatingIncrement(&SerialCdd_FrameRejects);
        return;
    }

    request.command = (SerialCommunicationCdd_CommandType)Frame[2];
    request.sessionId = SerialCdd_ReadU32(&Frame[4]);
    request.sequence = SerialCdd_ReadU32(&Frame[8]);
    request.leaseMs = SerialCdd_ReadU16(&Frame[12]);
    request.diagnosticSelector = SerialCdd_ReadU16(&Frame[14]);
    request.currentMa = SerialCdd_ReadS32(&Frame[16]);
    request.bootIdentity = SerialCdd_ReadU32(&Frame[20]);
    request.receiveTick = CurrentTick;

    if (request.sessionId == 0U)
    {
        SerialCdd_SaturatingIncrement(&SerialCdd_IdentityRejects);
        return;
    }
    if (SerialCdd_ValidateCommandFields(&request) == FALSE)
    {
        SerialCdd_SaturatingIncrement(&SerialCdd_FrameRejects);
        return;
    }

    if (SerialCdd_Bound == FALSE)
    {
        if ((request.command != SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ) ||
            (request.diagnosticSelector != 0U) ||
            (request.sequence != 0U))
        {
            SerialCdd_SaturatingIncrement(&SerialCdd_IdentityRejects);
            return;
        }
        if (request.bootIdentity == 0U)
        {
            if (SerialCdd_QueueTransportResponse(
                    &request,
                    SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE,
                    SERIAL_COMMUNICATION_CDD_DETAIL_NONE,
                    SerialCdd_BootIdentity) == E_OK)
            {
                SerialCdd_ChallengeSession = request.sessionId;
                SerialCdd_ChallengePending = TRUE;
            }
            return;
        }
        if ((SerialCdd_ChallengePending != TRUE) ||
            (request.sessionId != SerialCdd_ChallengeSession) ||
            (request.bootIdentity != SerialCdd_BootIdentity))
        {
            SerialCdd_SaturatingIncrement(&SerialCdd_IdentityRejects);
            return;
        }
        if (SerialCdd_AcceptRequest(&request, TRUE) == TRUE)
        {
            SerialCdd_ChallengePending = FALSE;
        }
        return;
    }

    if ((request.sessionId != SerialCdd_HostSession) ||
        (request.bootIdentity != SerialCdd_BootIdentity) ||
        (SerialCdd_LastSequence == UINT32_MAX) ||
        (request.sequence != (SerialCdd_LastSequence + 1U)))
    {
        SerialCdd_SaturatingIncrement(&SerialCdd_IdentityRejects);
        return;
    }
    if (((request.command == SERIAL_COMMUNICATION_CDD_COMMAND_START) ||
         (request.command == SERIAL_COMMUNICATION_CDD_COMMAND_SET_CURRENT)) &&
        (SerialCdd_MotorCommandRateEstablished == TRUE) &&
        ((uint32)(CurrentTick - SerialCdd_LastAcceptedMotorCommandTick) <
         SERIAL_COMMUNICATION_CDD_MIN_COMMAND_INTERVAL_MS))
    {
        SerialCdd_SaturatingIncrement(&SerialCdd_FrameRejects);
        (void)SerialCdd_QueueTransportResponse(
            &request, SERIAL_COMMUNICATION_CDD_STATUS_INVALID_STATE,
            SERIAL_COMMUNICATION_CDD_DETAIL_RATE_LIMIT, 0U);
        return;
    }

    if (SerialCdd_AcceptRequest(&request, FALSE) == TRUE)
    {
        if ((request.command == SERIAL_COMMUNICATION_CDD_COMMAND_START) ||
            (request.command == SERIAL_COMMUNICATION_CDD_COMMAND_SET_CURRENT))
        {
            SerialCdd_LastAcceptedMotorCommandTick = CurrentTick;
            SerialCdd_MotorCommandRateEstablished = TRUE;
        }
    }
}

Std_ReturnType SerialCommunicationCdd_Init(uint32 BootIdentity)
{
    uint32 slotIndex;

    if (BootIdentity == 0U)
    {
        return E_NOT_OK;
    }

    SerialCdd_Initialized = FALSE;
    USART2->CR1 = 0U;
    GPIOA->MODER = (GPIOA->MODER & ~SERIAL_CDD_GPIO_MODE_MASK) |
                   SERIAL_CDD_GPIO_MODE_AF;
    GPIOA->OTYPER &= ~SERIAL_CDD_GPIO_TYPE_MASK;
    GPIOA->OSPEEDR &= ~SERIAL_CDD_GPIO_SPEED_MASK;
    GPIOA->PUPDR &= ~SERIAL_CDD_GPIO_PULL_MASK;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~SERIAL_CDD_GPIO_AF_MASK) |
                    SERIAL_CDD_GPIO_AF7;

    USART2->CR2 = SERIAL_CDD_USART_CR2_EXPECTED;
    USART2->CR3 = SERIAL_CDD_USART_CR3_EIE;
    USART2->BRR = SERIAL_CDD_USART_BRR;
    USART2->PRESC = SERIAL_CDD_USART_PRESC;
    USART2->ICR = SERIAL_CDD_USART_ERROR_MASK;
    SERIAL_COMMUNICATION_CDD_TEST_RX_CLEAR_HOOK(SERIAL_CDD_USART_ERROR_MASK);

    __atomic_store_n(&SerialCdd_RingHead, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_RingTail, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_ParserResetEvent, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_RxActivity, 0U, __ATOMIC_RELAXED);
    SerialCdd_ParserLength = 0U;
    __atomic_store_n(&SerialCdd_ResponseState,
                     SERIAL_CDD_RESPONSE_FREE,
                     __ATOMIC_RELAXED);
    SerialCdd_ResponseIndex = 0U;
    for (slotIndex = 0U; slotIndex < SERIAL_CDD_MAILBOX_SLOTS; slotIndex++)
    {
        __atomic_store_n(&SerialCdd_RequestMailbox[slotIndex].state,
                         SERIAL_CDD_SLOT_FREE,
                         __ATOMIC_RELAXED);
    }
    __atomic_store_n(&SerialCdd_FrameRejects, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_IdentityRejects, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_UsartDrops, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_MailboxDrops, 0U, __ATOMIC_RELAXED);
    __atomic_store_n(&SerialCdd_ResponseDrops, 0U, __ATOMIC_RELAXED);
    SerialCdd_BootIdentity = BootIdentity;
    SerialCdd_ChallengeSession = 0U;
    SerialCdd_HostSession = 0U;
    SerialCdd_LastSequence = 0U;
    SerialCdd_LastAcceptedMotorCommandTick = 0U;
    SerialCdd_ChallengePending = FALSE;
    SerialCdd_Bound = FALSE;
    SerialCdd_MotorCommandRateEstablished = FALSE;
    SerialCdd_QuarantineStarted = FALSE;
    SerialCdd_IdleStarted = FALSE;
    SerialCdd_QuarantineReleased = FALSE;
    SerialCdd_QuarantineStartTick = 0U;
    SerialCdd_IdleStartTick = 0U;
    SerialCdd_ObservedRxActivity = 0U;

    NVIC_SetPriority(USART2_IRQn, 3U);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    USART2->CR1 = SERIAL_CDD_USART_CR1_EXPECTED;
    SERIAL_COMMUNICATION_CDD_TEST_INIT_HOOK();

    SerialCdd_Initialized = TRUE;
    if (SerialCommunicationCdd_CheckStaticConfiguration() != E_OK)
    {
        SerialCdd_Initialized = FALSE;
        USART2->CR1 = 0U;
        return E_NOT_OK;
    }
    return E_OK;
}

Std_ReturnType SerialCommunicationCdd_CheckStaticConfiguration(void)
{
    if ((SerialCdd_Initialized == FALSE) ||
        ((GPIOA->MODER & SERIAL_CDD_GPIO_MODE_MASK) !=
         SERIAL_CDD_GPIO_MODE_AF) ||
        ((GPIOA->OTYPER & SERIAL_CDD_GPIO_TYPE_MASK) != 0U) ||
        ((GPIOA->OSPEEDR & SERIAL_CDD_GPIO_SPEED_MASK) != 0U) ||
        ((GPIOA->PUPDR & SERIAL_CDD_GPIO_PULL_MASK) != 0U) ||
        ((GPIOA->AFR[0] & SERIAL_CDD_GPIO_AF_MASK) !=
         SERIAL_CDD_GPIO_AF7) ||
        (USART2->CR1 != SERIAL_CDD_USART_CR1_EXPECTED) ||
        (USART2->CR2 != SERIAL_CDD_USART_CR2_EXPECTED) ||
        (USART2->CR3 != SERIAL_CDD_USART_CR3_EIE) ||
        (USART2->BRR != SERIAL_CDD_USART_BRR) ||
        (USART2->PRESC != SERIAL_CDD_USART_PRESC) ||
        (NVIC_GetPriority(USART2_IRQn) != 3U) ||
        (NVIC_GetEnableIRQ(USART2_IRQn) == 0U))
    {
        return E_NOT_OK;
    }
    return E_OK;
}

void USART2_IRQHandler(void)
{
    uint32 status = USART2->ISR;
    uint32 error = status & SERIAL_CDD_USART_ERROR_MASK;

    if (error != 0U)
    {
        if ((status & SERIAL_CDD_USART_ISR_RXNE) != 0U)
        {
            volatile uint32 discarded = USART2->RDR;
            (void)discarded;
        }
        USART2->ICR = error;
        SERIAL_COMMUNICATION_CDD_TEST_RX_CLEAR_HOOK(
            error | (status & SERIAL_CDD_USART_ISR_RXNE));
        SerialCdd_SaturatingIncrement(&SerialCdd_UsartDrops);
        SerialCdd_RecordRxActivity();
        __atomic_store_n(&SerialCdd_ParserResetEvent, 1U, __ATOMIC_RELEASE);
    }
    else if ((status & SERIAL_CDD_USART_ISR_RXNE) != 0U)
    {
        uint8 byte = (uint8)USART2->RDR;
        uint32 head = __atomic_load_n(&SerialCdd_RingHead,
                                      __ATOMIC_RELAXED);
        uint32 next = (head + 1U) & SERIAL_CDD_RING_MASK;
        uint32 tail = __atomic_load_n(&SerialCdd_RingTail,
                                      __ATOMIC_ACQUIRE);

        SERIAL_COMMUNICATION_CDD_TEST_RX_CLEAR_HOOK(
            SERIAL_CDD_USART_ISR_RXNE);
        SerialCdd_RecordRxActivity();
        if (next == tail)
        {
            SerialCdd_SaturatingIncrement(&SerialCdd_UsartDrops);
            __atomic_store_n(&SerialCdd_ParserResetEvent,
                             1U,
                             __ATOMIC_RELEASE);
        }
        else
        {
            SerialCdd_Ring[head] = byte;
            __atomic_store_n(&SerialCdd_RingHead, next, __ATOMIC_RELEASE);
        }
    }
    else
    {
        /* A spurious USART2 entry has no transport effect. */
    }
    __DSB();
}

void SerialCommunicationCdd_MainFunction(uint32 CurrentTick)
{
    uint32 count;

    if (SerialCdd_Initialized == FALSE)
    {
        return;
    }

    if (SerialCdd_QuarantineReleased == FALSE)
    {
        (void)SerialCdd_QuarantineMainFunction(CurrentTick);
        return;
    }

    if (__atomic_load_n(&SerialCdd_ResponseState, __ATOMIC_ACQUIRE) ==
        SERIAL_CDD_RESPONSE_READY)
    {
        for (count = 0U;
             (count < SERIAL_CDD_TASK_BYTE_LIMIT) &&
             (SerialCdd_ResponseIndex <
              SERIAL_COMMUNICATION_CDD_FRAME_LENGTH) &&
             ((USART2->ISR & SERIAL_CDD_USART_ISR_TXE) != 0U);
             count++)
        {
            uint8 value = SerialCdd_ResponseFrame[SerialCdd_ResponseIndex];
            USART2->TDR = value;
            SERIAL_COMMUNICATION_CDD_TEST_TX_HOOK(value);
            SerialCdd_ResponseIndex++;
        }
        if (SerialCdd_ResponseIndex == SERIAL_COMMUNICATION_CDD_FRAME_LENGTH)
        {
            SerialCdd_ResponseIndex = 0U;
            __atomic_store_n(&SerialCdd_ResponseState,
                             SERIAL_CDD_RESPONSE_FREE,
                             __ATOMIC_RELEASE);
        }
        return;
    }

    if (__atomic_exchange_n(&SerialCdd_ParserResetEvent,
                            0U,
                            __ATOMIC_ACQUIRE) != 0U)
    {
        uint32 head = __atomic_load_n(&SerialCdd_RingHead, __ATOMIC_ACQUIRE);
        SerialCdd_ParserLength = 0U;
        __atomic_store_n(&SerialCdd_RingTail, head, __ATOMIC_RELEASE);
        return;
    }

    for (count = 0U; count < SERIAL_CDD_TASK_BYTE_LIMIT; count++)
    {
        uint32 tail = __atomic_load_n(&SerialCdd_RingTail,
                                      __ATOMIC_RELAXED);
        uint32 head = __atomic_load_n(&SerialCdd_RingHead,
                                      __ATOMIC_ACQUIRE);
        uint8 byte;

        if (tail == head)
        {
            break;
        }
        byte = SerialCdd_Ring[tail];
        __atomic_store_n(&SerialCdd_RingTail,
                         (tail + 1U) & SERIAL_CDD_RING_MASK,
                         __ATOMIC_RELEASE);

        if ((SerialCdd_ParserLength == 0U) &&
            (byte != SERIAL_COMMUNICATION_CDD_REQUEST_MAGIC))
        {
            SerialCdd_SaturatingIncrement(&SerialCdd_FrameRejects);
            continue;
        }
        SerialCdd_Parser[SerialCdd_ParserLength] = byte;
        SerialCdd_ParserLength++;
        SERIAL_COMMUNICATION_CDD_TEST_CONSUME_HOOK(SerialCdd_ParserLength);
        if (SerialCdd_ParserLength == SERIAL_COMMUNICATION_CDD_FRAME_LENGTH)
        {
            if (__atomic_exchange_n(&SerialCdd_ParserResetEvent,
                                    0U,
                                    __ATOMIC_ACQUIRE) != 0U)
            {
                head = __atomic_load_n(&SerialCdd_RingHead,
                                       __ATOMIC_ACQUIRE);
                SerialCdd_ParserLength = 0U;
                __atomic_store_n(&SerialCdd_RingTail,
                                 head,
                                 __ATOMIC_RELEASE);
                break;
            }
            SerialCdd_ProcessFrame(SerialCdd_Parser, CurrentTick);
            SerialCdd_ParserLength = 0U;
            if (__atomic_load_n(&SerialCdd_ResponseState,
                                __ATOMIC_ACQUIRE) !=
                SERIAL_CDD_RESPONSE_FREE)
            {
                break;
            }
        }
    }
}

Std_ReturnType SerialCommunicationCdd_RequestAcquire(
    const SerialCommunicationCdd_RequestType **Request)
{
    uint32 slotIndex;
    uint32 selected = SERIAL_CDD_MAILBOX_SLOTS;
    uint32 selectedSequence = UINT32_MAX;

    if ((Request == NULL_PTR) || (SerialCdd_Initialized == FALSE))
    {
        return E_NOT_OK;
    }
    *Request = NULL_PTR;

    for (slotIndex = 0U; slotIndex < SERIAL_CDD_MAILBOX_SLOTS; slotIndex++)
    {
        if (__atomic_load_n(&SerialCdd_RequestMailbox[slotIndex].state,
                            __ATOMIC_ACQUIRE) == SERIAL_CDD_SLOT_READY)
        {
            uint32 sequence =
                SerialCdd_RequestMailbox[slotIndex].request.sequence;
            if ((selected == SERIAL_CDD_MAILBOX_SLOTS) ||
                (sequence < selectedSequence))
            {
                selected = slotIndex;
                selectedSequence = sequence;
            }
        }
    }
    if (selected < SERIAL_CDD_MAILBOX_SLOTS)
    {
        uint32 expected = SERIAL_CDD_SLOT_READY;
        if (__atomic_compare_exchange_n(
                &SerialCdd_RequestMailbox[selected].state,
                &expected,
                SERIAL_CDD_SLOT_READING,
                FALSE,
                __ATOMIC_ACQUIRE,
                __ATOMIC_RELAXED))
        {
            *Request = &SerialCdd_RequestMailbox[selected].request;
            return E_OK;
        }
    }
    return E_NOT_OK;
}

Std_ReturnType SerialCommunicationCdd_RequestRelease(
    const SerialCommunicationCdd_RequestType *Request)
{
    uint32 slotIndex;

    if (Request == NULL_PTR)
    {
        return E_NOT_OK;
    }
    for (slotIndex = 0U; slotIndex < SERIAL_CDD_MAILBOX_SLOTS; slotIndex++)
    {
        SerialCdd_RequestSlotType *slot =
            &SerialCdd_RequestMailbox[slotIndex];
        if (&slot->request == Request)
        {
            uint32 expected = SERIAL_CDD_SLOT_READING;
            if (__atomic_compare_exchange_n(&slot->state,
                                            &expected,
                                            SERIAL_CDD_SLOT_FREE,
                                            FALSE,
                                            __ATOMIC_RELEASE,
                                            __ATOMIC_RELAXED))
            {
                return E_OK;
            }
            return E_NOT_OK;
        }
    }
    return E_NOT_OK;
}

Std_ReturnType SerialCommunicationCdd_ResponseQueue(
    const SerialCommunicationCdd_ResponseType *Response)
{
    if (SerialCdd_Initialized == FALSE)
    {
        return E_NOT_OK;
    }
    return SerialCdd_QueueResponseInternal(Response, FALSE);
}

Std_ReturnType SerialCommunicationCdd_GetDiagnostics(
    SerialCommunicationCdd_DiagnosticsType *Diagnostics)
{
    uint32 mailboxAggregate;

    if (Diagnostics == NULL_PTR)
    {
        return E_NOT_OK;
    }
    Diagnostics->frameOrCrcRejects =
        SerialCdd_LoadCounter(&SerialCdd_FrameRejects);
    Diagnostics->identityOrSequenceRejects =
        SerialCdd_LoadCounter(&SerialCdd_IdentityRejects);
    Diagnostics->usartErrorOrRingDrops =
        SerialCdd_LoadCounter(&SerialCdd_UsartDrops);
    mailboxAggregate =
        (uint32)SerialCdd_LoadCounter(&SerialCdd_MailboxDrops) +
        (uint32)SerialCdd_LoadCounter(&SerialCdd_ResponseDrops);
    Diagnostics->mailboxBusyOrResponseDrops =
        (uint16)((mailboxAggregate > SERIAL_CDD_COUNTER_MAX) ?
                 SERIAL_CDD_COUNTER_MAX : mailboxAggregate);
    return E_OK;
}
