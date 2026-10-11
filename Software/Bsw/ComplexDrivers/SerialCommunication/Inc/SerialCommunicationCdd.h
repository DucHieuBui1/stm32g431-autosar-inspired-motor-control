#ifndef SERIAL_COMMUNICATION_CDD_H
#define SERIAL_COMMUNICATION_CDD_H

#include "Std_Types.h"

#define SERIAL_COMMUNICATION_CDD_FRAME_LENGTH       28U
#define SERIAL_COMMUNICATION_CDD_REQUEST_MAGIC      0xA5U
#define SERIAL_COMMUNICATION_CDD_RESPONSE_MAGIC     0x5AU
#define SERIAL_COMMUNICATION_CDD_PROTOCOL_VERSION   0x01U
#define SERIAL_COMMUNICATION_CDD_MAX_LEASE_MS       100U
#define SERIAL_COMMUNICATION_CDD_CONFIGURED_LIMIT_MA ((sint32)0)
#define SERIAL_COMMUNICATION_CDD_MIN_COMMAND_INTERVAL_MS 1U

typedef enum
{
    SERIAL_COMMUNICATION_CDD_COMMAND_START = 0x01,
    SERIAL_COMMUNICATION_CDD_COMMAND_STOP = 0x02,
    SERIAL_COMMUNICATION_CDD_COMMAND_SET_CURRENT = 0x03,
    SERIAL_COMMUNICATION_CDD_COMMAND_DIAG_READ = 0x04
} SerialCommunicationCdd_CommandType;

typedef enum
{
    SERIAL_COMMUNICATION_CDD_STATUS_ACCEPTED = 0x0000,
    SERIAL_COMMUNICATION_CDD_STATUS_NOT_READY = 0x0001,
    SERIAL_COMMUNICATION_CDD_STATUS_INVALID_STATE = 0x0002,
    SERIAL_COMMUNICATION_CDD_STATUS_BUSY = 0x0003,
    SERIAL_COMMUNICATION_CDD_STATUS_INTERNAL = 0x0004,
    SERIAL_COMMUNICATION_CDD_STATUS_CHALLENGE = 0x0005
} SerialCommunicationCdd_StatusType;

typedef enum
{
    SERIAL_COMMUNICATION_CDD_DETAIL_NONE = 0x0000,
    SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_STATE = 0x0001,
    SERIAL_COMMUNICATION_CDD_DETAIL_CURRENT_LIMIT = 0x0002,
    SERIAL_COMMUNICATION_CDD_DETAIL_EXPIRED_LEASE = 0x0003,
    SERIAL_COMMUNICATION_CDD_DETAIL_LINK_DIAGNOSTICS = 0x0004,
    SERIAL_COMMUNICATION_CDD_DETAIL_RATE_LIMIT = 0x0005
} SerialCommunicationCdd_DetailType;

typedef struct
{
    SerialCommunicationCdd_CommandType command;
    uint32 sessionId;
    uint32 sequence;
    uint16 leaseMs;
    uint16 diagnosticSelector;
    sint32 currentMa;
    uint32 bootIdentity;
    uint32 receiveTick;
} SerialCommunicationCdd_RequestType;

typedef struct
{
    SerialCommunicationCdd_CommandType command;
    uint32 sessionId;
    uint32 sequence;
    SerialCommunicationCdd_StatusType status;
    SerialCommunicationCdd_DetailType detail;
    uint32 data0;
    uint32 data1;
} SerialCommunicationCdd_ResponseType;

typedef struct
{
    uint16 frameOrCrcRejects;
    uint16 identityOrSequenceRejects;
    uint16 usartErrorOrRingDrops;
    uint16 mailboxBusyOrResponseDrops;
} SerialCommunicationCdd_DiagnosticsType;

/**
 * Configure only PA2/PA3 and USART2 for the reviewed 115200-8-N-1 endpoint.
 * GPIOA and USART2 clocks and the USART2 kernel-clock selection are Mcu-owned
 * preconditions. BootIdentity must be the nonzero immutable startup identity.
 * Binding stays unavailable through a conservative 100 ms receive-drain
 * quarantine and a subsequent continuous 10 ms RX-idle interval on every
 * initialization; a received byte or USART error restarts the idle interval.
 */
Std_ReturnType SerialCommunicationCdd_Init(uint32 BootIdentity);

/** Read back every Serial CDD-owned static GPIO and USART field. */
Std_ReturnType SerialCommunicationCdd_CheckStaticConfiguration(void);

/**
 * StateTask entry. One call consumes at most 32 receive bytes and transmits at
 * most 32 response bytes. It never waits for a peripheral flag.
 */
void SerialCommunicationCdd_MainFunction(uint32 CurrentTick);

/** Category-1 USART2 receive/error ISR; it reads at most one byte. */
void USART2_IRQHandler(void);

/** Acquire the oldest immutable request. StateManager is the sole reader. */
Std_ReturnType SerialCommunicationCdd_RequestAcquire(
    const SerialCommunicationCdd_RequestType **Request);

/** Release the exact request pointer returned by RequestAcquire. */
Std_ReturnType SerialCommunicationCdd_RequestRelease(
    const SerialCommunicationCdd_RequestType *Request);

/**
 * Queue a StateManager-owned response without blocking or overwriting an
 * existing response. BUSY, CHALLENGE and RATE_LIMIT are transport-owned and
 * rejected.
 */
Std_ReturnType SerialCommunicationCdd_ResponseQueue(
    const SerialCommunicationCdd_ResponseType *Response);

/** Acquire the four saturating aggregate link counters. */
Std_ReturnType SerialCommunicationCdd_GetDiagnostics(
    SerialCommunicationCdd_DiagnosticsType *Diagnostics);

#endif /* SERIAL_COMMUNICATION_CDD_H */
