/*
 * CAN1.h
 *
 *  Created on: Jan 16, 2023
 *      Author: Matt
 */

#ifndef INC_CAN1_H_
#define INC_CAN1_H_

#include "main.h"
#include "cmsis_os.h"

extern osMessageQueueId_t CAN1_QHandle;
extern osMessageQueueId_t CAN2_QHandle;
extern osMessageQueueId_t CANRX_QHandle;

#define CAN1_FLAG 0x00000001U
#define CAN2_FLAG 0x00000002U

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t aData[8];
} CANMsg;

// Incoming transactions should follow this format:
typedef struct {
    uint8_t id; // unique ID coming from whichever device is sending this message (used for acks or naks)
    uint8_t type; // ascii encoded type i.e. T for incoming Torque map
    uint8_t size; // size of incoming payload
    uint8_t reserved; // reserved for future use
    uint8_t params[4]; // type specific data
} Transaction_Header_Struct;

// Format for ack/nak
// LSB: ACK/NAK
#define CAN_TRANSACTION_ACK 0x00
#define CAN_TRANSACTION_NAK 0x01
// IF SUCCESS
// TODO: extra info for header ack/message ack, in progress/finished, waiting for more data, etc
// IF ERROR
// 2nd LSB: message or header invalid
#define CAN_TRANSACTION_HEADER_INVALID 0x00
#define CAN_TRANSACTION_MESSAGE_INVALID 0x02 
// Bits 3-8: Error specifics
#define CAN_TRANSACTION_BUSY (0x00 << 2)
#define CAN_TRANSACTION_INTERNAL_ERROR (0x01 << 2)
#define CAN_TRANSACTION_INVALID_PARAMS (0x02 << 2)
#define CAN_TRANSACTION_UNKNOWN_TYPE (0x03 << 2)
#define CAN_TRANSACTION_INACTIVE (0x04 << 2)
#define CAN_TRANSACTION_BUFFER_OVERRUN (0x05 << 2)

typedef struct {
    uint8_t id; // Unique ID of whichever device sent the current transaction header
    uint8_t flags; // Flags - if non-zero, header or message was invalid
    uint8_t reserved[6]; // reserved for future use
} Transaction_Response_Struct;

#define CAN_TRANSACTION_PAUSED 0x01
typedef struct {
	uint8_t buffer[255]; // Buffer size limited to size of uint8_t (255)
	uint8_t flags;
	uint32_t currentSize;
    Transaction_Header_Struct header;
} Transaction_Data_Struct;

extern Transaction_Data_Struct Transaction_Data;


void startCAN1TxTask();
void startCANRxTask();
void canMsgHandler(CAN_RxHeaderTypeDef *msgHeader, uint8_t msgData[]);
void initiateTransaction(CAN_RxHeaderTypeDef *msgHeader, uint8_t msgData[]);
void handleTransactionPacket(CAN_RxHeaderTypeDef *msgHeader, uint8_t msgData[]);

#endif /* INC_CAN1_H_ */
