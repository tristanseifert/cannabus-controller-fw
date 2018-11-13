/*
 * canbus_private.h
 *
 * Private internal functions for the CAN bus.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef CANBUS_PRIVATE_H_
#define CANBUS_PRIVATE_H_

#include "canbus.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include <stdbool.h>

/// size of the RX buffer
#define kCANRxBufferSize			8
/// size of the CAN task message buffer
#define kCANMsgBufferSize			4

/// size of the CAN bus driver stack (words)
#define kCANStackSize				70



/**
 * Message types for the CAN task
 */
typedef enum {
	/// transmits a frame
	kCANTaskMsgTransmit				= 0x01,
	/// CAN error interrupt
	kCANTaskHandleError				= 0x02,
} can_task_msg_type_t;

/**
 * Message to pass to the CAN task
 */
typedef struct {
	/// type of message (see can_task_msg_type_t)
	uint8_t type;

	/// various message parameters
	union {
		/// can message for transmission
		can_message_t txMessage;
		/// error register state (CAN->ESR)
		uint32_t canError;
	};
} can_task_msg_t;


/**
 * Internal state of the CAN bus driver
 */
typedef struct {
	/// CAN task message buffer
	can_task_msg_t taskMsgBuffer[kCANMsgBufferSize];
	/// CAN task message queue struct
	StaticQueue_t taskMsgQueueStruct;
	/// CAN task message queue
	QueueHandle_t taskMsgQueue;

	/// receive buffer
	can_message_t rxQueueBuffer[kCANRxBufferSize];
	/// receive queue struct
	StaticQueue_t rxQueueStruct;
	/// receive queue handle
	QueueHandle_t rxQueue;

	/// number of dropped messages
	unsigned int numDroppedMessages;

	/// FreeRTOS task handle
	TaskHandle_t task;
	/// task control block
	StaticTask_t taskTCB;
	/// stack for task
	StackType_t taskStack[kCANStackSize];
} can_state_t;

/**
 * Entry point for the CAN bus task.
 *
 * This task primarily handles receiving frames.
 */
void can_task(void *);

/**
 * Transmits a message.
 */
int can_task_tx_message(can_task_msg_t *msg);



/**
 * Declare prototype for the IRQ
 */
void CEC_CAN_IRQHandler(void);



/**
 * Reads a message from the specified FIFO.
 *
 * @note This function MAY be called from an ISR.
 *
 * @return Whether a higher priority task was woken.
 */
bool can_isr_read_fifo(int fifo);

/**
 * Checks whether the receive FIFOs were overrun.
 */
void can_isr_check_fifo_overrun(void);



/**
 * Finds the next available transmit mailbox.
 */
int can_find_free_tx_mailbox(void);

/**
 * Transmits the given message on the given mailbox.
 */
int can_tx_with_mailbox(int mailbox, can_message_t *msg);



#endif /* CANBUS_PRIVATE_H_ */
