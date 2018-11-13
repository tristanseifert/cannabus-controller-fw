/*
 * cannabus_private.h
 *
 * Private interfaces into the CANnabus driver
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */

#ifndef CANNABUS_CANNABUS_PRIVATE_H_
#define CANNABUS_CANNABUS_PRIVATE_H_

#include "cannabus.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/// stack size for CANnabus driver
#define kCANnabusTaskStackSize	100

/**
 * Internal state for the CANnabus driver.
 */
typedef struct {
	/// node address
	cannabus_addr_t address;

	/// device type
	uint8_t deviceType;
	/// firmware upgrade capabilities
	uint8_t fwUpgradeCapabilities;

	/// number of received frames
	unsigned int rxFrames;
	/// number of sent frames
	unsigned int txFrames;

	/// callbacks in user code to handle CAN bus interfacing
	cannabus_callbacks_t callbacks;

	/// FreeRTOS task handle
	TaskHandle_t task;
	/// task control block
	StaticTask_t taskTCB;
	/// stack for task
	StackType_t taskStack[kCANnabusTaskStackSize];
} cannabus_state_t;



/**
 * CANnabus task entry point
 */
void cannabus_task(void *ctx);



/**
 * Converts a received CAN frame into a CANnabus operation.
 */
int cannabus_conv_frame_to_op(cannabus_can_frame_t *frame, cannabus_operation_t *op);

/**
 * Converts a CANnabus operation into a CAN frame to be transmitted.
 */
int cannabus_conv_op_to_frame(cannabus_operation_t *op, cannabus_can_frame_t *frame);



/**
 * Is this CANnabus operation internal, e.g. is it something specified in the
 * CANnabus protocol that all nodes interpret, thus something we handle internal
 * to the protocol handler?
 */
bool cannabus_is_op_internal(cannabus_operation_t *op);

/**
 * Handles an internal CANnabus operation.
 */
int cannabus_internal_op(cannabus_operation_t *op);

/**
 * Handles reads/writes to the Device ID (0x0000) register.
 */
int cannabus_internal_reg_deviceid(cannabus_operation_t *op);

/**
 * Sends the device ID response.
 */
int cannabus_internal_reg_deviceid_respond(cannabus_operation_t *op);

#endif /* CANNABUS_CANNABUS_PRIVATE_H_ */
