/*
 * cannabus.c
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */
#include "cannabus.h"
#include "cannabus_private.h"

#include "controller.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

/// CANnabus version implemented by this driver
const uint8_t kCannabusVersion = 0x08;

/// Internal state of the CANnabus driver
static cannabus_state_t gCANnabusState;



/**
 * Initializes the CANnabus and sets this node's address.
 */
int cannabus_init(cannabus_addr_t addr, const cannabus_callbacks_t *callbacks) {
	int err;

	// first, clear all state and copy callbacks
	memset(&gCANnabusState, 0, sizeof(gCANnabusState));
	memcpy(&gCANnabusState.callbacks, callbacks, sizeof(cannabus_callbacks_t));

	// create the task
	gCANnabusState.task = xTaskCreateStatic(cannabus_task, "CANnabus",
			kCANnabusTaskStackSize, NULL, kTaskPriorityCANnabus,
			(void *) &gCANnabusState.taskStack, &gCANnabusState.taskTCB);

	if(gCANnabusState.task == NULL) {
		return kErrTaskCreationFailed;
	}

	// set filter for the broadcast address
	err = gCANnabusState.callbacks.can_config_filter(1, 0x07FFF800, (0xFFFF << 11));

	if(err < kErrSuccess) {
		LOG("couldn't set broadcast filter: %d\n", err);
		return err;
	}

	// then, set the node address and device type
	err = cannabus_set_address(addr);

	if(err < kErrSuccess) {
		LOG("couldn't assign id %x: %d\n", addr, err);
		return err;
	}

	// set filter that will match any address (TODO: will this work?)
	err = gCANnabusState.callbacks.can_config_filter(1, 0x00000000, 0);

	if(err < kErrSuccess) {
		LOG("couldn't set wildcard filter: %d\n", err);
		return err;
	}

	// start the CAN bus
	err = gCANnabusState.callbacks.can_init();
	return err;
}

/**
 * Changes the node's address.
 */
int cannabus_set_address(cannabus_addr_t addr) {
	int err;

	// ensure it's not the broadcast address
	if(addr == 0xFFFF) {
		return kErrInvalidArgs;
	}

	// set address in state
	gCANnabusState.address = addr;
	LOG("CANnabus address: 0x%04x\n", addr);

	// update the filters on the CAN peripheral
	err = gCANnabusState.callbacks.can_config_filter(2, 0x07FFF800, (uint32_t) (addr << 11));

	return err;
}



/**
 * CANnabus task entry point
 */
__attribute__((noreturn)) void cannabus_task( __attribute__((unused)) void *ctx) {
	int err;
	cannabus_can_frame_t frame;
	cannabus_operation_t op;

	while(1) {
		// dequeue a message
		err = gCANnabusState.callbacks.can_rx_message(&frame);

		if(err < kErrSuccess) {
			LOG("can_rx_message: %d\n", err);
			continue;
		} else {
			gCANnabusState.rxFrames++;
		}

		// convert this message into a CANnabus operation
		err = cannabus_conv_frame_to_op(&frame, &op);

		if(err < kErrSuccess) {
			LOG("cannabus_conv_frame_to_op: %d\n", err);
		}

		// TODO: handle message here
		err = kErrUnimplemented;

		if(err < kErrSuccess) {
			LOG("handle_operation failed: %d\n", err);
		}
	}
}



/**
 * Sends the given operation on the bus.
 */
int cannabus_send_op(cannabus_operation_t *op) {
	int err;
	cannabus_can_frame_t frame;

	// convert the operation to a CAN frame
	err = cannabus_conv_op_to_frame(op, &frame);

	if(err < kErrSuccess) {
		return err;
	}

	// now, transmit the frame
	err = gCANnabusState.callbacks.can_tx_message(&frame);

	if(err >= kErrSuccess) {
		// increment send counter
		gCANnabusState.txFrames++;
	}

	return err;
}



/**
 * Acknowledges a received operation.
 *
 * An acknowledgment is a frame with no data, with the same address as the
 * last write, but with the second highest bit set in the identifier.
 */
int cannabus_ack_received(cannabus_operation_t *_op) {
	cannabus_operation_t op;
	memset(&op, 0, sizeof(op));

	// copy all relevant fields
	op.reg = _op->reg;
	op.priority = _op->priority;
	op.ack = 1;

	// send it
	return cannabus_send_op(&op);
}



/**
 * Converts a received CAN frame into a CANnabus operation.
 */
int cannabus_conv_frame_to_op(cannabus_can_frame_t *frame, cannabus_operation_t *op) {
	// extract relevant fields from identifier
	uint16_t nodeId = (uint16_t) ((frame->identifier >> 11) & 0xFFFF);
	uint16_t reg = (uint16_t) (frame->identifier & 0x7FF);

	// copy fields from the CAN frame
	op->priority = (frame->identifier & 0x10000000) ? 1 : 0;
	op->ack = (frame->identifier & 0x08000000) ? 1 : 0;
	op->broadcast = (nodeId == 0xFFFF) ? 1 : 0;
	op->data_len = frame->data_len;
	op->reg = reg;
	op->rtr = frame->rtr;

	// set rx field
	op->rx = 1;

	// copy the received node id (but don't set addrValid flag)
	op->addr = nodeId;

	// copy data
	memcpy(&op->data, &frame->data, 8);

	// message was ok
	return kErrSuccess;
}

/**
 * Converts a CANnabus operation into a CAN frame to be transmitted.
 */
int cannabus_conv_op_to_frame(cannabus_operation_t *op, cannabus_can_frame_t *frame) {
	// TODO: make sure the rx field is clear?

	// build the identifier
	uint32_t identifier = (uint32_t) ((op->reg & 0x7FF));

	if(op->broadcast) {
		identifier |= (uint32_t) (0xFFFF << 11);
	} else {
		// it's not a broadcast frame. do we use the specified address?
		if(op->addrValid) {
			identifier |= (uint32_t) (op->addr << 11);
		}
		// no, use the previously device id
		else {
			identifier |= (uint32_t) (gCANnabusState.address << 11);
		}
	}

	// is it an ack frame?
	if(op->ack) {
		identifier |= 0x08000000;
	}

	// is it a priority frame?
	if(op->priority) {
		identifier |= 0x10000000;
	}

	// copy parameters to frame
	frame->identifier = identifier;
	frame->rtr = op->rtr;
	frame->data_len = op->data_len;

	// copy data to frame
	memcpy(&frame->data, &op->data, 8);

	return kErrSuccess;
}
