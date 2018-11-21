/*
 * cannabus.c
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */
#include "cannabus.h"
#include "cannabus_private.h"

#include "controller.h"

#include <string.h>

/// CANnabus version implemented by this driver
const uint8_t kCannabusVersion = 0x08;

/// Internal state of the CANnabus driver
static cannabus_state_t gCANnabusState;



/**
 * Initializes the CANnabus and sets this node's address.
 */
int cannabus_init(cannabus_addr_t addr, uint8_t deviceType, uint8_t fwUpgradeCapabilities, const cannabus_callbacks_t *callbacks) {
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

	gCANnabusState.deviceType = deviceType;
	gCANnabusState.fwUpgradeCapabilities = fwUpgradeCapabilities;

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

		// handle the message internally if possible
		if(cannabus_is_op_internal(&op)) {
			err = cannabus_internal_op(&op);
		}
		// otherwise, call into application code
		else {
			err = gCANnabusState.callbacks.handle_operation(&op);
		}

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
	uint16_t nodeId = (uint16_t) (frame->identifier >> 11) & 0xFFFF;
	uint16_t reg = (uint16_t) (frame->identifier & 0x7FF);
//	uint8_t priority = (frame->identifier >> 27) & 0x03;

	// check this frame is destined for us? insurance against fucked filters
	if(nodeId != gCANnabusState.address && nodeId != 0xFFFF) {
		return kErrCannabusNodeIdMismatch;
	}

	// copy fields from the CAN frame
	op->priority = (frame->identifier & 0x10000000) ? 1 : 0;
	op->ack = (frame->identifier & 0x08000000) ? 1 : 0;
	op->broadcast = (nodeId == 0xFFFF) ? 1 : 0;
	op->data_len = frame->data_len;
	op->reg = reg;
	op->rtr = frame->rtr;

	memcpy(&op->data, &frame->data, 8);

	// message was ok
	return kErrSuccess;
}

/**
 * Converts a CANnabus operation into a CAN frame to be transmitted.
 *
 * TODO: handle priority here lmfao
 */
int cannabus_conv_op_to_frame(cannabus_operation_t *op, cannabus_can_frame_t *frame) {
	// build the identifier
	uint32_t identifier = (uint32_t) ((op->reg & 0x7FF) | (gCANnabusState.address << 11));

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

	memcpy(&frame->data, &op->data, 8);

	return kErrSuccess;
}



/**
 * Is this CANnabus operation internal, e.g. is it something specified in the
 * CANnabus protocol that all nodes interpret, thus something we handle internal
 * to the protocol handler?
 */
bool cannabus_is_op_internal(cannabus_operation_t *op) {
	// CANnabus protocol handles 0x0000 - 0x0003 rn
	if(op->reg == 0x000) {
		return true;
	}

	// message isn't handled internally otherwise
	return false;
}

/**
 * Handles an internal CANnabus operation.
 */
int cannabus_internal_op(cannabus_operation_t *op) {
	switch(op->reg) {
		// device id register
		case 0x000:
			return cannabus_internal_reg_deviceid(op);


		// unknown registers
		default:
			return kErrCannabusUnimplemented;
	}
}

/**
 * Handles reads/writes to the Device ID (0x0000) register.
 */
int cannabus_internal_reg_deviceid(cannabus_operation_t *_op) {
	int err = kErrCannabusUnimplemented;

	// ignore ack frames, we do not do any writes
	if(_op->ack) {
		return kErrSuccess;
	}

	// is this a broadcast frame?
	if(_op->broadcast) {
		// if so, respond with our device id register
		err = cannabus_internal_reg_deviceid_respond(_op);
	} else {
		// if it's an RTR frame, send the device id register
		if(_op->rtr) {
			err = cannabus_internal_reg_deviceid_respond(_op);
		}
		// otherwise, process a write of the register data
		else {
			// make sure that we received at least two bytes
			if(_op->data_len < 2) {
				return kErrCannabusInvalidFrameSize;
			}

			// the only value that can be written is the device id
			cannabus_addr_t deviceId;

			deviceId = (uint16_t) ((_op->data[0] << 8) | (_op->data[1]));

			err = cannabus_set_address(deviceId);
			if(err < kErrSuccess) {
				return err;
			}

			LOG("updated device id: %x", deviceId);

			// acknowledge if address was changed successfully
			err = cannabus_ack_received(_op);
		}
	}

	return err;
}


/**
 * Sends the device ID response.
 */
int cannabus_internal_reg_deviceid_respond(cannabus_operation_t *_op __attribute__((__unused__))) {
	int err;

	// build a CAN frame to transmit
	cannabus_operation_t op;
	memset(&op, 0, sizeof(op));

	op.reg = 0x000;
	op.data_len = 8;

	// device id
	op.data[0] = (uint8_t) ((gCANnabusState.address & 0xFF00) >> 8);
	op.data[1] = (uint8_t) (gCANnabusState.address & 0x00FF);

	// supported CANnabus version and device type
	op.data[2] = kCannabusVersion;
	op.data[3] = gCANnabusState.deviceType;

	// fw upgrade support, passed to the initializer
	op.data[4] = gCANnabusState.fwUpgradeCapabilities;

	// firmware version
	uint16_t version = gCANnabusState.callbacks.get_fw_version();

	op.data[5] = (uint8_t) ((version & 0xFF00) >> 8);
	op.data[6] = (uint8_t) (version & 0x00FF);

	// transmit the frame
	err = cannabus_send_op(&op);

	// done
	return err;
}
