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

	// set default timeouts (100ms)
	gCANnabusState.readTimeout = 10;
	gCANnabusState.writeTimeout = 10;

	// create io ops mutex
	gCANnabusState.ioopsMutex = xSemaphoreCreateMutexStatic(&gCANnabusState.ioopsMutexStruct);

	if(gCANnabusState.ioopsMutex == NULL) {
		return kErrSemaphoreCreationFailed;
	}

	// create IOOP timers (no locks since scheduler isn't running yet)
	for(int i = 0; i < kCannabusMaxIoRequests; i++) {
		cannabus_ioop_t *ioOp =  &(gCANnabusState.ioops[i]);

		// create timer. its id is NULL and is set later
		ioOp->timer = xTimerCreateStatic(NULL, 50, pdFALSE, NULL,
				cannabus_ioop_timeout_callback, &ioOp->timerStruct);

		if(ioOp->timer == NULL) {
			return kErrTimerCreationFailed;
		}
	}

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
	LOG("CANnabus: new address 0x%04x\n", addr);

	// update the filters on the CAN peripheral
	err = gCANnabusState.callbacks.can_config_filter(2, 0x07FFF800, (uint32_t) (addr << 11));

	return err;
}



/**
 * Sets the timeout for register read operations.
 *
 * @note Timeouts are specified in ticks, which are 10ms.
 */
int cannabus_set_read_timeout(uint32_t timeout) {
	// timeout may not be 0 and may not be above 255 (2.5 seconds)
	if(timeout == 0 || timeout > 255) {
		return kErrInvalidArgs;
	}

	// otherwise, set the timeout
	gCANnabusState.readTimeout = timeout;

	return kErrSuccess;
}
/**
 * Returns the read timeout.
 */
uint32_t cannabus_get_read_timeout(void) {
	return gCANnabusState.readTimeout;
}

/**
 * Sets the timeout for register write operations.
 *
 * @note Timeouts are specified in ticks, which are 10ms.
 */
int cannabus_set_write_timeout(uint32_t timeout) {
	// timeout may not be 0 and may not be above 255 (2.5 seconds)
	if(timeout == 0 || timeout > 255) {
		return kErrInvalidArgs;
	}

	// otherwise, set the timeout
	gCANnabusState.writeTimeout = timeout;

	return kErrSuccess;
}
/**
 * Returns the write timeout.
 */
uint32_t cannabus_get_write_timeout(void) {
	return gCANnabusState.writeTimeout;
}



/**
 * Performs a read from the given device's register.
 *
 * If the device address is the broadcast address (0xFFFF), the callback is
 * called for every frame that's received up to the timeout.
 */
int cannabus_reg_read(cannabus_addr_t device, uint16_t reg,
		cannabus_io_callback_t callback, uint32_t context, uint32_t timeout) {
	int err;
	BaseType_t ok;

	// use default timeout if needed
	if(timeout == 0) {
		timeout = gCANnabusState.readTimeout;
	}

	// find a free IOOP struct
	cannabus_ioop_t *ioOp = NULL;

	err = cannabus_find_free_ioop(&ioOp);

	if(err < kErrSuccess) {
		LOG("cannabus_find_free_ioop: %d\n", err);
		return err;
	}

	// set up IOOP
	err = cannabus_ioop_setup(ioOp, device, reg, callback, context, true);

	if(err < kErrSuccess) {
		LOG("cannabus_ioop_setup: %d\n", err);
		return err;
	}

	// send read request on the wire
	cannabus_operation_t op;
	memset(&op, 0, sizeof(op));

	op.reg = (reg & 0x7FFUL);
	op.addr = device;

	op.broadcast = (device == 0xFFFF) ? 1 : 0;

	op.rtr = 1;

	err = cannabus_send_op(&op);

	if(err < kErrSuccess) {
		LOG("cannabus_send_op: %d\n", err);
		return err;
	}

	// activate timer (set its period)
	ok = xTimerChangePeriod(ioOp->timer, timeout, portMAX_DELAY);
	return (ok == pdTRUE) ? kErrSuccess : kErrTimer;
}

/**
 * Performs a write to the given device's register.
 *
 * Writing to the broadcast address is not something that's implemented, even
 * though it's technically possible.
 */
int cannabus_reg_write(cannabus_addr_t device, uint16_t reg, void *data,
		size_t dataLen, cannabus_io_callback_t callback, uint32_t context,
		uint32_t timeout) {
	int err;
	BaseType_t ok;

	// validate arguments
	if(data == NULL || dataLen > 8) {
		return kErrInvalidArgs;
	}

	// use default timeout if needed
	if(timeout == 0) {
		timeout = gCANnabusState.writeTimeout;
	}

	// find a free IOOP struct
	cannabus_ioop_t *ioOp = NULL;

	err = cannabus_find_free_ioop(&ioOp);

	if(err < kErrSuccess) {
		LOG("cannabus_find_free_ioop: %d\n", err);
		return err;
	}

	// set up IOOP
	err = cannabus_ioop_setup(ioOp, device, reg, callback, context, false);

	if(err < kErrSuccess) {
		LOG("cannabus_ioop_setup: %d\n", err);
		return err;
	}

	// send read request on the wire
	cannabus_operation_t op;
	memset(&op, 0, sizeof(op));

	op.reg = (reg & 0x7FFUL);
	op.addr = device;

	op.broadcast = (device == 0xFFFF) ? 1 : 0;

	op.rtr = 0;

	op.data_len = dataLen & 0x0F;
	memcpy(&op.data, data, dataLen);

	err = cannabus_send_op(&op);

	if(err < kErrSuccess) {
		LOG("cannabus_send_op: %d\n", err);
		return err;
	}

	// activate timer (set its period)
	ok = xTimerChangePeriod(ioOp->timer, timeout, portMAX_DELAY);
	return (ok == pdTRUE) ? kErrSuccess : kErrTimer;
}



/**
 * Sets up an IOOP for a read/write of the given register on the given device.
 */
int cannabus_ioop_setup(cannabus_ioop_t *ioOp, cannabus_addr_t device, uint16_t reg,
		cannabus_io_callback_t callback, uint32_t context, bool read) {
	BaseType_t ok;

	// acquire lock on the struct to mark it as valid
	ok = xSemaphoreTake(gCANnabusState.ioopsMutex, portMAX_DELAY);

	if(ok != pdTRUE) {
		return kErrMutexTake;
	}

	ioOp->valid = 1;

	xSemaphoreGive(gCANnabusState.ioopsMutex);

	// set read/write flags
	ioOp->read = (read == true) ? 1 : 0;
	ioOp->write = (read == true) ? 0 : 1;

	// copy state into the IO op
	ioOp->broadcast = (device == 0xFFFF) ? 1 : 0;
	ioOp->addr = device;

	ioOp->reg = (reg & 0x7FFUL);

	ioOp->callback = callback;
	ioOp->cbContext = context;

	// set up the timer
	vTimerSetTimerID(ioOp->timer, ioOp);

	// success
	return kErrSuccess;
}

/**
 * Finds a free IO op struct.
 */
int cannabus_find_free_ioop(cannabus_ioop_t **outIoOp) {
	BaseType_t ok;

	// try to take lock
	ok = xSemaphoreTake(gCANnabusState.ioopsMutex, portMAX_DELAY);

	if(ok != pdTRUE) {
		return kErrMutexTake;
	}

	// check each ioop struct
	for(int i = 0; i < kCannabusMaxIoRequests; i++) {
		cannabus_ioop_t *ioOp =  &(gCANnabusState.ioops[i]);

		// is this struct free?
		if(ioOp->valid == 0) {
			// if so, store the address in the provided argument; release mutex
			*outIoOp = ioOp;

			xSemaphoreGive(gCANnabusState.ioopsMutex);
			return kErrSuccess;
		}
	}

	// couldn't find a free IO op. release mutex and return
	xSemaphoreGive(gCANnabusState.ioopsMutex);

	return kErrCannabusNoFreeIOOP;
}

/**
 * Finds an IO op struct that matches the received frame.
 */
int cannabus_find_matching_ioop(cannabus_operation_t *op, cannabus_ioop_t **outIoOp) {
	BaseType_t ok;

	// try to take lock
	ok = xSemaphoreTake(gCANnabusState.ioopsMutex, portMAX_DELAY);

	if(ok != pdTRUE) {
		return kErrMutexTake;
	}

	// check each ioop struct
	for(int i = 0; i < kCannabusMaxIoRequests; i++) {
		// is this struct valid?
		if(gCANnabusState.ioops[i].valid) {
			cannabus_ioop_t *ioOp =  &(gCANnabusState.ioops[i]);

			// does the register match?
			if(ioOp->reg == op->reg) {
				// if it's a broadcast frame, this is all we check
				if(ioOp->broadcast) {
					// store op in provided argument and release mutex
					*outIoOp = ioOp;

					xSemaphoreGive(gCANnabusState.ioopsMutex);
					return kErrSuccess;
				}
				// if it's not a broadcast op, check address
				else {
					if(ioOp->addr == op->addr) {
						// store op in provided argument and release mutex
						*outIoOp = ioOp;

						xSemaphoreGive(gCANnabusState.ioopsMutex);
						return kErrSuccess;
					}
					// the address didn't match
					else {
						continue;
					}
				}
			}
			// register did not match
			else {
				continue;
			}
		}
	}

	// no operation matched. release mutex and return
	xSemaphoreGive(gCANnabusState.ioopsMutex);

	return kErrCannabusNoMatchingIOOP;
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

		// handle the IO operation
		err = cannabus_task_rx(&op);

		if(err < kErrSuccess) {
			LOG("handle_operation failed: %d\n", err);
		}
	}
}

/**
 * Calls any necessary IO callbacks for the received frame.
 *
 * There is a possible race condition if the task gets preempted before the
 * timer is cancalled, but the timer fires on the next tick. We don't handle
 * this in any special way, since we can't wrap xTimerStop() in a critical
 * section; if this happens, the timeout values should be higher.
 */
int cannabus_task_rx(cannabus_operation_t *op) {
	int err;
	BaseType_t ok;

	// find an IO op that matches
	cannabus_ioop_t *ioOp = NULL;

	err = cannabus_find_matching_ioop(op, &ioOp);

	if(err < kErrSuccess) {
		LOG("cannabus_find_matching_ioop: %d\n", err);
		return err;
	}

	if(ioOp == NULL) {
		return kErrCannabusUnexpectedFrame;
	}

	// make sure the ACK bit is set
	if(op->ack) {
		// if not a broadcast frame, cancel timer
		if(ioOp->broadcast == 0) {
			ok = xTimerStop(ioOp->timer, portMAX_DELAY);

			if(ok != pdTRUE) {
				LOG("xTimerStop: %d\n", ok);
			}
		}

		// execute the callback
		err = ioOp->callback(kErrSuccess, ioOp->cbContext, op);

		if(err < kErrSuccess) {
			LOG("ioOp->callback(): %d\n", err);

			// ignore the error though
	//		return err;
		}

		// mark the io op as invalid (free)
		ok = xSemaphoreTake(gCANnabusState.ioopsMutex, portMAX_DELAY);

		if(ok != pdTRUE) {
			return kErrMutexTake;
		}

		ioOp->valid = 0;

		xSemaphoreGive(gCANnabusState.ioopsMutex);
	} else {
		// TODO: this might be too aggressive in declaring failure
		LOG("operation 0x%08x matches IOOP 0x%08x (device 0x%04x, reg 0x%03x) but has ACK=0\n",
				op, ioOp, ioOp->addr, ioOp->reg);

		return kErrCannabusUnexpectedFrame;
	}

	// nothing failed so we're good
	return kErrSuccess;
}

/**
 * IO op timeout callback
 */
void cannabus_ioop_timeout_callback(TimerHandle_t timer) {
	int err;
	BaseType_t ok;

	// get timer ID (IO state struct)
	cannabus_ioop_t *ioOp = (cannabus_ioop_t *) pvTimerGetTimerID(timer);

	if(ioOp == NULL) {
		LOG("CANnabus timeout fired with NULL timer id (timer 0x%08x)\n", timer);
		return;
	}

	// make sure this op is still valid
	if(ioOp->valid == 0) {
		LOG("CANnabus timeout fired with invalid IOOp (IOOp 0x%08x)\n", ioOp);
		return;
	}

	// run callback
	err = ioOp->callback(kErrCannabusTimeout, ioOp->cbContext, NULL);

	if(err < kErrSuccess) {
		LOG("ioOp->callback(): %d\n", err);
	}

	// mark the io op as invalid (free)
	ok = xSemaphoreTake(gCANnabusState.ioopsMutex, portMAX_DELAY);

	if(ok != pdTRUE) {
		LOG("xSemaphoreTake: %d\n", ok);
		// XXX: we should probably return
	}

	ioOp->valid = 0;

	xSemaphoreGive(gCANnabusState.ioopsMutex);
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
