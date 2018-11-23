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
#include "semphr.h"
#include "timers.h"

#include <stdbool.h>
#include <stddef.h>

/// stack size for CANnabus driver
#define kCANnabusTaskStackSize		100

/// maximum number of IO requests that can be in flight at once
#define kCannabusMaxIoRequests		8

/**
 * CANnabus IO request struct.
 *
 * One of these is created for each call to `cannabus_reg_read` and
 * `cannabus_reg_write`. Any received frames are checked to see if they match
 * the device ID and register and if so, the callback is executed. Once the
 * callback returns, the IO struct is freed.
 *
 * Likewise, if no response is received for a particular operation is received,
 * the callback is called with an error code.
 *
 * Note that broadcast IO behaves slightly differently:
 * - All received frames that match the register address are forwarded to the
 * 	 callback, e.g. device ID checking is not done. Additionally, the callback
 * 	 is not removed after the first frame received.
 * - The callback's timeout handler will be called no matter what to indicate
 * 	 that we do not anticipate receiving any more frames for that IO op.
 */
typedef struct {
	/// is this op valid?
	unsigned int valid				: 1;

	/// is this op for a register read?
	unsigned int read				: 1;
	/// is this op for a register write?
	unsigned int write				: 1;

	/// is this a broadcast IO request?
	unsigned int broadcast			: 1;
	/// what device is being addressed?
	unsigned int addr				: 16;

	/// what register does this IO pertain to?
	unsigned int reg				: 11;

	/// callback to execute
	cannabus_io_callback_t callback;
	/// context for callback
	uint32_t cbContext;

	/// timer struct
	StaticTimer_t timerStruct;
	/// timer to handle timeout
	TimerHandle_t timer;
} cannabus_ioop_t;

/**
 * Internal state for the CANnabus driver.
 */
typedef struct {
	/// node address
	cannabus_addr_t address;

	/// number of received frames
	size_t rxFrames;
	/// number of sent frames
	size_t txFrames;

	/// callbacks in user code to handle CAN bus interfacing
	cannabus_callbacks_t callbacks;

	/// FreeRTOS task handle
	TaskHandle_t task;
	/// task control block
	StaticTask_t taskTCB;
	/// stack for task
	StackType_t taskStack[kCANnabusTaskStackSize];

	/// timeout for register reads
	uint32_t readTimeout;
	/// timeout for register writes
	uint32_t writeTimeout;

	/// IO requests
	cannabus_ioop_t ioops[kCannabusMaxIoRequests];

	/// structure for ioops mutex
	StaticSemaphore_t ioopsMutexStruct;
	/**
	 * Lock protecting access to the `ioops` array. This lock should be taken
	 * when iterating the structure, and when modifying the `valid` flag in a
	 * particular op.
	 */
	SemaphoreHandle_t ioopsMutex;
} cannabus_state_t;



/**
 * CANnabus task entry point
 */
void cannabus_task(void *ctx);


/**
 * Calls any necessary IO callbacks for the received frame.
 */
int cannabus_task_rx(cannabus_operation_t *op);

/**
 * IO op timeout callback
 */
void cannabus_ioop_timeout_callback(TimerHandle_t timer);

/**
 * Sets up an IOOP for a read/write of the given register on the given device.
 */
int cannabus_ioop_setup(cannabus_ioop_t *ioOp, cannabus_addr_t device, uint16_t reg,
		cannabus_io_callback_t callback, uint32_t context, bool read);

/**
 * Finds a free IO op struct.
 */
int cannabus_find_free_ioop(cannabus_ioop_t **outIoOp);
/**
 * Finds an IO op struct that matches the received frame.
 */
int cannabus_find_matching_ioop(cannabus_operation_t *op, cannabus_ioop_t **outIoOp);



/**
 * Converts a received CAN frame into a CANnabus operation.
 */
int cannabus_conv_frame_to_op(cannabus_can_frame_t *frame, cannabus_operation_t *op);

/**
 * Converts a CANnabus operation into a CAN frame to be transmitted.
 */
int cannabus_conv_op_to_frame(cannabus_operation_t *op, cannabus_can_frame_t *frame);

#endif /* CANNABUS_CANNABUS_PRIVATE_H_ */
