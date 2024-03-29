/*
 * i2c_init_private.h
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_I2C_INIT_PRIVATE_H_
#define CONTROLLER_I2C_INIT_PRIVATE_H_

#include "i2c_discovery.h"
#include "i2c_cannabus.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"



/// Size of the controller task's stack
#define kControllerStackSize		100
/// Number of messages to hold in the message queue
#define kTaskMsgQueueSize		4

/**
 * Bitset of notifications for the I2C task: these are typically used inside
 * handler functions to wait for an event.
 */
typedef enum {
	/// notification process complete
	kNotificationDiscoveryDone		= (1 << 1)
} controller_i2c_task_notification_t;

/**
 * Type of task message
 */
typedef enum {
	/// begin discovery process
	kMsgTypeStartDiscovery			= 1,
	/// perform an IO operation
	kMsgTypeRegisterIORequest,
	/// an IO operation completed
	kMsgTypeRegisterIOCompleted,
} controller_i2c_task_msg_type_t;
/**
 * Task message: queued from register handlers to perform some sort of longer
 * running operation.
 */
typedef struct controller_i2c_task_msg {
	/// type
	controller_i2c_task_msg_type_t type;

	/// payload
	union {
		/// CANnabus IO request
		struct {
			/// mailbox to perform an IO for
			void *mailbox;

			/// is this a read operation?
			unsigned int read		: 1;
			/// is this a write operation?
			unsigned int write		: 1;
		} ioRequest;

		/// CANnabus IO completion
		struct {
			/// mailbox that had an event
			void *mailbox;
			/// status code from CANnabus stack
			int status;
		} ioComplete;
	};
} controller_i2c_task_msg_t;


/**
 * Macro defining a bit in a register.
 */
#define REG_BIT(x)					(1UL << x)



/**
 * Typedef for a register initialization function.
 */
typedef void (*controller_i2c_reg_init_t)(void);

/**
 * A set of read/write routines for a particular register.
 */
typedef struct {
	int (*read)(uint8_t, uint32_t);
	int (*write)(uint8_t, uint32_t);
} controller_i2c_routines_t;



/**
 * Define for a reserved and empty register and associated handlers
 */
#define kReservedRegister				{ .read = {0xFF,0xFF,0xFF,0xFF} }
#define kReservedRegisterHandlers		{ .read = reg_read_nop, .write = reg_write_nop }

#define kEmptyRegister					{ .read = {0x00,0x00,0x00,0x00} }

#define kDefaultHandlers					{ .read = reg_read_nop, .write = reg_write_nop }



/**
 * Internal state of the controller's I2C register interface.
 */
typedef struct {
	/// status register
	uint8_t status[4];

	/// count of read ok interrupts
	unsigned int rdOkIrqs			: 2;
	/// count of read error interrupts
	unsigned int rdErrIrqs			: 2;
	/// count of read timeout interrupts
	unsigned int rdTimeoutIrqs		: 2;

	/// count of write ok interrupts
	unsigned int wrOkIrqs			: 2;
	/// count of write error interrupts
	unsigned int wrErrIrqs			: 2;
	/// count of write timeout interrupts
	unsigned int wrTimeoutIrqs		: 2;

	/// discovery related status
	controller_i2c_discovery_state_t discovery;

	/// CANnabus glue state
	controller_i2c_cannabus_state_t cannabus;

	/// message queue buffer
	controller_i2c_task_msg_t msgQueueBuffer[kTaskMsgQueueSize];
	/// message queue struct
	StaticQueue_t msgQueueStruct;
	/// message queue handle
	QueueHandle_t msgQueue;

	/// FreeRTOS task handle
	TaskHandle_t task;
	/// task control block
	StaticTask_t taskTCB;
	/// stack for task
	StackType_t taskStack[kControllerStackSize];
} controller_i2c_state_t;



/**
 * I2C controller task entry
 */
void controller_i2c_task(void *ctx);



/**
 * Callback function for a register read.
 */
int controller_i2c_reg_read(uint8_t reg);

/**
 * Callback function for a register write.
 */
int controller_i2c_reg_write(uint8_t reg);



/**
 * No-op read handler
 */
int reg_read_nop(uint8_t reg, uint32_t readValue);



/**
 * No-op write handler
 */
int reg_write_nop(uint8_t reg, uint32_t writtenValue);
/**
 * Copy the written value into the read field of the register.
 */
int reg_write_copy(uint8_t reg, uint32_t writtenValue);



/**
 * Helper method that returns a register's read or write value as an uint32_t.
 */
uint32_t controller_i2c_get_reg(uint8_t reg, bool read);
/**
 * Helper method that sets a register's value from an uint32_t.
 */
void controller_i2c_set_reg(uint8_t reg, bool read, uint32_t newValue);
/**
 * Returns a read/write pointer for the given register.
 */
uint8_t *_controller_i2c_get_reg_ptr(uint8_t reg, bool read);

#endif /* CONTROLLER_I2C_INIT_PRIVATE_H_ */
