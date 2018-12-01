/*
 * i2c_cannabus.h
 *
 * CANNabus interface
 *
 *  Created on: Nov 21, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_I2C_CANNABUS_H_
#define CONTROLLER_I2C_CANNABUS_H_

#include "../cannabus/cannabus.h"

#include <stdint.h>

/// number of register read mailboxes
#define kNumReadMailboxes			3
/// number of register write mailboxes
#define kNumWriteMailboxes			3

/// forward declare task msg type
#ifndef controller_i2c_task_msg_t
	typedef struct controller_i2c_task_msg controller_i2c_task_msg_t;
#endif


/**
 * I2C CANnabus mailbox: these are used for both reads in writes, with the
 * meaning of the data field varying (for reads, it will contain the data read
 * from the device when the transaction completes; for writes, it is the data to
 * be written to the register.)
 */
typedef struct {
	/// is this mailbox in use? (set after the GO flag was set; cleared on completion)
	unsigned int used				: 1;
	/// should the request use priority?
	unsigned int priority			: 1;

	/// did the request complete successfully?
	unsigned int req_ok				: 1;
	/// did the request time out?
	unsigned int req_timeout		: 1;
	/// was there an error processing the request?
	unsigned int req_err			: 1;

	/// device being addressed
	cannabus_addr_t device;
	/// register to select
	uint16_t reg;

	/// number of bytes that will be written/were read
	uint8_t expectedNumBytes;

	/// was there a data overflow?
	unsigned int dataOverflow		: 1;
	/// data that will be written/was read
	uint8_t data[8];
	/// number of bytes in the buffer that are valid
	uint8_t dataLen;

	/// read/write cursor into the data
	uint8_t cursor;

	/// I2C register that was written to to begin this operation
	uint8_t i2cReg;
} controller_i2c_cannabus_mailbox_t;

/**
 * I2C CANnabus state
 */
typedef struct {
	/// mailboxes for register read operations
	controller_i2c_cannabus_mailbox_t readMailbox[kNumReadMailboxes];

	/// mailboxes for register write operations
	controller_i2c_cannabus_mailbox_t writeMailbox[kNumWriteMailboxes];
} controller_i2c_cannabus_state_t;



/**
 * Handles an IO request. This message is queued when the "GO" bit on a mailbox
 * is set, and will perform the heavy lifting of putting the transaction on the
 * bus in the controller task rather than the I2C callback.
 */
int controller_cannabus_handle_request(controller_i2c_task_msg_t *msg);

/**
 * Handles a completed read/write transaction. This updates interrupt state and
 * status registers.
 */
int controller_cannabus_handle_completion(controller_i2c_task_msg_t *msg);



/**
 * Initializes the CANnabus control register.
 */
void reg_init_cannabus_control(void);



/**
 * Updates the CANnabus controller status register.
 */
void controller_cannabus_update_status(void);



/**
 * Configures the CANnabus driver.
 */
int reg_write_cannabus_control(uint8_t reg, uint32_t writtenValue);

/**
 * Sets the CANnabus device ID.
 */
int reg_write_cannabus_device_id(uint8_t reg, uint32_t writtenValue);

/**
 * Sets the CANnabus interrupt configuration.
 */
int reg_write_cannabus_irq_config(uint8_t reg, uint32_t writtenValue);



/**
 * Handles writes to a read/write control register.
 */
int reg_write_cannabus_io_control(uint8_t reg, uint32_t writtenValue);

/**
 * Handles writes to a mailbox status register.
 */
int reg_write_cannabus_mailbox_status(uint8_t reg, uint32_t writtenValue);

/**
 * Handles a write to a mailbox data register.
 *
 * This is only valid for register write mailboxes: for read mailboxes, data
 * will be overwritten but nothing will happen.
 */
int reg_write_cannabus_mailbox_data(uint8_t reg, uint32_t writtenValue);

/**
 * Handles a read from a mailbox data register.
 *
 * This is really only useful for register read mailboxes: for write mailboxes,
 * it will just read out the data previously written to the mailbox.
 */
int reg_read_cannabus_mailbox_data(uint8_t reg, uint32_t readValue);



/**
 * Updates CANnabus interrupt counters.
 */
void controller_cannabus_irq_update(void);



/**
 * Finds the mailbox associated with a particular register.
 *
 * Each mailbox is allocated 4 registers. Read registers are at 0x1n, write regs
 * at 0x2n.
 */
controller_i2c_cannabus_mailbox_t *controller_cannabus_get_mailbox(uint8_t reg);

/**
 * Resets a mailbox.
 */
void controller_cannabus_mailbox_reset(controller_i2c_cannabus_mailbox_t *box);

/**
 * Updates a mailbox status register.
 *
 * @param reg Register number of any register for a particular mailbox. The
 * status register is automatically calcualted from it.
 */
void reg_mailbox_status_update(uint8_t reg, controller_i2c_cannabus_mailbox_t *box);



/**
 * IO callback for register reads/writes.
 *
 * Context is the address of the mailbox.
 */
int controller_cannabus_io_callback(int err, uint32_t context, cannabus_operation_t *op);

#endif /* CONTROLLER_I2C_CANNABUS_H_ */
