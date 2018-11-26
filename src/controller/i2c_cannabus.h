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
 * Initializes the CANnabus control register.
 */
void reg_init_cannabus_control(void);



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
 * Handles writes to a mailbox status register.
 */
int reg_write_cannabus_mailbox_status(uint8_t reg, uint32_t writtenValue);



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
 */
void reg_mailbox_status_update(uint8_t reg, controller_i2c_cannabus_mailbox_t *box);

#endif /* CONTROLLER_I2C_CANNABUS_H_ */
