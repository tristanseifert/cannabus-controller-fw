/*
 * i2c_cannabus.c
 *
 *  Created on: Nov 21, 2018
 *      Author: tristan
 */
#include "i2c_cannabus.h"

#include "i2c_shared.h"
#include "i2c_irqs.h"

#include "../hw/host_irq.h"
#include "../cannabus/cannabus.h"

#include <stdint.h>
#include <string.h>

/**
 * Initializes the CANnabus control register.
 *
 * This sets the initial timeout value to 500ms.
 */
void reg_init_cannabus_control(void) {
	// get timeouts from CANnabus controller
	uint32_t writeTimeout = cannabus_get_write_timeout();
	uint32_t readTimeout = cannabus_get_read_timeout();

	// register contains the timeouts only
	uint32_t reg = ((writeTimeout & 0xFF) << 16) | ((readTimeout & 0xFF) << 8);
	controller_i2c_set_reg(0x05, true, reg);

	// update the timeouts
	reg_write_cannabus_control(0x05, reg);
}



/**
 * Configures the CANnabus driver.
 */
int reg_write_cannabus_control(uint8_t reg, uint32_t writtenValue) {
	int err;

	// ignore reserved portions
	writtenValue &= 0x81FFFF00;
	controller_i2c_set_reg(reg, true, writtenValue);

	// set write timeout
	uint32_t timeout = (writtenValue & 0x00FF0000) >> 16;

	err = cannabus_set_write_timeout(timeout);

	if(err < kErrSuccess) {
		LOG("cannabus_set_write_timeout: %d\n", err);
		return err;
	}

	// set read timeout
	timeout = (writtenValue & 0x0000FF00) >> 8;

	err = cannabus_set_read_timeout(timeout);

	if(err < kErrSuccess) {
		LOG("cannabus_set_read_timeout: %d\n", err);
		return err;
	}

	// TODO: handle enable/reset

	// if we get down here, no errors occurred
	return kErrSuccess;
}

/**
 * Sets the CANnabus device ID.
 *
 * A 16-bit device ID is in the top 16 bits of the write.
 */
int reg_write_cannabus_device_id(uint8_t reg, uint32_t writtenValue) {
	// copy value
	controller_i2c_set_reg(reg, true, writtenValue);

	// get address
	uint16_t address = (uint16_t) (writtenValue >> 16);

	// set it
	return cannabus_set_address(address);
}



/**
 * Sets the CANnabus interrupt configuration.
 */
int reg_write_cannabus_irq_config(uint8_t reg, uint32_t writtenValue) {
	// copy value
	controller_i2c_set_reg(reg, true, writtenValue);
	LOG("Controller: CANnabus IRQ config 0x%08x\n", writtenValue);

	// begin modifying host IRQ state
	int err = host_irq_begin();

	if(err < kErrSuccess) {
		return err;
	}

	// should we have discovery done interrupts?
	err = host_irq_mask(kIrqLineDiscoveryDone,
			(writtenValue & REG_BIT(16)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}

	// should we have register read ok interrupts?
	err = host_irq_mask(kIrqLineRegReadOk,
			(writtenValue & REG_BIT(31)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}
	// should we have register read error interrupts?
	err = host_irq_mask(kIrqLineRegReadErr,
			(writtenValue & REG_BIT(30)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}
	// should we have register read timeout interrupts?
	err = host_irq_mask(kIrqLineRegReadTimeout,
			(writtenValue & REG_BIT(29)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}

	// should we have register write ok interrupts?
	err = host_irq_mask(kIrqLineRegWriteOk,
			(writtenValue & REG_BIT(28)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}
	// should we have register write error interrupts?
	err = host_irq_mask(kIrqLineRegWriteErr,
			(writtenValue & REG_BIT(27)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}
	// should we have register write timeout interrupts?
	err = host_irq_mask(kIrqLineRegWriteTimeout,
			(writtenValue & REG_BIT(26)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}

	// end IRQ update
	return host_irq_end();
}



/**
 * Handles writes to a mailbox status register.
 */
int reg_write_cannabus_mailbox_status(uint8_t reg, uint32_t writtenValue) {
	controller_i2c_cannabus_mailbox_t *box;

	// get mailbox
	box = controller_cannabus_get_mailbox(reg);

	// ensure mailbox is not NULL
	if(box == NULL) {
		LOG("Controller: no read/write mailbox for reg 0x%02x\n", reg);

		return kErrInvalidArgs;
	}

	// is the RESET bit set?
	if(writtenValue & REG_BIT(31)) {
		controller_cannabus_mailbox_reset(box);
	}

	// is the OK bit set? (clear OK interrupt)
	if(writtenValue & REG_BIT(23)) {
		box->req_ok = 0;
	}
	// is the ERR bit set? (clear ERR interrupt)
	if(writtenValue & REG_BIT(22)) {
		box->req_err = 0;
	}
	// is the TIMEOUT bit set? (clear TIMEOUT interrupt)
	if(writtenValue & REG_BIT(21)) {
		box->req_timeout = 0;
	}

	// update this mailbox's status register, then CANnabus interrupt state
	reg_mailbox_status_update(reg, box);

	controller_cannabus_irq_update();

	// success
	return kErrSuccess;
}



/**
 * Updates CANnabus interrupt counters.
 */
void controller_cannabus_irq_update(void) {
	controller_i2c_cannabus_mailbox_t *box;

	// clear read counters
	gState.rdOkIrqs = 0;
	gState.rdErrIrqs = 0;
	gState.rdTimeoutIrqs = 0;

	// update the read interrupt counters
	for(int i = 0; i < kNumReadMailboxes; i++) {
		box = &gState.cannabus.readMailbox[i];

		// is the OK flag set?
		if(box->req_ok) {
			gState.rdOkIrqs++;
		}
		// is the error flag set?
		else if(box->req_err) {
			gState.rdErrIrqs++;
		}
		// is the timeout flag set?
		else if(box->req_timeout) {
			gState.rdTimeoutIrqs++;
		}
	}

	// clear write counters
	gState.wrOkIrqs = 0;
	gState.wrErrIrqs = 0;
	gState.wrTimeoutIrqs = 0;

	// update the write interrupt counters
	for(int i = 0; i < kNumWriteMailboxes; i++) {
		box = &gState.cannabus.writeMailbox[i];

		// is the OK flag set?
		if(box->req_ok) {
			gState.wrOkIrqs++;
		}
		// is the error flag set?
		else if(box->req_err) {
			gState.wrErrIrqs++;
		}
		// is the timeout flag set?
		else if(box->req_timeout) {
			gState.wrTimeoutIrqs++;
		}
	}

	// update interrupt registers and host IRQ line
	reg_update_irqs();
}



/**
 * Finds the mailbox associated with a particular register.
 *
 * Each mailbox is allocated 4 registers. Read registers are at 0x1n, write regs
 * at 0x2n.
 */
controller_i2c_cannabus_mailbox_t *controller_cannabus_get_mailbox(uint8_t reg) {
	controller_i2c_cannabus_mailbox_t *box = NULL;

	// is it a read mailbox?
	if((reg & 0xF0) == 0x10) {
		unsigned int offset = ((reg & 0x0F) / 4U);
		box = &(gState.cannabus.readMailbox[offset]);
	}
	// is it a write mailbox?
	else if((reg & 0xF0) == 0x20) {
		unsigned int offset = ((reg & 0x0F) / 4U);
		box = &(gState.cannabus.writeMailbox[offset]);
	}

	LOG("Controller: Mailbox for reg 0x%02x: 0x%08x\n", reg, box);

	return box;
}

/**
 * Resets a mailbox.
 *
 * This just clears the memory belonging to it.
 */
void controller_cannabus_mailbox_reset(controller_i2c_cannabus_mailbox_t *box) {
	memset(box, 0, sizeof(controller_i2c_cannabus_mailbox_t));
}

/**
 * Updates a mailbox status register.
 */
void reg_mailbox_status_update(uint8_t reg, controller_i2c_cannabus_mailbox_t *box) {
	uint32_t status = 0;

	// copy size and overflow flag
	status |= (box->dataLen & 0xFF);

	if(box->dataOverflow) {
		status |= REG_BIT(8);
	}

	// copy the status flags
	if(box->req_ok) {
		status |= REG_BIT(23);
	} else if(box->req_err) {
		status |= REG_BIT(22);
	} else if(box->req_timeout) {
		status |= REG_BIT(21);
	}

	// set register
	controller_i2c_set_reg(reg, true, status);
}
