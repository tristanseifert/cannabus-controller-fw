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
 * Handles an IO request. This message is queued when the "GO" bit on a mailbox
 * is set, and will perform the heavy lifting of putting the transaction on the
 * bus in the controller task rather than the I2C callback.
 */
int controller_cannabus_handle_request(controller_i2c_task_msg_t *msg) {
	int err;
	controller_i2c_cannabus_mailbox_t *box;

	// get mailbox that completed
	box = (controller_i2c_cannabus_mailbox_t *) msg->ioRequest.mailbox;

	LOG("Controller: transaction request (device 0x%04x, reg 0x%03x, read %d write %d)\n",
			box->device, box->reg, msg->ioRequest.read, msg->ioRequest.write);

	// update status register
	controller_cannabus_update_status();

	// is it a read?
	if(msg->ioRequest.read) {
		// if so, perform a read from the register
		err = cannabus_reg_read(box->device, box->reg,
				controller_cannabus_io_callback, (uint32_t) box, 0);

		if(err < kErrSuccess) {
			LOG("Controller: cannabus_reg_read failed %d\n", err);
			return err;
		}
	}
	// is it a write?
	else if(msg->ioRequest.write) {
		// if so, write data to the register
		err = cannabus_reg_write(box->device, box->reg, box->data, box->dataLen,
				controller_cannabus_io_callback, (uint32_t) box, 0);

		if(err < kErrSuccess) {
			LOG("Controller: cannabus_reg_write failed %d\n", err);
			return err;
		}
	}
	// unknown IO request type, abort
	else {
		LOG("Controller: IO request is neither read nor write (0x%08x)\n", msg);
		return kErrInvalidArgs;
	}

	// if we get down here, assume success
	return kErrSuccess;
}

/**
 * Handles a completed read/write transaction. This updates interrupt state and
 * status registers.
 */
int controller_cannabus_handle_completion(controller_i2c_task_msg_t *msg) {
	controller_i2c_cannabus_mailbox_t *box;

	// get mailbox that completed
	box = (controller_i2c_cannabus_mailbox_t *) msg->ioComplete.mailbox;

	LOG("Controller: transaction completed (box 0x%08x, status %d)\n", box,
			msg->ioComplete.status);

	// clear the "in use" bits in the mailbox
	box->used = 0;

	// update this mailbox's status register and interrupts
	reg_mailbox_status_update(box->i2cReg, box);

	// update interrupt status
	controller_cannabus_irq_update();

	// success
	return kErrSuccess;
}

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
 * Updates the CANnabus controller status register.
 */
void controller_cannabus_update_status(void) {
	uint32_t reg = 0;

	// are any mailboxes in use?
	for(int i = 0; i < kNumReadMailboxes; i++) {
		if(gState.cannabus.readMailbox[i].used) {
			reg |= REG_BIT(29);
		}
	}
	for(int i = 0; i < kNumWriteMailboxes; i++) {
		if(gState.cannabus.writeMailbox[i].used) {
			reg |= REG_BIT(29);
		}
	}

	// set reg
	controller_i2c_set_reg(0x04, true, reg);
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
 * Handles writes to a read/write control register.
 */
int reg_write_cannabus_io_control(uint8_t reg, uint32_t writtenValue) {
	BaseType_t ok;
	controller_i2c_cannabus_mailbox_t *box;

	// get mailbox
	box = controller_cannabus_get_mailbox(reg);

	// ensure mailbox is not NULL
	if(box == NULL) {
		LOG("Controller: no read/write mailbox for reg 0x%02x\n", reg);

		return kErrInvalidArgs;
	}

	// copy device and register numbers
	box->device = (cannabus_addr_t) ((writtenValue & 0xFFFF0000) >> 16);
	box->reg = (writtenValue & 0x0000FFE0) >> 5;

	// expected size of transfer
	box->expectedNumBytes = (writtenValue & 0x0000001C) >> 2;

	// set priority bit if needed
	box->priority = (writtenValue & REG_BIT(1)) ? 1U : 0U;

	// write the register number (used later to update it)
	box->i2cReg = reg;

	// is the GO bit set?
	if((writtenValue & REG_BIT(0))) {
		// clear status bits
		box->req_err = 0;
		box->req_ok = 0;
		box->req_timeout = 0;

		// set "in use" bit
		box->used = 1;

		// set up message
		controller_i2c_task_msg_t msg;
		memset(&msg, 0, sizeof(msg));

		msg.type = kMsgTypeRegisterIORequest;
		msg.ioRequest.mailbox = box;

		// is this for a read mailbox?
		if((reg & 0xF0) == 0x10) {
			// request read transaction
			msg.ioRequest.read = 1;
		}
		// is it for a write mailbox?
		else if((reg & 0xF0) == 0x20) {
			// request write transaction
			msg.ioRequest.write = 1;
		}
		// this shouldn't happen
		else {
			LOG("Controller: write to CANnabus control reg at invalid address 0x%02x\n", reg);
			return kErrInvalidArgs;
		}

		// notify task
		ok = xQueueSendToBack(gState.msgQueue, &msg, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG("xQueueSendToBack: %d\n", ok);
			return kErrNotify;
		}
	}

	// update mailbox status as well as the control reg (with written value)
	reg_mailbox_status_update(reg, box);
	controller_i2c_set_reg(reg, true, writtenValue);

	// lastly, update CANnabus status
	controller_cannabus_update_status();

	return kErrSuccess;
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
 * Handles a write to a mailbox data register.
 *
 * This is only valid for register write mailboxes: for read mailboxes, data
 * will be overwritten but nothing will happen.
 */
int reg_write_cannabus_mailbox_data(uint8_t reg, uint32_t writtenValue) {
	controller_i2c_cannabus_mailbox_t *box;

	// get mailbox
	box = controller_cannabus_get_mailbox(reg);

	// ensure mailbox is not NULL
	if(box == NULL) {
		LOG("Controller: no read/write mailbox for reg 0x%02x\n", reg);

		return kErrInvalidArgs;
	}

	// write four bytes of data into the mailbox
	for(unsigned int i = 0; i < 4; i++) {
		unsigned int shift = ((3 - i) * 8);

		// write into the mailbox
		box->data[box->cursor] = (uint8_t) ((writtenValue & (0xFFU << shift)) >> shift);

		// advance cursor and ensure it stays in bounds (wrap if needed)
		box->cursor = (box->cursor + 1U) % sizeof(box->data);

		// increment byte counter
		box->dataLen = (box->dataLen + 1U) % sizeof(box->data);
	}

	// update this mailbox's status register, then CANnabus interrupt state
	reg_mailbox_status_update(reg, box);

	controller_cannabus_irq_update();

	// success!
	return kErrSuccess;
}

/**
 * Handles a read from a mailbox data register.
 *
 * This is really only useful for register read mailboxes: for write mailboxes,
 * it will just read out the data previously written to the mailbox.
 */
int reg_read_cannabus_mailbox_data(uint8_t reg,
		uint32_t readValue __attribute__((unused))) {
	controller_i2c_cannabus_mailbox_t *box;

	// get mailbox
	box = controller_cannabus_get_mailbox(reg);

	// ensure mailbox is not NULL
	if(box == NULL) {
		LOG("Controller: no read/write mailbox for reg 0x%02x\n", reg);

		return kErrInvalidArgs;
	}

	// copy four bytes of data from the data registers
	for(int i = 0; i < 4; i++) {
		gRegs[reg].read[i] = box->data[box->cursor];

		// advance cursor and ensure it stays in bounds (wrap if needed)
		box->cursor = (uint8_t) ((box->cursor + 1U) % box->dataLen);
	}

	// update this mailbox's status register, then CANnabus interrupt state
	reg_mailbox_status_update(reg, box);

	controller_cannabus_irq_update();

	// success!
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
 *
 * @param reg Register number of any register for a particular mailbox. The
 * status register is automatically calcualted from it.
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
	uint8_t statusReg = (uint8_t) ((reg & 0x3C) | 0x01);

	controller_i2c_set_reg(statusReg, true, status);
}



/**
 * IO callback for register reads/writes.
 *
 * Context is the address of the mailbox.
 */
int controller_cannabus_io_callback(int err, uint32_t context, cannabus_operation_t *op) {
	BaseType_t ok;
	controller_i2c_cannabus_mailbox_t *box;

	// get mailbox
	box = (controller_i2c_cannabus_mailbox_t *) context;

	if(box == NULL) {
		LOG_PUTS("Controller: NULL context on CANnabus read callback");
		return kErrInvalidArgs;
	}

	// timeout?
	if(err == kErrCannabusTimeout) {
		LOG("Controller: timeout for mailbox 0x%08x\n", box);

		// set flags
		box->req_timeout = 1;
	}
	// operation completed
	else if(err == kErrSuccess) {
		LOG("Controller: success for mailbox 0x%08x\n", box);

		// set ok flag
		box->req_ok = 1;

		// copy data if it was a read (reg was 0x1n)
		if((box->i2cReg & 0xF0) == 0x10) {
			box->dataLen = op->data_len;
			memcpy(box->data, op->data, sizeof(op->data));
		}
	}
	// undefined error
	else {
		LOG("Controller: other error for mailbox 0x%08x (%d)\n", box, err);

		// set error flag
		box->req_err = 1;
	}

	// notify task
	controller_i2c_task_msg_t msg;
	memset(&msg, 0, sizeof(msg));

	msg.type = kMsgTypeRegisterIOCompleted;
	msg.ioComplete.mailbox = box;
	msg.ioComplete.status = err;

	ok = xQueueSendToBack(gState.msgQueue, &msg, portMAX_DELAY);

	if(ok != pdTRUE) {
		LOG("xQueueSendToBack: %d\n", ok);
		return kErrNotify;
	}

	// success
	return kErrSuccess;
}

