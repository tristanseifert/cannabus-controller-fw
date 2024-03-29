/*
 * i2c_irqs.c
 *
 * Code for dealing with interrupt configuration and status registers.
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */
#include "i2c_irqs.h"

#include "i2c_shared.h"

#include "../hw/host_irq.h"

#include <stdint.h>



/**
 * Handles a write to the controller interrupt config register.
 */
int reg_write_ctrl_irq_cfg(uint8_t reg, uint32_t writtenValue) {
	// copy value
	controller_i2c_set_reg(reg, true, writtenValue);

	// begin modifying host IRQ state
	int err = host_irq_begin();

	if(err < kErrSuccess) {
		return err;
	}

	// should we have error interrupts?
	err = host_irq_mask(kIrqLineControllerError,
			(writtenValue & REG_BIT(31)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}

	// should we have init interrupts?
	err = host_irq_mask(kIrqLineControllerInit,
			(writtenValue & REG_BIT(30)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}

	// end IRQ update
	return host_irq_end();
}

/**
 * Handles a write to the controller interrupt status register.
 */
int reg_write_ctrl_irq_status(uint8_t reg __attribute__((unused)),
		uint32_t writtenValue) {
	// begin modifying host IRQ state
	int err = host_irq_begin();

	if(err < kErrSuccess) {
		return err;
	}

	// was the error interrupt acknowledged?
	if(writtenValue & REG_BIT(31)) {
		err = host_irq_set(kIrqLineControllerError, kLineDeasserted);

		if(err < kErrSuccess) {
			return err;
		}
	}

	// was the init interrupt acknowledged?
	if(writtenValue & REG_BIT(30)) {
		err = host_irq_set(kIrqLineControllerInit, kLineDeasserted);

		if(err < kErrSuccess) {
			return err;
		}
	}

	// update interrupt line and registers
	reg_update_irqs();

	// end IRQ update
	return host_irq_end();
}


/**
 * Handles a write to the CANnabus interrupt status register.
 *
 * The only interrupt that can be acknowledged by writing to this register is
 * the discovery done irq, so we have handle it in here instead of the
 * CANnabus specific files.
 */
int reg_write_cannabus_irq_status(uint8_t reg __attribute__((unused)),
		uint32_t writtenValue) {
	// begin modifying host IRQ state
	int err = host_irq_begin();

	// was the discovery complete interrupt acknowledged?
	if(writtenValue & REG_BIT(16)) {
		err = host_irq_set(kIrqLineDiscoveryDone, kLineDeasserted);

		if(err < kErrSuccess) {
			return err;
		}
	}

	// update interrupt line and registers
	reg_update_irqs();


	// end IRQ update
	return host_irq_end();
}



/**
 * Updates the state of IRQ lines related to register reads and writes, as well
 * as updating IRQ status registers.
 *
 * Read/write mailbox IRQs are updated by checking the related counters in
 * the status struct. These are updated whenever a transaction completes.
 */
void reg_update_irqs(void) {
	int err;

	// begin updates
	err = host_irq_begin();

	if(err < kErrSuccess) {
		LOG("host_irq_begin failed: %d\n", err);
	}

	// update the read lines
	err = host_irq_set(kIrqLineRegReadOk,
			(gState.rdOkIrqs) ? kLineAsserted : kLineDeasserted);

	err = host_irq_set(kIrqLineRegReadErr,
			(gState.rdErrIrqs) ? kLineAsserted : kLineDeasserted);

	err = host_irq_set(kIrqLineRegReadTimeout,
			(gState.rdTimeoutIrqs) ? kLineAsserted : kLineDeasserted);

	// update the write lines
	err = host_irq_set(kIrqLineRegWriteOk,
			(gState.wrOkIrqs) ? kLineAsserted : kLineDeasserted);

	err = host_irq_set(kIrqLineRegWriteErr,
			(gState.wrErrIrqs) ? kLineAsserted : kLineDeasserted);

	err = host_irq_set(kIrqLineRegWriteTimeout,
			(gState.wrTimeoutIrqs) ? kLineAsserted : kLineDeasserted);

	// end updates
	err = host_irq_end();

	if(err < kErrSuccess) {
		LOG("host_irq_end failed: %d\n", err);
	}

	// update registers
	reg_update_irq_regs();
}

/**
 * Updates the state of all IRQ status registers by reading the host IRQ
 * controller state.
 */
void reg_update_irq_regs(void) {
	uint32_t irqStatus = 0;
	controller_i2c_cannabus_mailbox_t *box;

	// update the controller IRQ status field
	if(host_irq_get(kIrqLineControllerError) == kLineAsserted) {
		irqStatus |= REG_BIT(31);
	}
	if(host_irq_get(kIrqLineControllerInit) == kLineAsserted) {
		irqStatus |= REG_BIT(30);
	}

	controller_i2c_set_reg(0x03, true, irqStatus);

	// update irq source in device status reg for controller IRQ sources
	if(irqStatus) {
		gRegs[0x00].read[3] = 0x01;
	} else {
		gRegs[0x00].read[3] = 0x00;
	}


	// update the CANnabus IRQ state from IRQ lines
	irqStatus = 0;

	if(host_irq_get(kIrqLineDiscoveryDone) == kLineAsserted) {
		irqStatus |= REG_BIT(16);
	}

	// update the read IRQs from IRQ counters
	if(gState.rdOkIrqs) {
		irqStatus |= REG_BIT(31);
	}
	if(gState.rdErrIrqs) {
		irqStatus |= REG_BIT(30);
	}
	if(gState.rdTimeoutIrqs) {
		irqStatus |= REG_BIT(29);
	}

	// set which read mailboxes generated events
	for(int i = 0; i < kNumReadMailboxes; i++) {
		box = &gState.cannabus.readMailbox[i];

		// are any of the flags set on this mailbox?
		if(box->req_ok | box->req_err | box->req_timeout) {
			irqStatus |= REG_BIT((8 + (i & 0x3)));
		}
	}


	// update the write IRQs from the IRQ counters
	if(gState.wrOkIrqs) {
		irqStatus |= REG_BIT(28);
	}
	if(gState.wrErrIrqs) {
		irqStatus |= REG_BIT(27);
	}
	if(gState.wrTimeoutIrqs) {
		irqStatus |= REG_BIT(28);
	}

	// set which write mailboxes generated events
	for(int i = 0; i < kNumWriteMailboxes; i++) {
		box = &gState.cannabus.writeMailbox[i];

		// are any of the flags set on this mailbox?
		if(box->req_ok | box->req_err | box->req_timeout) {
			irqStatus |= REG_BIT((0 + (i & 0x3)));
		}
	}

	// update the register
	controller_i2c_set_reg(0x08, true, irqStatus);


	// update irq source in device status reg for CANnabus IRQ sources
	if(irqStatus) {
		gRegs[0x00].read[2] = 0x01;
	} else {
		gRegs[0x00].read[2] = 0x00;
	}
}
