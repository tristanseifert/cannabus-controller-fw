/*
 * i2c_discovery.c
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */
#include "i2c_discovery.h"
#include "i2c_irqs.h"
#include "i2c_shared.h"

#include "../cannabus/cannabus.h"

#include "../hw/host_irq.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>



/**
 * Task entry point for discovering devices.
 */
int controller_discover(void) {
	// load timer
	gState.discovery.timeout = gState.discovery.timeoutReload;

	LOG("Controller: discovery timeout %u ticks\n", gState.discovery.timeout);

	// TODO: send discovery frame

	// TODO: wait for responses to come in

	// clear "in progress" flag
	gState.discovery.inProgress = 0;

	// update registers and mailbox
	reg_disc_update();
	reg_disc_update_mailbox();

	// at this point, discovery is complete. assert interrupt
	host_irq_set(kIrqLineDiscoveryDone, kLineAsserted);
	reg_update_irq_regs();

	// return status
	return kErrSuccess;
}



/**
 * Updates the device discovery registers.
 */
void reg_disc_update(void) {
	uint32_t reg = 0;

	// update the mailbox status register
	reg = (gState.discovery.devices * 2) & 0xFF;

	if(gState.discovery.dataOverflow) {
		reg |= REG_BIT(8);
	}

	controller_i2c_set_reg(0x0A, true, reg);
}
/**
 * Updates the discovery mailbox.
 */
void reg_disc_update_mailbox(void) {
	int mailboxOffset = gState.discovery.mailboxView;

	for(int i = 0; i < 2; i++) {
		// get device id
		uint16_t id = gState.discovery.deviceIds[mailboxOffset];

		// copy it into the data field
		gRegs[0x0B].read[(i * 2) + 0] = (uint8_t) ((id & 0xFF00) >> 8);
		gRegs[0x0B].read[(i * 2) + 1] = (uint8_t) ((id & 0x00FF) >> 0);

		// increment counter
		mailboxOffset++;
	}
}

/**
 * Resets the mailbox.
 */
void reg_disc_reset_mailbox(void) {
	// reset the state of the mailbox
	gState.discovery.dataOverflow = 0;
	gState.discovery.devices = 0;
	gState.discovery.mailboxView = 0;

	// also, clear the buffer
	memset(&gState.discovery.deviceIds, 0, sizeof(gState.discovery.deviceIds));
}



/**
 * Handles a write to the device discovery control register.
 */
int reg_write_disc_control(uint8_t reg __attribute__((unused)),
		uint32_t writtenValue) {
	BaseType_t ok;

	// copy timeout value (if not all zeros)
	if(writtenValue & 0x00FF0000) {
		gState.discovery.timeoutReload = (writtenValue & 0x00FF0000) >> 16;
	}

	// if the start bit is set, set up and notify task
	if(writtenValue & REG_BIT(31)) {
		// reset state
		gState.discovery.inProgress = 1;

		// reset mailbox
		reg_disc_reset_mailbox();

		// finally, update discovery registers
		reg_disc_update();

		// notify task
		ok = xTaskNotify(gState.task, kNotificationStartDiscovery, eSetBits);

		if(ok != pdTRUE) {
			LOG("xTaskNotify failed: %d\n", ok);
			return kErrNotify;
		}
	}

	// success
	return kErrSuccess;
}
/**
 * Handles a write to the device discovery mailbox status register.
 */
int reg_write_disc_mailbox_status(uint8_t reg __attribute__((unused)),
		uint32_t writtenValue) {
	// is the reset bit set?
	if(writtenValue & REG_BIT(31)) {
		// if so, reset it and update mailbox
		reg_disc_reset_mailbox();
		reg_disc_update();
	}

	// success
	return kErrSuccess;
}

/**
 * Handles a read from the device discovery mailbox.
 */
int reg_read_disc_mailbox(uint8_t reg __attribute__((unused)),
		uint32_t readValue __attribute__((unused))) {
	// increment the mailbox view
	gState.discovery.mailboxView += 2;

	// XXX: no bounds checking, the mailbox will just wrap

	// Output the next two device IDs on the output mailbox
	reg_disc_update_mailbox();

	// success
	return kErrSuccess;
}