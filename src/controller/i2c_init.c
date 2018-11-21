/*
 * i2c_init.c
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */
#include "i2c_init.h"
#include "i2c_init_private.h"
#include "i2c_init_regs.h"

#include "controller.h"

#include "../periph/i2c.h"

#include "../cannabus/cannabus.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/// Number of registers on the I2C bus
#define kNumRegs					3
/// Number of initialization routines
#define kNumInitRoutines		2

/// I2C interface state
static controller_i2c_state_t gState;



/**
 * I2C driver callbacks
 */
static const i2c_callbacks_t gCallbacks = {
	.read = controller_i2c_reg_read,
	.written = controller_i2c_reg_write
};



/**
 * Initializers to run before passing the registers to the driver.
 */
static const controller_i2c_reg_init_t gInitRoutines[] = {
	reg_init_version,
	NULL
};


/**
 * Read/write handling routines for each register.
 *
 * Read routines are called after the register has been read.
 * Write routines are called after the register was written to.
 */
static const controller_i2c_routines_t gRoutines[kNumRegs] = {
	// Reg 0x00: Status register
	{
		.read = controller_i2c_routine_read_nop,
		.write = controller_i2c_routine_write_nop
	},
	// Reg 0x01: Version
	{
		.read = controller_i2c_routine_read_nop,
		.write = controller_i2c_routine_write_nop
	},
	// Reg 0x02: Controller interrupt config
	{
		.read = controller_i2c_routine_read_nop,
		.write = controller_i2c_routine_write_nop
	}
};

/**
 * Register map for the I2C driver.
 *
 * 0x00: Controller status
 * 0x01: Controller version
 * 0x02: Controller IRQ config
 * 0x03: Controller IRQ status
 * 0x04: CANnabus status
 * 0x05: CANnabus control
 * 0x06: CANnabus device ID
 * 0x07: CANnabus IRQ config
 * 0x08: CANnabus IRQ status
 * 0x09: Device discovery control
 * 0x0A: Device discovery mailbox status
 * 0x0B: Device discovery mailbox
 * 0x0C: Reserved
 * 0x0D: Reserved
 * 0x0E: Reserved
 * 0x0F: Reserved
 *
 * 0x10: Register read 0 control
 * 0x11: Register read 0 mailbox status
 * 0x12: Register read 0 mailbox
 * 0x13: Register read 0, reserved
 *
 * 0x20: Register write 0 control
 * 0x21: Register write 0 mailbox status
 * 0x22: Register write 0 mailbox
 * 0x23: Register write 0, reserved
 */
i2c_register_t gRegs[] = {
	// Reg 0x00: Status register
	{
		.read = {0xDE, 0xAD, 0xBE, 0xEF}
	},
	// Reg 0x01: Version
	{
		.read = {0x00, 0x00, 0x00, 0x00}
	},
	// Reg 0x02: Controller interrupt config
	{
		.read = {0x00, 0x00, 0x00, 0x00}
	}
};



/**
 * Initializes the controller's I2C interface.
 */
void controller_i2c_init(void) {
	int err;

	// clear state
	memset(&gState, 0, sizeof(gState));

	// run init routines
	controller_i2c_reg_init_t *init;
	init = (controller_i2c_reg_init_t *) &gInitRoutines;

	while(*init != NULL) {
		(*init)();
		init++;
	}

	// initialize driver
	err = i2c_init(&gCallbacks, (i2c_register_t *) &gRegs, kNumRegs);

	if(err < kErrSuccess) {
		LOG("i2c_init: %d\n", err);
	}
}



/**
 * Callback function for a register read.
 */
int controller_i2c_reg_read(uint8_t reg) {
	// get the value of the register as it was read
	uint32_t newValue = controller_i2c_get_reg(reg, true);
	LOG("register 0x%02x read (0x%08x)\n", reg, newValue);

	// call the register specific routine
	return gRoutines[reg].read(reg, newValue);
}

/**
 * Callback function for a register write.
 */
int controller_i2c_reg_write(uint8_t reg) {
	// get the value just written to the register
	uint32_t newValue = controller_i2c_get_reg(reg, false);
	LOG("register 0x%02x written (0x%08x)\n", reg, newValue);

	// call the register specific routine
	return gRoutines[reg].write(reg, newValue);
}



/**
 * No-op read handler
 */
int controller_i2c_routine_read_nop(uint8_t reg __attribute__((unused)),
		uint32_t readValue __attribute__((unused))) {
	return kErrSuccess;
}
/**
 * No-op write handler
 */
int controller_i2c_routine_write_nop(uint8_t reg __attribute__((unused)),
		uint32_t writtenValue __attribute__((unused))) {
	return kErrSuccess;
}



/**
 * Helper method that returns a register's read or write value as an uint32_t.
 */
uint32_t controller_i2c_get_reg(uint8_t reg, bool read) {
	uint32_t value = 0;

	// get a pointer to the array
	uint8_t *ptr = _controller_i2c_get_reg_ptr(reg, read);

	// copy data
	value |= (uint32_t) ((*ptr++) << 24);
	value |= (uint32_t) ((*ptr++) << 16);
	value |= (uint32_t) ((*ptr++) <<  8);
	value |= (uint32_t) ((*ptr++) <<  0);

	return value;
}

/**
 * Helper method that sets a register's value from an uint32_t.
 */
void controller_i2c_set_reg(uint8_t reg, bool read, uint32_t newValue) {
	// get a pointer to the array
	uint8_t *ptr = _controller_i2c_get_reg_ptr(reg, read);

	// copy data
	*ptr++ = (uint8_t) ((newValue & 0xFF000000) >> 24);
	*ptr++ = (uint8_t) ((newValue & 0x00FF0000) >> 16);
	*ptr++ = (uint8_t) ((newValue & 0x0000FF00) >>  8);
	*ptr++ = (uint8_t) ((newValue & 0x000000FF) >>  0);
}

/**
 * Returns a read/write pointer for the given register.
 */
static uint8_t *_controller_i2c_get_reg_ptr(uint8_t reg, bool read) {
	if(read) {
		return (uint8_t *) &gRegs[reg].read;
	} else {
		return (uint8_t *) &gRegs[reg].write;
	}
}
