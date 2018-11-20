/*
 * i2c_init.c
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */
#include "i2c_init.h"
#include "i2c_init_private.h"

#include "controller.h"

#include "../periph/i2c.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
 * Register map for the I2C driver.
 */
#define kNumRegs					3
static i2c_register_t gRegs[kNumRegs] = {
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

	LOG("regs are at 0x%08x\n", &gRegs);

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
	LOG("register 0x%02x read (0x%08x)\n", reg,
			controller_i2c_convert_reg(reg, true));

	// TODO: implement
	return kErrUnimplemented;
}

/**
 * Callback function for a register write.
 */
int controller_i2c_reg_write(uint8_t reg) {
	LOG("register 0x%02x written (0x%08x)\n", reg,
			controller_i2c_convert_reg(reg, false));

	// TODO: implement
	return kErrUnimplemented;
}


/**
 * Test method that turns a register's read/write data into a single value.
 */
static uint32_t controller_i2c_convert_reg(uint8_t reg, bool read) {
	uint32_t value = 0;

	// get a pointer to the array
	uint8_t *ptr = NULL;

	if(read) {
		ptr = (uint8_t *) &gRegs[reg].read;
	} else {
		ptr = (uint8_t *) &gRegs[reg].write;
	}

	// copy data
	value |= (uint32_t) ((*ptr++) << 24);
	value |= (uint32_t) ((*ptr++) << 16);
	value |= (uint32_t) ((*ptr++) <<  8);
	value |= (uint32_t) ((*ptr++) <<  0);

	return value;
}
