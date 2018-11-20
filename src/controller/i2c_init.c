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

#include <string.h>

/// I2C interface state
static controller_i2c_state_t gState;

/**
 * I2C driver callbacks
 */
static const i2c_callbacks_t gCallbacks = {

};

/**
 * Register map for the I2C driver.
 */
#define kNumRegs					1
static i2c_register_t gRegs[kNumRegs] = {
	// Reg 0x00: Status register
	{
		.read = {0xDE, 0xAD, 0xBE, 0xEF}
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
	LOG("register 0x%02x read\n", reg);

	// TODO: implement
	return kErrUnimplemented;
}

/**
 * Callback function for a register write.
 */
int controller_i2c_reg_write(uint8_t reg) {
	LOG("register 0x%02x written\n", reg);

	// TODO: implement
	return kErrUnimplemented;
}
