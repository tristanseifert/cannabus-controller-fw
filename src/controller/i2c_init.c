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

uint8_t test[4] = {
	0x01, 0x02, 0x03, 0x84
};

/**
 * Register map for the I2C driver.
 */
#define kNumRegs					1
static i2c_register_t gRegs[kNumRegs] = {
	// Reg 0x00: Status register
	{
		.valid = 1,

		.readSize = 4,
		.regReadBuffer = &test,

		.writeSize = 0,
		.writeCb = NULL
	}
};



/**
 * Initializes the controller's I2C interface.
 */
void controller_i2c_init(void) {
	int err;

	// clear state
	memset(&gState, 0, sizeof(gState));

	// initialize status reg
	gState.status[0] = 0xDE;
	gState.status[1] = 0xAD;
	gState.status[2] = 0xBE;
	gState.status[3] = 0xEF;

	LOG("regs are at 0x%08x\n", &gRegs);
	LOG("status data at 0x%08x (0x%08x)\n", gRegs[0].regReadBuffer, &gState.status);

	// initialize driver
	err = i2c_init((i2c_register_t *) &gRegs, kNumRegs);

	if(err < kErrSuccess) {
		LOG("i2c_init: %d\n", err);
	}
}
