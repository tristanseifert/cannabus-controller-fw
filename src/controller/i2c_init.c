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
 * Callbacks for the I2C handler
 */
static const i2c_callbacks_t callbacks = {
	.read = controller_i2c_read,
	.reg_read = controller_i2c_reg_read,
	.reg_write_max = controller_i2c_reg_write_max,
	.reg_write = controller_i2c_reg_write
};



/**
 * Initializes the controller's I2C interface.
 */
void controller_i2c_init(void) {
	int err;

	// clear state
	memset(&gState, 0, sizeof(gState));

	// initialize driver
	err = i2c_init(&callbacks);

	if(err < kErrSuccess) {
		LOG("i2c_init: %d\n", err);
	}
}



/**
 * Handles a read directly from the device: this returns the 4 byte status
 * register.
 */
int controller_i2c_read(void **outBuffer, size_t *outBufferSz) {
	// prepare the 4 byte buffer
	*outBuffer = &gState.readBuffer;
	*outBufferSz = 4;

	// TODO: actually fill this lol
	gState.readBuffer[0] = 0xDE;
	gState.readBuffer[1] = 0xAD;
	gState.readBuffer[2] = 0xBE;
	gState.readBuffer[3] = 0xEF;

	// done!
	return kErrSuccess;
}


/**
 * Responds to a request to read the given register.
 */
int controller_i2c_reg_read(uint8_t reg) {
	int err = kErrUnimplemented;

	// status register?
	if(reg == 0) {
		// get the status register data
		void *buffer;
		size_t bufferSz;
		err = controller_i2c_read(&buffer, &bufferSz);

		// handle errors
		if(err < kErrSuccess) {
			return err;
		}

		// send that data
		err = i2c_write(buffer, bufferSz);
	}

	// return error code. it's kErrUnimplemented by default
	return err;
}

/**
 * Returns the number of bytes that can be written into the specified register.
 */
int controller_i2c_reg_write_max(uint8_t reg, size_t *maxSz) {
	return kErrUnimplemented;
}

/**
 * Handles data that was written to the register.
 */
int controller_i2c_reg_write(uint8_t reg, void *data, size_t dataSz) {
	return kErrUnimplemented;
}
