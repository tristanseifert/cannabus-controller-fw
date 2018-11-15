/*
 * i2c_init_private.h
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_I2C_INIT_PRIVATE_H_
#define CONTROLLER_I2C_INIT_PRIVATE_H_

#include <stdint.h>
#include <stddef.h>

/**
 * Internal state of the controller's I2C register interface.
 */
typedef struct {

} controller_i2c_state_t;



/**
 * Handles a read directly from the device: this returns the 4 byte status
 * register.
 */
int controller_i2c_read(void **outBuffer, size_t *outBufferSz);

/**
 * Returns the number of bytes that can be written into the specified register.
 */
int controller_i2c_reg_write_max(uint8_t reg, size_t *maxSz);
/**
 * Handles data that was written to the register.
 */
int controller_i2c_reg_write(uint8_t reg, void *data, size_t dataSz);
/**
 * Responds to a request to read the given register.
 */
int controller_i2c_reg_read(uint8_t reg);

#endif /* CONTROLLER_I2C_INIT_PRIVATE_H_ */
