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
	/// status register
	uint8_t status[4];
} controller_i2c_state_t;



/**
 * Callback function for a register read.
 */
int controller_i2c_reg_read(uint8_t reg);

/**
 * Callback function for a register write.
 */
int controller_i2c_reg_write(uint8_t reg);

#endif /* CONTROLLER_I2C_INIT_PRIVATE_H_ */
