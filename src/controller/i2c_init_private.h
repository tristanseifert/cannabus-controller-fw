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
#include <stdbool.h>

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



/**
 * Helper method that returns a register's read or write value as a single value
 * to be used for printing.
 */
static uint32_t controller_i2c_convert_reg(uint8_t reg, bool read);

#endif /* CONTROLLER_I2C_INIT_PRIVATE_H_ */
