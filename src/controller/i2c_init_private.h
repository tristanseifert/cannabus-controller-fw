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
	uint8_t status[0];
} controller_i2c_state_t;

#endif /* CONTROLLER_I2C_INIT_PRIVATE_H_ */
