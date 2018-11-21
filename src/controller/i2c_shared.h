/*
 * i2c_shared.h
 *
 * Include for all I2C controller related headers.
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_I2C_SHARED_H_
#define CONTROLLER_I2C_SHARED_H_

#include "i2c_init.h"
#include "i2c_init_private.h"

#include "../periph/i2c.h"

#include "controller.h"



/// I2C controller state
extern controller_i2c_state_t gState;
/// I2C registers
extern i2c_register_t gRegs[];

#endif /* CONTROLLER_I2C_SHARED_H_ */
