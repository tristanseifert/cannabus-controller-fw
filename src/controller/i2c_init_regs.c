/*
 * i2c_init_regs.c
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */
#include "i2c_init_regs.h"

#include "i2c_shared.h"

#include "../cannabus/cannabus.h"

#include <stdbool.h>



/**
 * Initializes the version register.
 */
void reg_init_version(void) {
	// Initialize version register
	controller_i2c_set_reg(0x01, true, 0x00100100);
	gRegs[0x01].read[3] = kCannabusVersion;
}
