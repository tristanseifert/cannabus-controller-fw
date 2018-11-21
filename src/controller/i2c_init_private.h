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
 * Typedef for a register initialization function.
 */
typedef void (*controller_i2c_reg_init_t)(void);

/**
 * Define for a reserved register
 */
#define kReservedRegister				{ .read = {0xFF,0xFF,0xFF,0xFF} }

/**
 * A set of read/write routines for a particular register.
 */
typedef struct {
	int (*read)(uint8_t, uint32_t);
	int (*write)(uint8_t, uint32_t);
} controller_i2c_routines_t;

/**
 * Internal state of the controller's I2C register interface.
 */
typedef struct {
	/// status register
	uint8_t status[4];

	/// controller status
	struct {

	} ctrl_status;

	/// cannabus status
	struct {

	} cannabus_status;
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
 * No-op read handler
 */
int controller_i2c_routine_read_nop(uint8_t reg, uint32_t readValue);
/**
 * No-op write handler
 */
int controller_i2c_routine_write_nop(uint8_t reg, uint32_t writtenValue);



/**
 * Helper method that returns a register's read or write value as an uint32_t.
 */
uint32_t controller_i2c_get_reg(uint8_t reg, bool read);
/**
 * Helper method that sets a register's value from an uint32_t.
 */
void controller_i2c_set_reg(uint8_t reg, bool read, uint32_t newValue);
/**
 * Returns a read/write pointer for the given register.
 */
static uint8_t *_controller_i2c_get_reg_ptr(uint8_t reg, bool read);

#endif /* CONTROLLER_I2C_INIT_PRIVATE_H_ */
