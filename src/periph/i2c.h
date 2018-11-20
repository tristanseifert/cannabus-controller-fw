/*
 * i2c.h
 *
 * Provides a simple interface to the application for implementing a register
 * type interface over I2C. Application code is called to handle writes as well
 * as any reads.
 *
 * Received data beyond the first byte (register number) is passed to the
 * application.
 *
 * For writes from the master, data is passed to the application in a buffer
 * no larger than previously determined by a call to reg_write_max.
 *
 * For reads, the application can provide a buffer of a specific length that
 * is transmitted over the bus. The call will return with the number of bytes
 * read by the master, which is either the whole buffer, or less if it we
 * received a NACK before.
 *
 * Reads without a provided register number are handled by a special function
 * that just returns a fixed size buffer asynchronously.
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */

#ifndef PERIPH_I2C_H_
#define PERIPH_I2C_H_

#include <stdint.h>
#include <stddef.h>

/**
 * I2C driver register state. The driver is given an array of one or more of
 * these and uses them for all I/O to those registers.
 */
typedef struct {
	/// Contents of this register when read
	uint8_t read[4];
	/// Contents of this register that are written to
	uint8_t write[4];
} i2c_register_t;

/**
 * Callbacks used by the I2C driver to inform the client code when a register
 * has been read or written.
 */
typedef struct {
	/// The given register was read.
	int (*read)(uint8_t);
	/// The given register was written to.
	int (*written)(uint8_t);
} i2c_callbacks_t;



/**
 * Initializes the I2C driver.
 */
int i2c_init(const i2c_callbacks_t *callbacks, i2c_register_t *regs, uint8_t numRegs);


#endif /* PERIPH_I2C_H_ */
