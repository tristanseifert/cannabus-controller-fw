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
	/// Is this register valid?
	unsigned int valid				: 1;
	/// Number of bytes that can be read from this register
	unsigned int readSize			: 8;
	/// Number of bytes that can be written to this register
	unsigned int writeSize			: 8;

	/// Pointer to this register's read buffer
	void *regReadBuffer;
	/// Function to call when the register was written to.
	int (*writeCb)(uint8_t, void *, size_t);
} i2c_register_t;



/**
 * Initializes the I2C driver.
 */
int i2c_init(i2c_register_t *regs, uint8_t numRegs);


#endif /* PERIPH_I2C_H_ */
