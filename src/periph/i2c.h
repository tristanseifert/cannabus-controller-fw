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
 * I2C driver callbacks
 */
typedef struct {
	/// returns a fixed size buffer for a device read
	int (*read)(void **, size_t *);

	/// handles a request to read from a given register
	int (*reg_read)(uint8_t);

	/// gets the maximum number of bytes that may be written to this register
	int (*reg_write_max)(uint8_t, size_t *);
	/// handles a write to the given register.
	int (*reg_write)(uint8_t, void *, size_t);
} i2c_callbacks_t;



/**
 * Initializes the I2C driver.
 */
int i2c_init(const i2c_callbacks_t *callbacks);


#endif /* PERIPH_I2C_H_ */
