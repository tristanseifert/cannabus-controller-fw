/*
 * i2c_private.h
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */

#ifndef PERIPH_I2C_PRIVATE_H_
#define PERIPH_I2C_PRIVATE_H_

#include "i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * Notification values for the I2C task
 */
typedef enum {
	kNotificationRead			= (1 << 0),
	kNotificationWrite			= (1 << 1),

	kNotificationAny			= (kNotificationRead | kNotificationWrite)
} i2c_task_notification_t;

/**
 * Size for the I2C handler task: this is comparatively large since we will be
 * calling out to the app code, which could be reasonably complex.
 */
#define kI2CStackSize				70

/**
 * Size of the I2C receive buffer.
 */
#define kI2CRxBuffer				16


/**
 * Internal state of the I2C driver.
 */
typedef struct {
	/// has a register been latched?
	bool regLatched;
	/// current i2c register we're dealing with, or 0xFFFF if none
	uint16_t reg;

	/// I2C register to which the last read/write notification pertains
	uint16_t notificationReg;
	/// byte counter for the notification
	size_t notificationByteCounter;

	/// how many bytes of the register have been read?
	size_t readCounter;
	/// how many bytes of the register have been written?
	size_t writeCounter;

	/// how many writes have taken place since start
	size_t totalNumWrites;
	/// how many reads have taken place since start
	size_t totalNumReads;

	/// FreeRTOS task handle
	TaskHandle_t task;
	/// task control block
	StaticTask_t taskTCB;
	/// stack for task
	StackType_t taskStack[kI2CStackSize];


	/// callbacks to notify client after reads/writes to registers
	i2c_callbacks_t cb;

	/// registers the driver accesses
	i2c_register_t *regs;
	uint8_t numRegs;
} i2c_state_t;



/**
 * Initializes the GPIOs used for I2C.
 */
void i2c_init_gpio(void);
/**
 * Initializes the I2C peripheral.
 */
void i2c_init_peripheral(void);
/**
 * Performs final initialization of the I2C peripheral before attaching to the
 * bus: this also finally enables I2C interrupts.
 */
void i2c_init_begin(void);
/**
 * Initialize slave byte control for just the register number.
 */
void i2c_sbc_init_regnum(void);



/**
 * Entry point for the I2C task.
 */
void i2c_task(void *);



/**
 * I2C interrupt handler
 */
void I2C1_IRQHandler(void);
/**
 * Sends a NACK.
 */
static inline void i2c_irq_nack(void);
/**
 * Reloads the NBYTES counter for slave byte control.
 */
static void i2c_irq_sbc_reload(void);
/**
 * When new data needs to be transmitted (an I2C read txn is occurring), this
 * does that.
 */
static void i2c_irq_tx(void);

#endif /* PERIPH_I2C_PRIVATE_H_ */
