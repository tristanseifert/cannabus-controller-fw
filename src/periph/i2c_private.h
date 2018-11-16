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
	kNotificationI2CError			= (1 << 30),
	kNotificationI2CBusError		= (1 << 29),
	kNotificationI2CNACK			= (1 << 27),
	kNotificationI2CSTOP		= (1 << 26),
	kNotificationRegDataRx			= (1 << 23),
	kNotificationRegLatched			= (1 << 22),
	kNotificationSlaveSelect		= (1 << 21),
	kNotificationDMAError		= (1 << 7),
	kNotificationDMAComplete		= (1 << 6),

	kNotificationAny			= (kNotificationI2CError |
			kNotificationI2CBusError | kNotificationI2CNACK |
			kNotificationI2CSTOP | kNotificationRegDataRx |
			kNotificationRegLatched | kNotificationSlaveSelect |
			kNotificationDMAError | kNotificationDMAComplete)
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
	/// has a register been selected?
	bool regSelected;
	/// current i2c register we're dealing with, or 0xFFFF if none
	uint16_t reg;

	/// I2C receive buffer
	uint8_t rxBuffer[kI2CRxBuffer];
	/// how many bytes of data are valid in the receive buffer?
	size_t rxBufferSz;
	/// total number of bytes we can receive into this buffer
	size_t rxBufferMax;

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

	/// task to notify when the DMA completes, if any
	TaskHandle_t dmaCompleteTask;

	/// callbacks used by the driver
	i2c_callbacks_t cb;
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
 * Initializes the DMA channel for I2C transmission.
 */
void i2c_init_dma(void);
/**
 * Performs final initialization of the I2C peripheral before attaching to the
 * bus: this also finally enables I2C interrupts.
 */
void i2c_init_begin(void);



/**
 * Entry point for the I2C task.
 */
void i2c_task(void *);



/**
 * I2C interrupt handler
 */
void I2C1_IRQHandler(void);
/**
 * DMA channels 2 and 3 interrupt handler: we only use channel 2 for I2C1_TX.
 */
void DMA1_Channel2_3_IRQHandler(void);

#endif /* PERIPH_I2C_PRIVATE_H_ */
