/*
 * i2c_cannabus.h
 *
 * CANNabus interface
 *
 *  Created on: Nov 21, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_I2C_CANNABUS_H_
#define CONTROLLER_I2C_CANNABUS_H_

#include <stdint.h>

/**
 * Initializes the CANnabus control register.
 */
void reg_init_cannabus_control(void);



/**
 * Configures the CANnabus driver.
 */
int reg_write_cannabus_control(uint8_t reg, uint32_t writtenValue);

/**
 * Sets the CANnabus device ID.
 */
int reg_write_cannabus_device_id(uint8_t reg, uint32_t writtenValue);

/**
 * Sets the CANnabus interrupt configuration.
 */
int reg_write_cannabus_irq_config(uint8_t reg, uint32_t writtenValue);

#endif /* CONTROLLER_I2C_CANNABUS_H_ */
