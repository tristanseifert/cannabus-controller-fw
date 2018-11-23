/*
 * i2c_cannabus.c
 *
 *  Created on: Nov 21, 2018
 *      Author: tristan
 */
#include "i2c_cannabus.h"

#include "i2c_shared.h"

#include "../hw/host_irq.h"
#include "../cannabus/cannabus.h"

#include <stdint.h>

/**
 * Initializes the CANnabus control register.
 *
 * This sets the initial timeout value to 500ms.
 */
void reg_init_cannabus_control(void) {
	// get timeouts from CANnabus controller
	uint32_t writeTimeout = cannabus_get_write_timeout();
	uint32_t readTimeout = cannabus_get_read_timeout();

	// register contains the timeouts only
	uint32_t reg = ((writeTimeout & 0xFF) << 16) | ((readTimeout & 0xFF) << 8);
	controller_i2c_set_reg(0x05, true, reg);

	// update the timeouts
	reg_write_cannabus_control(0x05, reg);
}



/**
 * Configures the CANnabus driver.
 */
int reg_write_cannabus_control(uint8_t reg, uint32_t writtenValue) {
	int err;

	// ignore reserved portions
	writtenValue &= 0x81FFFF00;
	controller_i2c_set_reg(reg, true, writtenValue);

	// set write timeout
	uint32_t timeout = (writtenValue & 0x00FF0000) >> 16;

	err = cannabus_set_write_timeout(timeout);

	if(err < kErrSuccess) {
		LOG("cannabus_set_write_timeout: %d\n", err);
		return err;
	}

	// set read timeout
	timeout = (writtenValue & 0x0000FF00) >> 8;

	err = cannabus_set_read_timeout(timeout);

	if(err < kErrSuccess) {
		LOG("cannabus_set_read_timeout: %d\n", err);
		return err;
	}

	// TODO: handle enable/reset

	// if we get down here, no errors occurred
	return kErrSuccess;
}

/**
 * Sets the CANnabus device ID.
 *
 * A 16-bit device ID is in the top 16 bits of the write.
 */
int reg_write_cannabus_device_id(uint8_t reg, uint32_t writtenValue) {
	// copy value
	controller_i2c_set_reg(reg, true, writtenValue);

	// get address
	uint16_t address = (uint16_t) (writtenValue >> 16);

	// set it
	return cannabus_set_address(address);
}



/**
 * Sets the CANnabus interrupt configuration.
 */
int reg_write_cannabus_irq_config(uint8_t reg, uint32_t writtenValue) {
	// copy value
	controller_i2c_set_reg(reg, true, writtenValue);
	LOG("Controller: CANnabus IRQ config 0x%08x\n", writtenValue);

	// begin modifying host IRQ state
	int err = host_irq_begin();

	if(err < kErrSuccess) {
		return err;
	}

	// should we have discovery done interrupts?
	err = host_irq_mask(kIrqLineDiscoveryDone,
			(writtenValue & REG_BIT(16)) ? kLineUnmasked : kLineMasked);

	if(err < kErrSuccess) {
		return err;
	}

	// TODO: handle other interrupts

	// end IRQ update
	return host_irq_end();
}
