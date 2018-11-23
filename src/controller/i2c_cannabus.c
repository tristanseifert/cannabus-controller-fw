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
 * Sets the CANnabus interrupt configuration.
 */
int reg_write_cannabus_irq_config(uint8_t reg, uint32_t writtenValue) {
	// copy value
	controller_i2c_set_reg(reg, true, writtenValue);
	LOG("cannabus IRQ config: %08x\n", writtenValue);

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
