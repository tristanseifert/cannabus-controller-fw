/*
 * i2c_init.c
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */
#include "i2c_init.h"
#include "i2c_init_private.h"

#include "i2c_init_regs.h"
#include "i2c_irqs.h"
#include "i2c_cannabus.h"
#include "i2c_discovery.h"

#include "controller.h"

#include "../periph/i2c.h"

#include "../cannabus/cannabus.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/// Number of registers on the I2C bus
#define kNumRegs					44

// Log register accesses
#define LOG_REG_ACCESS			1

/// I2C interface state
controller_i2c_state_t gState;



/**
 * I2C driver callbacks
 */
static const i2c_callbacks_t gCallbacks = {
	.read = controller_i2c_reg_read,
	.written = controller_i2c_reg_write
};



/**
 * Initializers to run before passing the registers to the driver.
 */
static const controller_i2c_reg_init_t gInitRoutines[] = {
	reg_init_version,
	NULL
};


/**
 * Read/write handling routines for each register.
 *
 * Read routines are called after the register has been read.
 * Write routines are called after the register was written to.
 */
static const controller_i2c_routines_t gRoutines[kNumRegs] = {
	// Reg 0x00: Status register
	kDefaultHandlers,
	// Reg 0x01: Version
	kDefaultHandlers,

	// Reg 0x02: Controller interrupt config
	{
		.read = reg_read_nop,
		.write = reg_write_ctrl_irq_cfg
	},
	// Reg 0x03: Controller interrupt status
	{
		.read = reg_read_nop,
		.write = reg_write_ctrl_irq_status,
	},

	// Reg 0x04: CANnabus status
	kDefaultHandlers,

	// Reg 0x05: CANnabus control
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x06: CANnabus device ID
	{
		.read = reg_read_nop,
		.write = reg_write_cannabus_device_id
	},

	// Reg 0x07: CANnabus interrupt config
	{
		.read = reg_read_nop,
		.write = reg_write_cannabus_irq_config
	},
	// Reg 0x08: CANnabus interrupt status
	{
		.read = reg_read_nop,
		.write = reg_write_cannabus_irq_status
	},

	// Reg 0x09: Device discovery control
	{
		.read = reg_read_nop,
		.write = reg_write_disc_control
	},
	// Reg 0x0A: Device discovery mailbox status
	{
		.read = reg_read_nop,
		.write = reg_write_disc_mailbox_status
	},
	// Reg 0x0B: Device discovery mailbox
	{
		.read = reg_read_disc_mailbox,
		.write = reg_write_nop
	},

	// Regs 0x0C - 0x0F: Reserved
	kReservedRegisterHandlers, kReservedRegisterHandlers,
	kReservedRegisterHandlers, kReservedRegisterHandlers,

	// Reg 0x10: Register read 0 control
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x11: Register read 0 mailbox status
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x12: Register read 0 mailbox
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x13: Register read 0 reserved
	kReservedRegisterHandlers,
	// Reg 0x14: Register read 1 control
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x15: Register read 1 mailbox status
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x16: Register read 1 mailbox
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x17: Register read 1 reserved
	kReservedRegisterHandlers,
	// Reg 0x18: Register read 2 control
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x19: Register read 2 mailbox status
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x1A: Register read 2 mailbox
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x1B: Register read 2 reserved
	kReservedRegisterHandlers,

	// Reg 0x1C - 0x1F: Reserved
	kReservedRegisterHandlers, kReservedRegisterHandlers,
	kReservedRegisterHandlers, kReservedRegisterHandlers,

	// Reg 0x20: Register write 0 control
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x21: Register write 0 mailbox status
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x22: Register write 0 mailbox
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x23: Register write 0 reserved
	kReservedRegisterHandlers,
	// Reg 0x24: Register write 1 control
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x25: Register write 1 mailbox status
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x26: Register write 1 mailbox
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x27: Register write 1 reserved
	kReservedRegisterHandlers,
	// Reg 0x28: Register write 2 control
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x29: Register write 2 mailbox status
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x2A: Register write 2 mailbox
	{
		.read = reg_read_nop,
		.write = reg_write_nop
	},
	// Reg 0x2B: Register write 2 reserved
	kReservedRegisterHandlers,
};

/**
 * Register map for the I2C driver.
 *
 * 0x00: Controller status
 * 0x01: Controller version
 * 0x02: Controller IRQ config
 * 0x03: Controller IRQ status
 * 0x04: CANnabus status
 * 0x05: CANnabus control
 * 0x06: CANnabus device ID
 * 0x07: CANnabus IRQ config
 * 0x08: CANnabus IRQ status
 * 0x09: Device discovery control
 * 0x0A: Device discovery mailbox status
 * 0x0B: Device discovery mailbox
 * 0x0C: Reserved
 * 0x0D: Reserved
 * 0x0E: Reserved
 * 0x0F: Reserved
 *
 * 0x10: Register read 0 control
 * 0x11: Register read 0 mailbox status
 * 0x12: Register read 0 mailbox
 * 0x13: Register read 0, reserved
 *
 * 0x20: Register write 0 control
 * 0x21: Register write 0 mailbox status
 * 0x22: Register write 0 mailbox
 * 0x23: Register write 0, reserved
 */
i2c_register_t gRegs[] = {
	// Reg 0x00: Status register
	kEmptyRegister,
	// Reg 0x01: Version
	kEmptyRegister,

	// Reg 0x02: Controller interrupt config
	kEmptyRegister,
	// Reg 0x03: Controller interrupt status
	kEmptyRegister,

	// Reg 0x04: CANnabus status
	kEmptyRegister,

	// Reg 0x05: CANnabus control
	kEmptyRegister,
	// Reg 0x06: CANnabus device ID
	kEmptyRegister,

	// Reg 0x07: CANnabus interrupt config
	kEmptyRegister,
	// Reg 0x08: CANnabus interrupt status
	kEmptyRegister,

	// Reg 0x09: Device discovery control
	kEmptyRegister,
	// Reg 0x0A: Device discovery mailbox status
	kEmptyRegister,
	// Reg 0x0B: Device discovery mailbox
	kEmptyRegister,

	// Reg 0x0C - 0x0F: Reserved
	kReservedRegister, kReservedRegister, kReservedRegister, kReservedRegister,

	// Reg 0x10: Register read 0 control
	kEmptyRegister,
	// Reg 0x11: Register read 0 mailbox status
	kEmptyRegister,
	// Reg 0x12: Register read 0 mailbox
	kEmptyRegister,
	// Reg 0x13: Register read 0 reserved
	kReservedRegister,
	// Reg 0x14: Register read 1 control
	kEmptyRegister,
	// Reg 0x15: Register read 1 mailbox status
	kEmptyRegister,
	// Reg 0x16: Register read 1 mailbox
	kEmptyRegister,
	// Reg 0x17: Register read 1 reserved
	kReservedRegister,
	// Reg 0x18: Register read 2 control
	kEmptyRegister,
	// Reg 0x19: Register read 2 mailbox status
	kEmptyRegister,
	// Reg 0x1A: Register read 2 mailbox
	kEmptyRegister,
	// Reg 0x1B: Register read 2 reserved
	kReservedRegister,

	// Reg 0x1C - 0x1F: Reserved
	kReservedRegister, kReservedRegister, kReservedRegister, kReservedRegister,

	// Reg 0x20: Register write 0 control
	kEmptyRegister,
	// Reg 0x21: Register write 0 mailbox status
	kEmptyRegister,
	// Reg 0x22: Register write 0 mailbox
	kEmptyRegister,
	// Reg 0x23: Register write 0 reserved
	kReservedRegister,
	// Reg 0x24: Register write 1 control
	kEmptyRegister,
	// Reg 0x25: Register write 1 mailbox status
	kEmptyRegister,
	// Reg 0x26: Register write 1 mailbox
	kEmptyRegister,
	// Reg 0x27: Register write 1 reserved
	kReservedRegister,
	// Reg 0x28: Register write 2 control
	kEmptyRegister,
	// Reg 0x29: Register write 2 mailbox status
	kEmptyRegister,
	// Reg 0x2A: Register write 2 mailbox
	kEmptyRegister,
	// Reg 0x2B: Register write 2 reserved
	kReservedRegister,
};



/**
 * Initializes the controller's I2C interface.
 */
void controller_i2c_init(void) {
	int err;

	// clear state
	memset(&gState, 0, sizeof(gState));

	// set up the task
	gState.task = xTaskCreateStatic(controller_i2c_task, "I2CCtrlr",
			kControllerStackSize, NULL, kTaskPriorityController,
			(void *) &gState.taskStack, &gState.taskTCB);

	if(gState.task == NULL) {
		LOG_PUTS("couldn't create controller task");
		asm volatile("bkpt 0");
	}

	// run init routines
	controller_i2c_reg_init_t *init;
	init = (controller_i2c_reg_init_t *) &gInitRoutines;

	while(*init != NULL) {
		(*init)();
		init++;
	}

	// initialize driver
	err = i2c_init(&gCallbacks, (i2c_register_t *) &gRegs, kNumRegs);

	if(err < kErrSuccess) {
		LOG("i2c_init: %d\n", err);
	}
}



/**
 * I2C controller task entry
 */
__attribute__((noreturn)) void controller_i2c_task(void *ctx __attribute__((unused))) {
	int err;
	BaseType_t ok;
	uint32_t notification;

	LOG_PUTS("Controller: ready");

	while(1) {
		// wait for the task notification
		ok = xTaskNotifyWait(0, kNotificationAny, &notification, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG("xTaskNotifyWait: %d\n", ok);
			continue;
		}

		// set the busy flag
		gRegs[0x00].read[0] |= REG_BIT(0);

		// should we start discovery?
		if(notification & kNotificationStartDiscovery) {
			err = controller_discover();

			if(err < kErrSuccess) {
				LOG("controller_discover: %d\n", err);
			}
		}

		// clear the busy flag
		gRegs[0x00].read[0] &= (uint8_t) (~REG_BIT(0));
	}
}



/**
 * Callback function for a register read.
 */
int controller_i2c_reg_read(uint8_t reg) {
	// get the value of the register as it was read
	uint32_t newValue = controller_i2c_get_reg(reg, true);

#if LOG_REG_ACCESS
	LOG("register 0x%02x read (0x%08x)\n", reg, newValue);
#endif

	// call the register specific routine
	return gRoutines[reg].read(reg, newValue);
}

/**
 * Callback function for a register write.
 */
int controller_i2c_reg_write(uint8_t reg) {
	// get the value just written to the register
	uint32_t newValue = controller_i2c_get_reg(reg, false);

#if LOG_REG_ACCESS
	LOG("register 0x%02x written (0x%08x)\n", reg, newValue);
#endif

	// call the register specific routine
	return gRoutines[reg].write(reg, newValue);
}



/**
 * No-op read handler
 */
int reg_read_nop(uint8_t reg __attribute__((unused)),
		uint32_t readValue __attribute__((unused))) {
	return kErrSuccess;
}



/**
 * No-op write handler
 */
int reg_write_nop(uint8_t reg __attribute__((unused)),
		uint32_t writtenValue __attribute__((unused))) {
	return kErrSuccess;
}
/**
 * Copy the written value into the read field of the register.
 */
int reg_write_copy(uint8_t reg, uint32_t writtenValue) {
	controller_i2c_set_reg(reg, true, writtenValue);

	// success
	return kErrSuccess;
}



/**
 * Helper method that returns a register's read or write value as an uint32_t.
 */
uint32_t controller_i2c_get_reg(uint8_t reg, bool read) {
	uint32_t value = 0;

	// get a pointer to the array
	uint8_t *ptr = _controller_i2c_get_reg_ptr(reg, read);

	// copy data
	value |= (uint32_t) ((*ptr++) << 24);
	value |= (uint32_t) ((*ptr++) << 16);
	value |= (uint32_t) ((*ptr++) <<  8);
	value |= (uint32_t) ((*ptr++) <<  0);

	return value;
}

/**
 * Helper method that sets a register's value from an uint32_t.
 */
void controller_i2c_set_reg(uint8_t reg, bool read, uint32_t newValue) {
	// get a pointer to the array
	uint8_t *ptr = _controller_i2c_get_reg_ptr(reg, read);

	// copy data
	*ptr++ = (uint8_t) ((newValue & 0xFF000000) >> 24);
	*ptr++ = (uint8_t) ((newValue & 0x00FF0000) >> 16);
	*ptr++ = (uint8_t) ((newValue & 0x0000FF00) >>  8);
	*ptr++ = (uint8_t) ((newValue & 0x000000FF) >>  0);
}

/**
 * Returns a read/write pointer for the given register.
 */
uint8_t *_controller_i2c_get_reg_ptr(uint8_t reg, bool read) {
	if(read) {
		return (uint8_t *) &gRegs[reg].read;
	} else {
		return (uint8_t *) &gRegs[reg].write;
	}
}
