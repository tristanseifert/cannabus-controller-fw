/*
 * i2c_irqs.h
 *
 * Interrupt configuration and status register related routines
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_I2C_IRQS_H_
#define CONTROLLER_I2C_IRQS_H_

#include <stdint.h>

/**
 * Handles a write to the controller interrupt config register.
 */
int reg_write_ctrl_irq_cfg(uint8_t reg, uint32_t writtenValue);
/**
 * Handles a write to the controller interrupt status register.
 */
int reg_write_ctrl_irq_status(uint8_t reg, uint32_t writtenValue);

/**
 * Handles a write to the CANnabus interrupt status register.
 *
 * The only interrupt that can be acknowledged this way is the discovery done,
 * so we have it in here instead of the CANnabis specific files.
 */
int reg_write_cannabus_irq_status(uint8_t reg, uint32_t writtenValue);



/**
 * Updates the state of IRQ lines related to register reads and writes, as well
 * as updating IRQ status registers.
 */
void reg_update_irqs(void);
/**
 * Updates the state of all IRQ status registers by reading the host IRQ
 * controller state.
 */
void reg_update_irq_regs(void);

#endif /* CONTROLLER_I2C_IRQS_H_ */
