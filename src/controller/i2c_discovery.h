/*
 * i2c_discovery.h
 *
 * Handles discovery-related tasks.
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_I2C_DISCOVERY_H_
#define CONTROLLER_I2C_DISCOVERY_H_

#include <stdint.h>

/**
 * Task entry point for discovering devices.
 */
int controller_discover(void);



/**
 * Updates the device discovery registers.
 */
void reg_disc_update(void);
/**
 * Updates the discovery mailbox.
 */
void reg_disc_update_mailbox(void);

/**
 * Resets the mailbox.
 */
void reg_disc_reset_mailbox(void);



/**
 * Handles a write to the device discovery control register.
 */
int reg_write_disc_control(uint8_t reg, uint32_t writtenValue);
/**
 * Handles a write to the device discovery mailbox status register.
 */
int reg_write_disc_mailbox_status(uint8_t reg, uint32_t writtenValue);

/**
 * Handles a read from the device discovery mailbox.
 */
int reg_read_disc_mailbox(uint8_t reg, uint32_t readValue);


#endif /* CONTROLLER_I2C_DISCOVERY_H_ */