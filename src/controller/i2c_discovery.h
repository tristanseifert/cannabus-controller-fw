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

#include "../cannabus/cannabus.h"

#include <stdint.h>

/// maximum number of devices to allocate space for in the discovery struct
#define kDiscoveryMaxDevices		8

/// forward declare task msg type
#ifndef controller_i2c_task_msg_t
	typedef struct controller_i2c_task_msg controller_i2c_task_msg_t;
#endif

/**
 * Discovery state
 */
typedef struct {
	/// is device discovery in progress?
	unsigned int inProgress		: 1;

	/// number of ticks to wait for discovery to complete
	unsigned int timeout		: 8;

	/// were more than the maximum number of devices discovered?
	unsigned int dataOverflow	: 1;
	/// number of devices that have been discovered
	unsigned int devices		: 4;

	/// what offset into the device IDs array is output on the mailbox?
	unsigned int mailboxView	: 3;

	/// IDs of discovered devices
	uint16_t deviceIds[kDiscoveryMaxDevices];
} controller_i2c_discovery_state_t;


/**
 * Task entry point for discovering devices.
 */
int controller_discover(controller_i2c_task_msg_t *msg);

/**
 * Callback for the discovery process. Each time we receive data, we extract the
 * device ID from the frame and add it to the mailbox.
 *
 * When the error value is kErrCannabusTimeout, notify task since discovery is
 * then done.
 */
int controller_discover_io_callback(int, uint32_t, cannabus_operation_t *);



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
