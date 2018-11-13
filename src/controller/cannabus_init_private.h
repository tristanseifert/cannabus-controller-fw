/*
 * cannabus_init_private.h
 *
 * Prototypes of Cannabus callbacks
 *
 *  Created on: Nov 13, 2018
 *      Author: tristan
 */

#ifndef CONTROLLER_CANNABUS_INIT_PRIVATE_H_
#define CONTROLLER_CANNABUS_INIT_PRIVATE_H_

#include "../cannabus/cannabus.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * CANnabus callback: initializes CAN bus.
 */
int controller_cannabus_can_init(void);
/**
 * CANnabus callback: configures a mask-based filter.
 */
int controller_cannabus_can_config_filter(unsigned int filter, uint32_t mask, uint32_t identifier);
/**
 * CANnabus callback: are there any messages waiting?
 */
bool controller_cannabus_can_rx_waiting(void);
/**
 * CANnabus callback: receives a message from CAN peripheral.
 */
int controller_cannabus_can_rx_message(cannabus_can_frame_t *frame);
/**
 * CANnabus callback: transmits a message.
 */
int controller_cannabus_can_tx_message(cannabus_can_frame_t *frame);



/**
 * CANnabus callback: returns the device firmware version.
 */
uint16_t controller_cannabus_get_fw_version(void);

#endif /* CONTROLLER_CANNABUS_INIT_PRIVATE_H_ */
