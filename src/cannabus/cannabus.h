/*
 * cannabus.h
 *
 * Provides an interface to the CANnabus.
 *
 *  Created on: Nov 2, 2018
 *      Author: tristan
 */

#ifndef CANNABUS_CANNABUS_H_
#define CANNABUS_CANNABUS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * CANnabus error codes
 */
enum {
	kErrCannabusUnimplemented		= -42000,
	kErrCannabusNodeIdMismatch		= -42001,
	kErrCannabusInvalidFrameSize	= -42002,
	kErrCannabusCRCInvalid			= -42003,
};

/**
 * CANnabus nodes have a 16-bit node address
 */
typedef uint16_t cannabus_addr_t;

/**
 * A single CAN message, received from the bus.
 *
 * The identifier is structured as 2 bits of priority, a 16 bit ID, followed by
 * an 11 bit register value.
 */
typedef struct {
	/// CAN identifier
	uint32_t identifier	: 29;
	/// is this a remote transmission request?
	uint32_t rtr		: 1;

	/// length of data
	uint8_t data_len;
	/// data (up to 8 bytes)
	uint8_t data[8];
} cannabus_can_frame_t;

/**
 * A CANnabus request.
 *
 * We do not care about the priority or device ID here.
 */
typedef struct {
	/// was this a broadcast message?
	uint16_t broadcast	: 1;
	/// register to access
	uint16_t reg		: 11;
	/// was this message a remote transmission request?
	uint16_t rtr		: 1;
	/// was this frame an acknowledgement?
	uint16_t ack		: 1;
	/// is this frame marked as 'high priority?'
	uint16_t	 priority	: 1;

	/// how many bytes of data do we have?
	uint8_t data_len;
	/// data (up to 8 bytes)
	uint8_t data[8];
} cannabus_operation_t;



/**
 * Function pointers to interface with the CAN driver and system software to
 * handle CANnabus register accesses.
 */
typedef struct {
	/// prepares the CAN bus for communication.
	int (*can_init)(void);

	/// sets one of four mask-based CAN identifier filters in the CAN driver
	int (*can_config_filter)(unsigned int, uint32_t, uint32_t);

	/// check if there are CAN bus messages pending, blocking the task if not
	bool (*can_rx_waiting)(void);
	/// retrieve the last CAN message
	int (*can_rx_message)(cannabus_can_frame_t *);

	/// transmits a frame on the CAN bus.
	int (*can_tx_message)(cannabus_can_frame_t *);

	/// returns the firmware version.
	uint16_t (*get_fw_version)(void);

	/// begins a firmware update session, with the given CRC.
	int (*upgrade_begin)(uint16_t);
	/// writes n bytes of firmware data
	int (*upgrade_write)(size_t, void *);
	/// finishes a firmware update session. if the CRC is invalid this errors
	int (*upgrade_end)(void);
	/// resets the device after a firmware has been written.
	int (*upgrade_reset)(void);

	/// handles an incoming CANnabus message, if not a system message
	int (*handle_operation)(cannabus_operation_t *);
} cannabus_callbacks_t;



/**
 * Initializes the CANnabus and sets this node's address.
 */
int cannabus_init(cannabus_addr_t addr, uint8_t deviceType, uint8_t fwUpgradeCapabilities, const cannabus_callbacks_t *callbacks);

/**
 * Changes the node's address.
 */
int cannabus_set_address(cannabus_addr_t addr);



/**
 * Gets any waiting messages from the CAN bus driver and processes them.
 *
 * @returns A negative error code, or the number of messages processed.
 */
int cannabus_process(void);



/**
 * Sends the given operation on the bus.
 */
int cannabus_send_op(cannabus_operation_t *op);

/**
 * Acknowledges a received operation.
 *
 * An acknowledgment is a frame with no data, with the same address as the
 * last write, but with the second highest bit set in the identifier.
 */
int cannabus_ack_received(cannabus_operation_t *op);


#endif /* CANNABUS_CANNABUS_H_ */
