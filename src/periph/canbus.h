/*
 * canbus.h
 *
 * Provides an interface to the CAN bus. This driver is mostly interrupt and
 * DMA driven, and maintains an internal queue of messages that have been
 * received, so reception happens in the background.
 *
 * Transmitting of messages is blocking and will not return until the message
 * has been handed over to the CAN peripheral, blocking if there are no transmit
 * FIFO slots available.
 *
 * Filtering is possible by specifying one exact match, and one mask filter.
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */

#ifndef can_H_
#define can_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * CAN bus errors
 */
enum {
	/// no tx mailbox is free to send this frame, try again later
	kErrCanNoFreeMailbox			= -2000,
	/// frame is invalid
	kErrCanInvalidFrame				= -2001,
	/// no frames received since last call
	kErrCanNoRxFramesPending		= -2002,
	/// tx timeout. maybe broken bus?
	kErrCanTxTimeout				= -2003,
};



/**
 * A single CAN message.
 */
typedef struct {
	/// is this a valid message?
	uint32_t valid		: 1;

	/// CAN identifier: only the low 29 bits are used.
	uint32_t identifier	: 29;
	// is this message a remote transmission request?
	uint32_t rtr		: 1;

	/// data length
	uint8_t length;
	/// data
	uint8_t data[8];
} can_message_t;



/**
 * Initializes the CAN peripheral.
 */
int can_init(void);

/**
 * Starts the CAN driver.
 */
void can_start(void);

/**
 * Stops the CAN driver.
 */
void can_stop(void);



/**
 * Configures a filter bank to work as an exact identifier filter.
 *
 * @param bank The bank to configure: this should be [1, 14].
 *
 * @note Only the low 29 bits are considered.
 */
int can_filter_exact(unsigned int bank, uint32_t identifier);

/**
 * Configures a filter bank to work as a mask identifier filter.
 *
 * @param bank The bank to configure: this should be [1, 14].
 *
 * @note Only the low 29 bits of the identifier and mask are considered.
 */
int can_filter_mask(unsigned int bank, uint32_t mask, uint32_t identifier);


/**
 * Are there any messages waiting to be read?
 */
bool can_messages_available(void);

/**
 * Copies the oldest message to the specified buffer, then removes it from the
 * internal queue.
 *
 * This function blocks the calling task until a frame is available.
 */
int can_get_last_message(can_message_t *msg);



/**
 * Transmits the given message.
 */
int can_transmit_message(can_message_t *msg);



#endif /* can_H_ */
