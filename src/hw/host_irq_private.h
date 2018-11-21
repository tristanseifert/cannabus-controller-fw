/*
 * host_irq_private.h
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */

#ifndef HW_HOST_IRQ_PRIVATE_H_
#define HW_HOST_IRQ_PRIVATE_H_

#include <stdint.h>

/**
 * Internal state for the host IRQ system
 *
 * Each interrupt line, 0-31, is implemented as one of the bits of these ints.
 */
typedef struct {
	/// state of each line
	uint32_t lineState;
	/// mask for each line
	uint32_t lineMask;

	/// resulting interrupt value
	uint32_t result;

	/// nesting level of the updates
	int nesting;
	/// number of updates performed
	int numUpdates;
} host_irq_state_t;



/**
 * Updates the state of the external interrupt line.
 */
void host_irq_update(void);

#endif /* HW_HOST_IRQ_PRIVATE_H_ */
