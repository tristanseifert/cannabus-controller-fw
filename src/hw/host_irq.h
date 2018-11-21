/*
 * host_irq.h
 *
 * Provides an interface to the host interrupt routine line.
 *
 * Access to the line is done by 32 different "sources:" sources can be masked
 * and unmasked independently. At least one unmasked interrupt must be asserted
 * to trigger the external interrupt.
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */

#ifndef HW_HOST_IRQ_H_
#define HW_HOST_IRQ_H_

#include <stdint.h>

/**
 * Host IRQ line numbers
 */
typedef enum {
	kIrqLineControllerError			= 31,
	kIrqLineControllerInit			= 30,

	kIrqLineRegReadOk				= 24,
	kIrqLineRegReadErr				= 23,
	kIrqLineRegReadTimeout			= 21,

	kIrqLineRegWriteOk				= 19,
	kIrqLineRegWriteErr				= 18,
	kIrqLineRegWriteTimeout			= 17,

	kIrqLineDiscoveryDone			= 15,
} host_irq_line_t;



/**
 * IRQ related error codes
 */
enum {
	kErrHostIrqInvalidLine			= -22000
};



/**
 * State of an interrupt line
 */
typedef enum {
	kLineDeasserted					= 0,
	kLineAsserted					= 1
} host_irq_line_state_t;

/**
 * Mask states of an interrupt line
 */
typedef enum {
	kLineMasked						= 0,
	kLineUnmasked					= 1
} host_irq_line_mask_t;



/**
 * Initializes the interrupt system.
 */
void host_irq_init(void);



/**
 * Prepares to modify the IRQ state.
 *
 * For each call to `host_irq_begin()` you must have a call to `host_irq_end` to
 * finishe the IRQ state change.
 */
int host_irq_begin(void);
/**
 * Finishes modifying the IRQ state; at this point, the IRQ line's state will
 * change.
 */
int host_irq_end(void);



/**
 * Masks/unmasks the given IRQ line.
 */
int host_irq_mask(unsigned int line, host_irq_line_mask_t state);

/**
 * Sets the state of the given IRQ line.
 */
int host_irq_set(unsigned int line, host_irq_line_state_t state);
/**
 * Gets the state of the given IRQ line.
 *
 * This takes into account the interrupt mask.
 */
host_irq_line_state_t host_irq_get(unsigned int line);


#endif /* HW_HOST_IRQ_H_ */
