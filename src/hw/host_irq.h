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
 * Masks/unmasks the given IRQ line.
 */
int host_irq_mask(unsigned int line, host_irq_line_mask_t state);

/**
 * Sets the state of the given IRQ line.
 */
int host_irq_set(unsigned int line, host_irq_line_state_t state);


#endif /* HW_HOST_IRQ_H_ */
