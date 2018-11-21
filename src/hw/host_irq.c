/*
 * host_irq.c
 *
 * On the STM32F042, this IO is mapped to PA0.
 * On the STM32F072, this IO is mapped to PA10.
 *
 * The line uses open collector signalling, so an external pull up resistor is
 * recommended. The internal resistor on the chip is used, but this is in the
 * realm of 50 kOhms and will result in slow rise times.
 *
 *  Created on: Nov 20, 2018
 *      Author: tristan
 */
#include "host_irq.h"
#include "host_irq_private.h"

#include "controller.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/// global state
static host_irq_state_t gIrqState;



/**
 * Initializes the interrupt system.
 */
void host_irq_init(void) {
	// clear state
	memset(&gIrqState, 0, sizeof(gIrqState));

	// set up GPIO
#ifdef STM32F042
	// enable GPIO clock and configure them open drain with a pull up
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= GPIO_MODER_MODER0_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;
#endif
#ifdef STM32F072
	// enable GPIO clock and configure them open drain with a pull up
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER |= GPIO_MODER_MODER10_0;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_10;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;
#endif

	// update IRQ line (it will be unasserted)
	host_irq_update();
}

/**
 * Prepares to modify the IRQ state.
 *
 * For each call to `host_irq_begin()` you must have a call to `host_irq_end` to
 * finishe the IRQ state change.
 */
int host_irq_begin(void) {
	// increment nesting value
	gIrqState.nesting++;

	return kErrSuccess;
}

/**
 * Finishes modifying the IRQ state; at this point, the IRQ line's state will
 * change.
 */
int host_irq_end(void) {
	// if nesting is zero, update IRQ
	if(--gIrqState.nesting == 0) {
		// were any updates made?
		if(gIrqState.numUpdates) {
			host_irq_update();
		}

		// reset update counter
		gIrqState.numUpdates = 0;
	}

	return kErrSuccess;
}



/**
 * Masks/unmasks the given IRQ line.
 */
int host_irq_mask(unsigned int line, host_irq_line_mask_t state) {
	// ensure line is valid
	if(line > 31) {
		return kErrHostIrqInvalidLine;
	}

	// bitmask for this line
	uint32_t bitmask = (uint32_t) (1 << line);

	// want the interrupt unmasked (e.g. mask bit set)?
	if(state == kLineUnmasked) {
		gIrqState.lineMask |= bitmask;
	}
	// otherwise, we want the bit cleaned
	else {
		gIrqState.lineMask &= ~bitmask;
	}

	// increment output counter and log
	gIrqState.numUpdates++;

	LOG("HostIRQ: mask 0x%08x\n", gIrqState.lineMask);

	return kErrSuccess;
}

/**
 * Sets the state of the given IRQ line.
 */
int host_irq_set(unsigned int line, host_irq_line_state_t state) {
	// ensure line is valid
	if(line > 31) {
		return kErrHostIrqInvalidLine;
	}

	// bitmask for this line
	uint32_t bitmask = (uint32_t) (1 << line);

	// setting the bit?
	if(state == kLineAsserted) {
		gIrqState.lineState |= bitmask;
	}
	// clearing the bit
	else {
		gIrqState.lineState &= ~bitmask;
	}

	// increment output counter and log
	gIrqState.numUpdates++;

	LOG("HostIRQ: state 0x%08x\n", gIrqState.lineState);

	return kErrSuccess;
}
/**
 * Gets the state of the given IRQ line.
 *
 * This takes into account the interrupt mask.
 */
host_irq_line_state_t host_irq_get(unsigned int line) {
	uint32_t bitmask = (uint32_t) (1 << line);

	// check the result against it
	if(gIrqState.result & bitmask) {
		return kLineAsserted;
	} else {
		return kLineDeasserted;
	}
}



/**
 * Updates the state of the external interrupt line.
 */
void host_irq_update(void) {
	// enter critical section
	taskENTER_CRITICAL();

	// AND the state with the mask bit
	gIrqState.result = (gIrqState.lineState & gIrqState.lineMask);

	// interrupt should be asserted (low output)
	if(gIrqState.result) {
#ifdef STM32F042
		GPIOA->BSRR |= GPIO_BSRR_BR_0;
#endif
#ifdef STM32F072
		GPIOA->BSRR |= GPIO_BSRR_BR_10;
#endif
	}
	// input should not be asserted (high output)
	else {
#ifdef STM32F042
		GPIOA->BSRR |= GPIO_BSRR_BS_0;
#endif
#ifdef STM32F072
		GPIOA->BSRR |= GPIO_BSRR_BS_10;
#endif
	}

	// exit critical section
	taskEXIT_CRITICAL();

	// log
	LOG("HostIRQ: result 0x%08x\n", gIrqState.result);
}
