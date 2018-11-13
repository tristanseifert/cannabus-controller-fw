/*
 * status.h
 *
 * Provides an interface to the status LEDs on the board.
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */

#ifndef HW_STATUS_H_
#define HW_STATUS_H_

#include <stdbool.h>

/**
 * Type describing a status LED
 */
typedef enum {
	kStatusLED0						= 0,
	kStatusLED1						= 1,
} status_led_t;


/**
 * Initializes the status LED GPIOs.
 */
void status_init(void);

/**
 * Sets the state of the specified LED.
 */
void status_set(status_led_t led, bool on);

/**
 * Gets the state of the specified LED.
 */
bool status_get(status_led_t led);



/**
 * For the STM32F042 target, the status outputs are multiplexed with the SPI
 * pins. This function restores the proper pin state once the SPI peripheral is
 * no logner needed.
 */
void status_restore(void);

/**
 * Context switching callback from FreeRTOS: when the idle task is switched in,
 * STATUS[0] turns off, but when another task is switched in, it is turned on.
 * This makes that status LED basically an indicator of CPU load.
 */
void status_handle_context_switch(void);


#endif /* HW_STATUS_H_ */
