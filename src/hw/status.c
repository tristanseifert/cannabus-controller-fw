/*
 * status.c
 *
 * On the STM32F042 hardware:
 * 	- STATUS[0]: PA7
 * 	- STATUS[1]: PA6
 *
 * On the STM320F72 hardware:
 * 	- STATUS[0]: PC10
 * 	- STATUS[1]: PC12
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */
#include "status.h"

#include "controller.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/// state of the status LEDs
static bool gLEDState[2];


/**
 * Initializes the status LED GPIOs.
 */
void status_init(void) {
	// initialize status
	memset(&gLEDState, 0, sizeof(gLEDState));

	// basically, "restore" the state of all off
	status_restore();
}



/**
 * Sets the state of the specified LED.
 *
 * LEDs are active HIGH.
 */
void status_set(status_led_t led, bool on) {
	// set last state
	gLEDState[led] = on;

#ifdef STM32F042
	switch(led) {
	case kStatusLED0:
		if(on) {
			GPIOA->ODR |= GPIO_ODR_7;
		} else {
			GPIOA->ODR &= (uint16_t) ~GPIO_ODR_7;
		}
		break;

	case kStatusLED1:
		if(on) {
			GPIOA->ODR |= GPIO_ODR_6;
		} else {
			GPIOA->ODR &= (uint16_t) ~GPIO_ODR_6;
		}
		break;
	}
#endif
#ifdef STM32F072
	switch(led) {
	case kStatusLED0:
		if(on) {
			GPIOC->ODR |= GPIO_ODR_10;
		} else {
			GPIOC->ODR &= (uint16_t) ~GPIO_ODR_10;
		}
		break;

	case kStatusLED1:
		if(on) {
			GPIOC->ODR |= GPIO_ODR_12;
		} else {
			GPIOC->ODR &= (uint16_t) ~GPIO_ODR_12;
		}
		break;
	}
#endif
}

/**
 * Gets the state of the specified LED.
 */
bool status_get(status_led_t led) {
	return gLEDState[led];
}



/**
 * For the STM32F042 target, the status outputs are multiplexed with the SPI
 * pins. This function restores the proper pin state once the SPI peripheral is
 * no logner needed.
 */
void status_restore(void) {
	// set up GPIOs
#ifdef STM32F042
	// enable GPIO clock and configure them outputs
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOA->MODER &= (uint32_t) ~
			((GPIO_MODER_MODER7_0 | GPIO_MODER_MODER7_1) |
			(GPIO_MODER_MODER6_0 | GPIO_MODER_MODER6_1));
	GPIOA->MODER |= GPIO_MODER_MODER7_0 | GPIO_MODER_MODER6_0;
#endif
#ifdef STM32F072
	// enable GPIO clock and configure them outputs
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	GPIOC->MODER |= GPIO_MODER_MODER10_0 | GPIO_MODER_MODER12_0;
#endif

	// set both LEDs to off
	status_set(kStatusLED0, gLEDState[0]);
	status_set(kStatusLED1, gLEDState[1]);
}



/**
 * Context switching callback from FreeRTOS: when the idle task is switched in,
 * STATUS[0] turns off, but when another task is switched in, it is turned on.
 * This makes that status LED basically an indicator of CPU load.
 */
void status_handle_context_switch(void) {
	// was the idle task switched in?
	if(xTaskGetCurrentTaskHandle() == xTaskGetIdleTaskHandle()) {
		status_set(kStatusLED0, false);
	} else {
		status_set(kStatusLED0, true);
	}
}
