#include <stdio.h>
#include <stdlib.h>

#include "controller.h"

#include "periph/canbus.h"
#include "hw/status.h"
#include "hw/host_irq.h"
#include "controller/cannabus_init.h"
#include "controller/i2c_init.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gitcommit.h"

/**
 * Performs initialization of all hardware.
 */
static void init_hardware(void) {
#ifdef STM32F042
	status_init();
	host_irq_init();
	can_init();
#endif
#ifdef STM32F072
	status_init();
	host_irq_init();
	can_init();
#endif
}

/**
 * Application entry point
 */
__attribute__((noreturn)) int main(int argc __attribute__((__unused__)), char* argv[]__attribute__((__unused__))) {
#ifdef DEBUG
	// initialize trace
	trace_initialize();
	LOG("cannabus-controller-fw, git rev %s\n", GIT_INFO);

#ifdef STM32F042
	LOG_PUTS("hw: STM32F042");
#endif
#ifdef STM32F072
	LOG_PUTS("hw: STM32F072");
#endif
#endif

	// initialize hardware
	init_hardware();

	// initialize I2C interface
	controller_i2c_init();

	// initialize CANnabus
	controller_cannabus_init();

	// start FreeRTOS scheduler. this should not return
	vTaskStartScheduler();
	NVIC_SystemReset();
}
