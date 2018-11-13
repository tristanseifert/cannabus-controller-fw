#include <stdio.h>
#include <stdlib.h>

#include "controller.h"

#include "gitcommit.h"

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

	// if we get down here, reset the system
	NVIC_SystemReset();
}
