/*
 * freertos_support.c
 *
 *  Created on: Nov 6, 2018
 *      Author: tristan
 */
#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"

// declare prototypes: freertos should do this for us
void vApplicationGetIdleTaskMemory(StaticTask_t**, StackType_t**, uint32_t*);
void vApplicationGetTimerTaskMemory(StaticTask_t**, StackType_t**, uint32_t*);

void vApplicationStackOverflowHook(TaskHandle_t, char*);
void vApplicationIdleHook(void);


#if configSUPPORT_STATIC_ALLOCATION
// memory for idle task
static StaticTask_t gIdleTaskTCB;
static StackType_t gIdleTaskStack[configMINIMAL_STACK_SIZE];

// memory for timer task
#if configUSE_TIMERS
	static StaticTask_t gTimerTaskTCB;
	static StackType_t gTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
#endif

/**
 * Provides the memory used by the idle task.
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **taskTCB, StackType_t **taskStack, uint32_t *taskStackSz) {
    *taskTCB = &gIdleTaskTCB;
    *taskStack = gIdleTaskStack;
    *taskStackSz = configMINIMAL_STACK_SIZE;
}


#if configUSE_TIMERS
/**
 * Provides the memory used by the timer task.
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **taskTCB, StackType_t **taskStack, uint32_t *taskStackSz) {
    *taskTCB = &gTimerTaskTCB;
    *taskStack = gTimerTaskStack;
    *taskStackSz = configTIMER_TASK_STACK_DEPTH;
}
#endif
#endif



/**
 * Handles stack overflow in a task.
 *
 * On debug builds. this will log info and step into the debugger, while on
 * release builds this resets the system.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName) {
	LOG("stack overflow in task %s", pcTaskName);

#ifdef DEBUG
	__asm volatile("bkpt 0");
#else
	NVIC_SystemReset();
#endif
}



/**
 * Called when the idle task executes. This puts the CPU into a slightly lower
 * power state.
 */
void vApplicationIdleHook(void) {
	__WFI();
}
