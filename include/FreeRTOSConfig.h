/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#include <stdint.h>
extern uint32_t SystemCoreClock;

// task switching configuration
#define configUSE_PREEMPTION			1

// kernel ticks
#define configUSE_TICK_HOOK				0
#define configCPU_CLOCK_HZ				SystemCoreClock
#define configTICK_RATE_HZ				((TickType_t) 1000)
#define configUSE_16_BIT_TICKS			0

// task priorities and stack sizes
#define configMAX_PRIORITIES			(5)
#define configMINIMAL_STACK_SIZE		((unsigned short) 60)

#define configMAX_TASK_NAME_LEN			10

// TODO: we should define configKERNEL_INTERRUPT_PRIORITY and configMAX_SYSCALL_INTERRUPT_PRIORITY
// interrupt priorities
#define configKERNEL_INTERRUPT_PRIORITY			0
#define configMAX_SYSCALL_INTERRUPT_PRIORITY		8

// idle task
#define configIDLE_SHOULD_YIELD			1
#define configUSE_IDLE_HOOK				1


// debugging
#ifdef DEBUG
	#define configUSE_TRACE_FACILITY		1
	#define configQUEUE_REGISTRY_SIZE		8
	#define configGENERATE_RUN_TIME_STATS	0
#else
	#define configUSE_TRACE_FACILITY		0
	#define configQUEUE_REGISTRY_SIZE		0
	#define configGENERATE_RUN_TIME_STATS	0
#endif

// stack overflow checking
#define configCHECK_FOR_STACK_OVERFLOW	2
#define configRECORD_STACK_HIGH_ADDRESS	1

// semaphores
#define configUSE_COUNTING_SEMAPHORES	0

// mutexes
#define configUSE_MUTEXES				1
#define configUSE_RECURSIVE_MUTEXES		0

// use direct to task notifications
#define configUSE_TASK_NOTIFICATIONS		1

// do not use port specific task selection
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	0

// do not use dynamic memory
#define configSUPPORT_STATIC_ALLOCATION		1
#define configSUPPORT_DYNAMIC_ALLOCATION	0

// don't use coroutines
#define configUSE_CO_ROUTINES 			0

// don't use timers
#define configUSE_TIMERS				0
#define configTIMER_TASK_PRIORITY		2
#define configTIMER_QUEUE_LENGTH		5
#define configTIMER_TASK_STACK_DEPTH	80

// task-related API functions to include
#define INCLUDE_vTaskPrioritySet		0
#define INCLUDE_uxTaskPriorityGet		0
#define INCLUDE_vTaskDelete				0
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			0
#define INCLUDE_vTaskDelayUntil			0
#define INCLUDE_vTaskDelay				0

// enable stuff to be able to get CPU usage in debug mode
#ifdef DEBUG
#define INCLUDE_xTaskGetIdleTaskHandle	1
#endif

// assert for FreeRTOS
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); __asm volatile("bkpt 0"); for( ;; ); }

// define some conversion helpers
#define pdMSTOTICKS(xTimeInMs) ((TickType_t) (((TickType_t) (xTimeInMs) * (TickType_t) configTICK_RATE_HZ) / (TickType_t) 1000))

// mapping of ISR names
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

// set the status handler for context switches
#define traceTASK_SWITCHED_IN()		status_handle_context_switch()

#endif /* FREERTOS_CONFIG_H */

