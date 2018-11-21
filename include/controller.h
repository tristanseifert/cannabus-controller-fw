#ifndef LICHTENSTEIN_H
#define LICHTENSTEIN_H

#include "stm32f0xx.h"

#include "diag/Trace.h"

#include "FreeRTOSConfig.h"

#include "errors.h"

// debug macros
#ifdef DEBUG
#define LOG(format, ...) trace_printf(format, __VA_ARGS__)
#define LOG_PUTS(string) trace_puts(string)
#else
#define LOG(format, ...)
#define LOG_PUTS(string)
#endif

/**
 * Interrupt priorities: lower numbers are higher priorities!
 *
 * In this instance, we favor CAN interrupts to acknowledge and handle CANnabus
 * messages quicker. That way, data is ready for the I2C handler whenever and
 * we do not run the risk of a slow I2C operation adding latency to the CAN
 * segment.
 */
#define kIRQPriorityCAN				4
#define kIRQPriorityI2C				5

/**
 * Task priorities: lower numbers are lower priorities.
 *
 * In this case, we favor higher priorities for tasks that deal directly with
 * some peripheral, followed by protocol handlers, and then any other general
 * purpose tasks.
 *
 * Additionally, the handling of messages on the CAN bus is given higher
 * priority than handling I2C related tasks.
 */
#define kTaskPriorityMax			(configMAX_PRIORITIES - 1)

#define kTaskPriorityCAN			(kTaskPriorityMax - 0)
#define kTaskPriorityI2C			(kTaskPriorityMax - 1)

#define kTaskPriorityCANnabus	(kTaskPriorityMax - 1)
#define kTaskPriorityController	(kTaskPriorityMax - 2)

#endif
