#ifndef LICHTENSTEIN_H
#define LICHTENSTEIN_H

#include "stm32f0xx.h"

#include "diag/Trace.h"

#include "errors.h"

// debug macros
#ifdef DEBUG
#define LOG(format, ...) trace_printf(format, __VA_ARGS__)
#define LOG_PUTS(string) trace_puts(string)
#else
#define LOG(format, ...)
#define LOG_PUTS(string)
#endif

#endif
