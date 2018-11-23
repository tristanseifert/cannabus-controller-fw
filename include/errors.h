/*
 * errors.h
 *
 * Error codes used throughout the software.
 *
 *  Created on: Nov 3, 2018
 *      Author: tristan
 */

#ifndef INCLUDE_ERRORS_H_
#define INCLUDE_ERRORS_H_

enum {
	kErrSuccess					= 0,

	/// attempted call is not implemented
	kErrUnimplemented			= -999,

	// generic errors
	kErrInvalidArgs				= -1000,
	kErrInsufficientResources	= -1001,
	/// could not create a task (xTaskCreate* returned NULL)
	kErrTaskCreationFailed		= -1002,
	/// could not create a semaphore (xSemaphoreCreate* returned NULL)
	kErrSemaphoreCreationFailed	= -1003,
	/// could not create a queue (xQueueCreate* returned NULL)
	kErrQueueCreationFailed		= -1004,
	/// could not create a timer (xTimerCreate* returned NULL)
	kErrTimerCreationFailed		= -1005,

	/// an error occurred when receiving from a queue
	kErrQueueReceive			= -1010,
	/// an error occurred when sending to a queue
	kErrQueueSend				= -1011,

	/// an error occurred when notifying another task
	kErrNotify					= -1020,

	/// an error occurred when trying to take a mutex
	kErrMutexTake				= -1030,
	/// an error occurred when trying to give a mutex
	kErrMutexGive				= -1031,

	/// error when changing a timer's state
	kErrTimer					= -1040,

};


#endif /* INCLUDE_ERRORS_H_ */
