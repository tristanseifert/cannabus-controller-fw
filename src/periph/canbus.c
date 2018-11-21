/*
 * canbus.c
 *
 * On the STM32F042 based hardware:
 * CAN_RX:	PA11 (remap)
 * CAN_TX:	PA12 (remap)
 *
 * On the STM32F072 Nucleo64 board:
 * CAN_RX:	PA11
 * CAN_TX:	PA12
 *
 * BTR register value calculated with http://www.bittiming.can-wiki.info/
 *
 *  Created on: Nov 1, 2018
 *      Author: tristan
 */
#include "canbus.h"
#include "canbus_private.h"

#include "controller.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/// internal state
static can_state_t gCANState;


/**
 * Initializes the CAN peripheral.
 */
int can_init(void) {
	// clear state
	memset(&gCANState, 0, sizeof(gCANState));

	// create rx queue
	gCANState.rxQueue = xQueueCreateStatic(kCANRxBufferSize, sizeof(can_message_t),
			(void *) &gCANState.rxQueueBuffer, &gCANState.rxQueueStruct);

	if(gCANState.rxQueue == NULL) {
		return kErrQueueCreationFailed;
	}

	// create task message queue
	gCANState.taskMsgQueue = xQueueCreateStatic(kCANMsgBufferSize,
			sizeof(can_task_msg_t), (void *) &gCANState.taskMsgBuffer,
			&gCANState.taskMsgQueueStruct);

	if(gCANState.taskMsgQueue == NULL) {
		return kErrQueueCreationFailed;
	}

	// create the task
	gCANState.task = xTaskCreateStatic(can_task, "CAN",
			kCANStackSize, NULL, kTaskPriorityCAN,
			(void *) &gCANState.taskStack, &gCANState.taskTCB);

	if(gCANState.task == NULL) {
		return kErrTaskCreationFailed;
	}

//#ifdef STM32F042
	// enable GPIO, SYSCFG clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	// remap PA11/PA12 to the pins on STM32F042
#ifdef STM32F042
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
#endif

	// configure PA11 and PA12 as high speed alternate function outputs
	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR11 | GPIO_OSPEEDER_OSPEEDR12);

	// configure PA11 and PA12 alternate functions
	GPIOA->AFR[1] |= 0x04 << (3 * 4);
	GPIOA->AFR[1] |= 0x04 << (4 * 4);
/*#endif
#ifdef STM32F072
	// enable GPIO clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// configure PB8 and PB9 as high speed alternate function outputs
	GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);

	// configure PB8 and PB9 alternate functions
	GPIOB->AFR[1] |= 0x04 << (0 * 4);
	GPIOB->AFR[1] |= 0x04 << (1 * 4);
#endif*/

	// enable CAN clock and reset
	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	RCC->APB1RSTR |= RCC_APB1RSTR_CANRST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;

	// exit sleep mode
	CAN->MCR &= (uint32_t) ~CAN_MCR_SLEEP;

	// enter initialization mode
	CAN->MCR |= CAN_MCR_INRQ;
	while(!(CAN->MSR & CAN_MSR_INAK)) {}

	/*
	 * Configure CAN master control:
	 *
	 * - Automatic bus-off management
	 * - Automatic wake-up mode
	 * - Discard messages if FIFO is full
	 *
	 */
	CAN->MCR |= CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_RFLM /*| CAN_MCR_NART*/;

	/**
	 * Enable CAN interrupts:
	 *
	 * - Error interrupt
	 * - Bus off interrupt
	 * - Error passive interrupt
	 * - FIFO0/1 full
	 * - FIFO0/1 message pending
	 * - TX mailbox empty
	 */
	CAN->IER |= CAN_IER_ERRIE | CAN_IER_BOFIE | CAN_IER_EPVIE |
			CAN_IER_FFIE0 | CAN_IER_FMPIE0 |
			CAN_IER_FFIE1 | CAN_IER_FMPIE1 |
			CAN_IER_TMEIE;

	// unmask interrupts
	NVIC_EnableIRQ(CEC_CAN_IRQn);
	NVIC_SetPriority(CEC_CAN_IRQn, kIRQPriorityCAN);

	// configure CAN bit timing for 125kbps nominal
	CAN->BTR = 0x001c0017;

	// success
	return kErrSuccess;
}

/**
 * Starts the CAN driver.
 */
void can_start(void) {
	// clear the queue
	xQueueReset(gCANState.rxQueue);

	// exit initialization mode; wait for INAK clear
	CAN->MCR &= (uint32_t) ~CAN_MCR_INRQ;
	while((CAN->MSR & CAN_MSR_INAK)) {}
}


/**
 * Stops the CAN driver.
 */
void can_stop(void) {
	// re-enter initialization mode
	CAN->MCR |= CAN_MCR_INRQ;
	while(!(CAN->MSR & CAN_MSR_INAK)) {}
}



/**
 * Configures the exact identifier filter.
 *
 * This is done with filter bank 0. We configure it in mask mode such that we
 * get the message regardless of the RTR bit.
 *
 * Filtered messages are dropped into FIFO0.
 *
 * @note Only the low 29 bits are considered.
 */
int can_filter_exact(unsigned int bank, uint32_t identifier) {
	return can_filter_mask(bank, 0x1FFFFFFF, identifier);
}

/**
 * Configures the mask identifier filter.
 *
 * @param bank The bank to configure: this should be [1, 14].
 *
 * @note Only the low 29 bits of the identifier and mask are considered.
 */
int can_filter_mask(unsigned int _bank, uint32_t mask, uint32_t identifier) {
	// make bank zero based, validate it, then get the bit flag for config regs
	unsigned int bank = _bank - 1;

	if(bank > 13) {
//		LOG("invalid bank: %u\n", bank);
		return kErrInvalidArgs;
	}

	uint32_t bankFlag = 1UL << (bank);

	// enable filter initialization mode, deactivate filter
	CAN->FMR |= CAN_FMR_FINIT;
	CAN->FA1R &= (uint32_t) ~bankFlag;


	// filter is configured as a single 32-bit filter in mask mode
	CAN->FS1R |= bankFlag;
	CAN->FM1R &= (uint32_t) ~bankFlag;

	// set the mask (high 29 bits, then IDE is 1)
	CAN->sFilterRegister[bank].FR2 = ((mask & 0x1FFFFFFF) << 3) | 0x04;
	// copy low 29 bits of address, shift left 3, then add IDE bit (0x04)
	CAN->sFilterRegister[bank].FR1 = ((identifier & 0x1FFFFFFF) << 3) | 0x04;

	// Use FIFO 0
	CAN->FFA1R &= (uint32_t) ~bankFlag;

	// activate filter, then disable filter init mode
	CAN->FA1R |= bankFlag;
	CAN->FMR &= (uint32_t) ~CAN_FMR_FINIT;

	// success
	return kErrSuccess;
}



/**
 * Entry point for the CAN bus task.
 *
 * This task is woken up by various events being sent to it from ISRs or other
 * locations. The FreeRTOS notification field is used for this:
 *
 * - Bit 0: CAN error
 */
__attribute__((noreturn)) void can_task( __attribute__((unused)) void *ctx) {
	BaseType_t ok;
	can_task_msg_t msg;

	while(1) {
		// wait for a message
		ok = xQueueReceive(gCANState.taskMsgQueue, &msg, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG("xQueueReceive: %d\n", ok);
			continue;
		}

		// handle the message
		switch(msg.type) {
		// transmit the frame?
		case kCANTaskMsgTransmit:
			ok = can_task_tx_message(&msg);

			if(ok < kErrSuccess) {
				LOG("can_task_tx_message: %d\n", ok);
			}

			break;

		// CAN error detected?
		case kCANTaskHandleError:
			LOG("CAN error: %x\n", msg.canError);
			break;

		// invalid message type
		default:
			LOG("invalid can_task_msg type: %x\n", msg.type);
			break;
		}
	}
}

/**
 * Transmits a message.
 */
int can_task_tx_message(can_task_msg_t *msg) {
	// find a free transmit mailbox
	int box = can_find_free_tx_mailbox();

	if(box < 0) {
		return kErrCanNoFreeMailbox;
	}

	// attempt to transmit message into that mailbox
	return can_tx_with_mailbox(box, &msg->txMessage);
}



/**
 * Are there any messages waiting to be read?
 */
bool can_messages_available(void) {
	return (uxQueueMessagesWaiting(gCANState.rxQueue) != 0);
}

/**
 * Copies the oldest message to the specified buffer, then removes it from the
 * internal queue.
 *
 * This function blocks the calling task until a frame is available.
 */
int can_get_last_message(can_message_t *msg) {
	BaseType_t ok;

	// buffer cannot be null
	if(msg == NULL) {
		return kErrInvalidArgs;
	}

	// attempt to dequeue a message
	ok = xQueueReceive(gCANState.rxQueue, msg, portMAX_DELAY);

	if(ok != pdTRUE) {
		return kErrQueueReceive;
	}

	return kErrSuccess;
}



/**
 * Transmits the given message.
 */
int can_transmit_message(can_message_t *txMsg) {
	BaseType_t ok;

	// construct the message
	can_task_msg_t msg;
	msg.type = kCANTaskMsgTransmit;
	memcpy(&msg.txMessage, txMsg, sizeof(can_message_t));

	// send it
	ok = xQueueSendToBack(gCANState.taskMsgQueue, &msg, portMAX_DELAY);

	if(ok != pdTRUE) {
		return kErrQueueSend;
	}

	// otherwise, the frame was queued
	return kErrSuccess;
}



/**
 * Finds the next available transmit mailbox.
 *
 * This will wait up to 500ms for a mailbox to open up. If no mailbox is then
 * available, the call will error out.
 */
int can_find_free_tx_mailbox(void) {
	unsigned int counter = 0;

	BaseType_t ok;
	uint32_t notification;

	while(true) {
		// is mailbox 0 empty?
		if((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
			return 0;
		}
		// is mailbox 1 empty?
		else if((CAN->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
			return 1;
		}
		// is mailbox 2 empty?
		else if((CAN->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
			return 2;
		}

		// was this the 6th time we've done this?
		if(++counter == 6) {
			LOG_PUTS("wait for tx mailbox timed out");
			// if so, give up :(
			break;
		}

		// wait on the notification
checkNotification: ;
		ok = xTaskNotifyWait(0, 0x80000000, &notification, pdMSTOTICKS(100));

		if(ok == pdTRUE) {
			// we received one, check if it was the one we care about
			if(notification & 0x80000000) {
				// if so, find a free mailbox
				continue;
			}
			// otherwise, we got a random notification, so check again
			else {
				goto checkNotification;
			}
		}
		// the notification timed out. try again
		else {

		}
	}

	// no mailbox could be found that's free
	return kErrCanNoFreeMailbox;
}

/**
 * Transmits the given message on the given mailbox.
 */
int can_tx_with_mailbox(int mailbox, can_message_t *msg) {
	uint32_t reqAckFlag = 0, txOkFlag = 0, mailboxEmptyFlag = 0;

	// get the right flags
	switch(mailbox) {
	case 0:
		reqAckFlag = CAN_TSR_RQCP0;
		txOkFlag = CAN_TSR_TXOK0;
		mailboxEmptyFlag = CAN_TSR_TME0;
		break;
	case 1:
		reqAckFlag = CAN_TSR_RQCP1;
		txOkFlag = CAN_TSR_TXOK1;
		mailboxEmptyFlag = CAN_TSR_TME1;
		break;
	case 2:
		reqAckFlag = CAN_TSR_RQCP2;
		txOkFlag = CAN_TSR_TXOK2;
		mailboxEmptyFlag = CAN_TSR_TME2;
		break;
	}

	// make sure this mailbox is empty
	if(!(CAN->TSR & mailboxEmptyFlag)) {
		LOG("mailbox not empty: %u\n", mailbox);
		return kErrCanNoFreeMailbox;
	}

	// set the address into the mailbox; use identifier extension, data frame
	uint32_t TIR = 0x04;
	TIR |= (msg->identifier & 0x1FFFFFFFUL) << 3;

	if(msg->rtr) {
		TIR |= 0x02;
	}

	CAN->sTxMailBox[mailbox].TIR = TIR;

	// set data length
	CAN->sTxMailBox[mailbox].TDTR = msg->length;

	// copy low byte of data
	uint32_t data;

	data = (uint32_t) ((msg->data[3] << 24) | (msg->data[2] << 16) |
			(msg->data[1] << 8) | (msg->data[0] << 0));
	CAN->sTxMailBox[mailbox].TDLR = data;

	data = (uint32_t) ((msg->data[7] << 24) | (msg->data[6] << 16) |
			(msg->data[5] << 8) | (msg->data[4] << 0));
	CAN->sTxMailBox[mailbox].TDHR = data;

	// request transmission of mailbox, and wait for request acknowledgement
	CAN->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;
//	while(!(CAN->TSR & reqAckFlag)) {}

	// wait for the message to have been transmitted successfully
	// notification bits: 0x01000000 / 0x02000000 / 0x04000000
//	while(!(CAN->TSR & txOkFlag)) {}
	return kErrSuccess;
}



/**
 * CAN IRQ handler
 */
void CEC_CAN_IRQHandler(void) {
	BaseType_t higherPriorityWoken = pdFALSE, ok;

	// was this a CAN IRQ?
//	if(SYSCFG->IT_LINE_SR[30] & SYSCFG_ITLINE30_SR_CAN) {
	if(1) {
		uint32_t masterIrq = CAN->MSR;

		// is this an error interrupt?
		if(masterIrq & CAN_MSR_ERRI) {
			// acknowledge error interrupt
			CAN->MSR &= (uint32_t) ~CAN_MSR_ERRI;

			// send a message to the handler task
			can_task_msg_t msg;
			msg.type = kCANTaskHandleError;
			msg.canError = CAN->ESR;

			ok = xQueueSendToFrontFromISR(gCANState.taskMsgQueue, &msg, &higherPriorityWoken);
		}
		// did a TX mailbox free up? (check if at least one mailbox is free)
		masterIrq = CAN->TSR;

		if((masterIrq & (CAN_TSR_TME2 | CAN_TSR_TME1 | CAN_TSR_TME0)) != (CAN_TSR_TME2 | CAN_TSR_TME1 | CAN_TSR_TME0)) {
			// we don't care about which mailbox became empty, just notify waiting task
			// acknowledge the interrupt
			CAN->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;

			// notify the CAN message task
			ok = xTaskNotifyFromISR(gCANState.task, 0x80000000, eSetBits, &higherPriorityWoken);
		}

		// did a mailbox transmit successfully?
		if(masterIrq & CAN_TSR_TXOK0) {
			ok = xTaskNotifyFromISR(gCANState.task, 0x01000000, eSetBits, &higherPriorityWoken);
			CAN->TSR = CAN_TSR_RQCP0;
		} if(masterIrq & CAN_TSR_TXOK1) {
			ok = xTaskNotifyFromISR(gCANState.task, 0x02000000, eSetBits, &higherPriorityWoken);
			CAN->TSR = CAN_TSR_RQCP1;
		} if(masterIrq & CAN_TSR_TXOK2) {
			ok = xTaskNotifyFromISR(gCANState.task, 0x04000000, eSetBits, &higherPriorityWoken);
			CAN->TSR = CAN_TSR_RQCP2;
		}

		// do we have any pending messages in the FIFOs? repeat while we do
		uint32_t fifo0_pending = (CAN->RF0R & CAN_RF0R_FMP0);
		uint32_t fifo1_pending = (CAN->RF1R & CAN_RF1R_FMP1);

		while(fifo0_pending || fifo1_pending) {
			// is there any pending messages in FIFO 0?
			if(fifo0_pending) {
				higherPriorityWoken = can_isr_read_fifo(0) ? pdTRUE : pdFALSE;
			}
			// are there any pending messages in FIFO 1?
			if(fifo1_pending) {
				higherPriorityWoken = can_isr_read_fifo(1) ? pdTRUE : pdFALSE;
			}

			// check for overruns
			can_isr_check_fifo_overrun();

			// re-check fifo status
			fifo0_pending = (CAN->RF0R & CAN_RF0R_FMP0);
			fifo1_pending = (CAN->RF1R & CAN_RF1R_FMP1);
		}
	}

	// force a context switch if needed
    portYIELD_FROM_ISR(higherPriorityWoken);
}

/**
 * Reads a message from the specified FIFO.
 *
 * @note This function MAY be called from an ISR.
 *
 * @return Whether a higher priority task was woken.
 */
bool can_isr_read_fifo(int fifo) {
	BaseType_t ok;

	// get the identifier of the message
	can_message_t msg;
	memset(&msg, 0, sizeof(msg));

	msg.valid = 1;
	msg.rtr = (CAN->sFIFOMailBox[fifo].RIR & 0x02) ? 1 : 0;
	msg.identifier = (CAN->sFIFOMailBox[fifo].RIR & 0xFFFFFFF8) >> 3;

	// copy the data out of the register
	msg.length = CAN->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_DLC;

	uint32_t data;

	// read low word of data
	data = CAN->sFIFOMailBox[fifo].RDLR;

	msg.data[0] = (uint8_t) ((data & 0x000000FF) >> 0);
	msg.data[1] = (uint8_t) ((data & 0x0000FF00) >> 8);
	msg.data[2] = (uint8_t) ((data & 0x00FF0000) >> 16);
	msg.data[3] = (uint8_t) ((data & 0xFF000000) >> 24);

	// read high word of data
	data = CAN->sFIFOMailBox[fifo].RDHR;

	msg.data[4] = (uint8_t) ((data & 0x000000FF) >> 0);
	msg.data[5] = (uint8_t) ((data & 0x0000FF00) >> 8);
	msg.data[6] = (uint8_t) ((data & 0x00FF0000) >> 16);
	msg.data[7] = (uint8_t) ((data & 0xFF000000) >> 24);

	// release message from the FIFO
	if(fifo == 0) {
		CAN->RF0R |= CAN_RF0R_RFOM0;

		// wait for the mailbox to be released
		while(CAN->RF0R & CAN_RF0R_RFOM0) {}
	} else if(fifo == 1) {
		CAN->RF1R |= CAN_RF1R_RFOM1;

		// wait for the mailbox to be released
		while(CAN->RF1R & CAN_RF1R_RFOM1) {}
	}

	// send message to queue
	BaseType_t wokeHigherPriority;

	ok = xQueueSendFromISR(gCANState.rxQueue, &msg, &wokeHigherPriority);

	if(ok != pdTRUE) {
		LOG("xQueueSendFromISR: %d\n", ok);
	}

	return (wokeHigherPriority == pdTRUE);
}

/**
 * Checks whether the receive FIFOs were overrun.
 */
void can_isr_check_fifo_overrun(void) {
	// was FIFO0 overrun?
	if(CAN->RF0R & CAN_RF0R_FOVR0) {
		gCANState.numDroppedMessages++;

		CAN->RF0R &= (uint32_t) ~CAN_RF0R_FOVR0;
	}
	// Was FIFO1 overrun?
	if(CAN->RF1R & CAN_RF1R_FOVR1) {
		gCANState.numDroppedMessages++;

		CAN->RF1R &= (uint32_t) ~CAN_RF1R_FOVR1;
	}
}
