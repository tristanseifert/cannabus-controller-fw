/*
 * i2c.c
 *
 * I2C1 is assigned as follows on both STM32F042:
 * - SCL: PF1 (AF1)
 * - SDA: PF0 (AF1)
 *
 * On STM32F072:
 * - SCL: PB8 (AF1)
 * - SDA: PB9 (AF1)
 *
 * NOTES:
 * - For some reason, the first byte of a read is garbage data. So, to read a
 * 	 full register, read 5 bytes and ignore the first one.
 *
 *  Created on: Nov 14, 2018
 *      Author: tristan
 */
#include "i2c.h"
#include "i2c_private.h"

#include "controller.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/// I2C driver state
static i2c_state_t gI2CState;



/**
 * Initializes the I2C driver.
 */
int i2c_init(const i2c_callbacks_t *callbacks, i2c_register_t *regs, uint8_t numRegs) {
	// clear state and copy callbacks
	memset(&gI2CState, 0, sizeof(gI2CState));
	memcpy(&gI2CState.cb, callbacks, sizeof(i2c_callbacks_t));

	LOG("I2C: initializing with %d regs at 0x%08x\n", numRegs, regs);

	gI2CState.regs = regs;
	gI2CState.numRegs = numRegs;

	// set up GPIOs and I2C
	i2c_init_gpio();
	i2c_init_peripheral();

	// set up task message queue
	gI2CState.msgQueue = xQueueCreateStatic(kTaskMsgQueueSize,
			sizeof(i2c_task_msg_t), (void *) &gI2CState.msgQueueBuffer,
			&gI2CState.msgQueueStruct);

	if(gI2CState.msgQueue == NULL) {
		return kErrQueueCreationFailed;
	}

	// set up the I2C task
	gI2CState.task = xTaskCreateStatic(i2c_task, "I2C",
			kI2CStackSize, NULL, kTaskPriorityI2C,
			(void *) &gI2CState.taskStack, &gI2CState.taskTCB);

	if(gI2CState.task == NULL) {
		return kErrTaskCreationFailed;
	}

	// now that everything is ready, begin I2C
	i2c_init_begin();

	// success
	return kErrSuccess;
}

/**
 * Initializes the GPIOs used for I2C.
 */
void i2c_init_gpio(void) {
#ifdef STM32F042
	// enable GPIO clock
	RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	// configure PF0 and PF1 as high speed alternate function outputs
	GPIOF->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
	GPIOF->OTYPER |= (GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1);
	GPIOF->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);

	// configure PF0 and PF1 alternate function 1
	GPIOF->AFR[0] |= (0x01 << (0 * 4)) | (0x01 << (1 * 4));
#endif
#ifdef STM32F072
	// enable GPIO clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// configure PB8 and PB9 as high speed alternate function outputs
	GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);

	// configure PB8 and PB9 alternate function 1
	GPIOB->AFR[1] |= (0x01 << (0 * 4)) | (0x01 << (1 * 4));
#endif

	// enable Fm+ drive control on GPIOs
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C_FMP_I2C1;
}

/**
 * Initializes the I2C peripheral.
 */
void i2c_init_peripheral(void) {
	// enable I2C clock and reset
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

	// this seems to be ignored in slave mode
	// 400kHz clock, with 48MHz I2CCLK; 140ns rise, 40ns fall
	// I2C1->TIMINGR = 0x00B00000;

	// Disable I2C
	I2C1->CR1 &= ~I2C_CR1_PE;

	/*
	 * Configure I2C peripheral:
	 *
	 * CR1:
	 * - Enable wakeup from STOP mode
	 * - Enable error interrupts
	 * - Enable STOP and NACK interrupts
	 * - Enable address match interrupts
	 * - Enable RX interrupt
	 * - Enable TX interrupt
	 */
	I2C1->CR1 = (I2C_CR1_WUPEN | /*I2C_CR1_SBC |*/
				 I2C_CR1_ERRIE |
				 I2C_CR1_NACKIE | I2C_CR1_STOPIE |
				 I2C_CR1_ADDRIE |
				 I2C_CR1_RXIE | I2C_CR1_TXIE |
				 I2C_CR1_PE);

	// Set I2C address
	I2C1->OAR1 |= (0x69 << 1);
	I2C1->OAR1 |= I2C_OAR1_OA1EN;
}

/**
 * Performs final initialization of the I2C peripheral before attaching to the
 * bus: this also finally enables I2C interrupts.
 */
void i2c_init_begin(void) {
	// unmask the I2C interrupts
	NVIC_EnableIRQ(I2C1_IRQn);
	NVIC_SetPriority(I2C1_IRQn, kIRQPriorityI2C);
}



/**
 * Entry point for the I2C task.
 *
 * This task constantly waits on a notification, which is split into a bit
 * field indicating various actions.
 */
__attribute__((noreturn)) void i2c_task(void *ctx __attribute__((unused))) {
	int err;
	BaseType_t ok;
	i2c_task_msg_t msg;

	LOG_PUTS("I2C: ready");

	while(1) {
		// dequeue a message
		ok = xQueueReceive(gI2CState.msgQueue, &msg, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG("xQueueReceive: %d\n", ok);
			continue;
		}

		// reset per-message state
		err = kErrSuccess;

		// handle message
		switch(msg.type) {
			// no-op
			case kTaskMsgTypeNop:
				break;

			// notification from ISR
			case kTaskMsgTypeNotification:
				err = i2c_task_notification(&msg);
				break;

			// unknown
			default:
				LOG("I2C: unknown message type %d\n", msg.type);
				break;
		}

		// did an error occur?
		if(err < kErrSuccess) {
			LOG("I2C: error handling message type %d: %d\n", msg.type, err);
		}
	}
}

/**
 * Handles an ISR notification message.
 */
int i2c_task_notification(i2c_task_msg_t *msg) {
	int err = kErrSuccess;

	const uint32_t notification = msg->notification.type;

	// was a register read?
	if(notification & kNotificationRead) {
		const uint8_t reg = (msg->notification.data & 0xFF);

		err = gI2CState.cb.read(reg);

		if(err < kErrSuccess) {
			LOG("I2C: read callback failed %d\n", err);
		}
	}
	// was a register written?
	else if(notification & kNotificationWrite) {
		const uint8_t reg = (msg->notification.data & 0xFF);

		err = gI2CState.cb.written(reg);

		if(err < kErrSuccess) {
			LOG("I2C: write callback failed %d\n", err);
		}
	}
	// did a bus error occurr?
	else if(notification & kNotificationBusErr) {
		LOG("I2C: bus error, ISR = 0x%08x\n", msg->notification.data);
	}

	return err;
}




/**
 * I2C interrupt handler
 *
 * This implements a simple state machine:
 *
 * 1.	Wait to receive a single byte. This is the register number.
 * 2.	Pre-load the transmit register with the first byte of the register. Once
 * 		this has been done, wait for either further bytes to be received (3) or
 * 		for a restart (4).
 * 3.	Receive four more bytes and write them into the register structure. Go
 * 		to (5).
 * 4.	Transmit four bytes from the register struct. Go to (5).
 * 5.	Wait for STOP condition, reset internal state.
 */
void I2C1_IRQHandler(void) {
	int err = kErrSuccess;
	BaseType_t higherPriorityWoken = pdFALSE;

	// check the interrupt
	uint32_t isr = I2C1->ISR;

	// did we see our address?
	if(isr & I2C_ISR_ADDR) {
		I2C1->ICR = I2C_ICR_ADDRCF;

		// was a register latched?
		if(!gI2CState.regLatched) {
			// if not, and this is a read, act as if reg 0 is being read
			if(isr & I2C_ISR_DIR) {
				gI2CState.reg = 0x00;
				gI2CState.regLatched = true;
			}
			// it's a write for the register number, do nothing here
			else { /* nothing */ }
		}
		// yes: we should only go to a read now and transmit the register
		else {
			// is this a read (we transmit)?
			if(isr & I2C_ISR_DIR) {
				// transmit a single byte of data
//				i2c_irq_tx();
			}
			// it's a write… this shouldn't happen
			else { /* nothing */ }
		}
	}
	// was there a bus error?
	if(isr & I2C_ISR_BERR) {
		I2C1->ICR = I2C_ICR_BERRCF;

		// notify task
		err = i2c_irq_notify(kNotificationBusErr, isr, &higherPriorityWoken);
	}
	// did a STOP condition occur?
	if(isr & I2C_ISR_STOPF) {
		I2C1->ICR = I2C_ICR_STOPCF;

		// call read callback if readCounter != 0
		if(gI2CState.readCounter != 0) {
			uint32_t data = (gI2CState.reg & 0xFF) | ((gI2CState.readCounter & 0xFF) << 8);
			err = i2c_irq_notify(kNotificationRead, data, &higherPriorityWoken);

			// increment global counters
			gI2CState.totalNumReads++;
		}
		// call write callback if writeCounter != 0
		else if(gI2CState.writeCounter != 0) {
			uint32_t data = (gI2CState.reg & 0xFF) | ((gI2CState.writeCounter & 0xFF) << 8);
			err = i2c_irq_notify(kNotificationWrite, data, &higherPriorityWoken);

			// increment global counters
			gI2CState.totalNumWrites++;
		}

		// reset register state
		gI2CState.regLatched = false;
		gI2CState.reg = 0xFFFF;

		// reset the read (I2C tx) counters
		gI2CState.readCounter = 0;

		// reset the write (I2C rx) counters
		gI2CState.writeCounter = 0;
	}
	// did we receive a NACK?
	if(isr & I2C_ISR_NACKF) {
		I2C1->ICR = I2C_ICR_NACKCF;

		// TODO: handle (but we can probably just ignore this)
	}
	// did we receive a byte?
	if(isr & I2C_ISR_RXNE) {
		// read data (this clears the interrupt)
		uint8_t data = (I2C1->RXDR & 0xFF);

		// if no register was latched yet, this is the register number
		if(!gI2CState.regLatched) {
			// make sure we have this register defined
			if(data >= gI2CState.numRegs) {
				// we do not, so NACK the transmission
//				LOG("I2C: invalid reg 0x%02x\n", data);
				i2c_irq_nack();
			}
			// otherwise, set up for a write to this register
			// this may change later if we get a RESTART
			else {
				gI2CState.reg = data;
				gI2CState.regLatched = true;

				// reset byte counters
				gI2CState.readCounter = 0;
				gI2CState.writeCounter = 0;
			}
		}
		// a register was latched, this data should be written to the register
		else {
			// registers will just loop after 4 bytes
			size_t byteOff = (gI2CState.writeCounter & 0x03);

			// get a byte pointer into the register's read value and send it
			uint8_t *writePtr = (uint8_t *) &(gI2CState.regs[gI2CState.reg].write);
			writePtr[byteOff] = data;

			// increment counter
			gI2CState.writeCounter++;
		}
	}
	// is the tx register is empty?
	if(isr & I2C_ISR_TXE) {
		// if so, transmit one byte
		i2c_irq_tx();
	}

	// break into debugger if there was an error
	if(err < kErrSuccess) {
		asm volatile("bkpt 0");
	}

	// switch tasks if needed
	portYIELD_FROM_ISR(higherPriorityWoken);
}

/**
 * Sends a NACK.
 */
static inline void i2c_irq_nack(void) {
	I2C1->CR2 |= I2C_CR2_NACK;
}

/**
 * When new data needs to be transmitted (an I2C read txn is occurring), this
 * does that.
 */
static void i2c_irq_tx(void) {
	// was a register latched?
	if(gI2CState.regLatched) {
		// registers will just loop after 4 bytes
		size_t byteOff = (gI2CState.readCounter & 0x03);

		// get a byte pointer into the register's read value and send it
		uint8_t *readPtr = (uint8_t *) &(gI2CState.regs[gI2CState.reg].read);
		I2C1->TXDR = readPtr[byteOff];

		// increment counter
		gI2CState.readCounter++;
	}
	// if not, output 0xFF
	else {
		I2C1->TXDR = 0xFF;
	}
}

/**
 * Sends a notification to the I2C task.
 */
static int i2c_irq_notify(uint32_t notification, uint32_t data,
		BaseType_t *higherPriorityWoken) {
	BaseType_t ok;

	// construct message
	i2c_task_msg_t msg;

	msg.type = kTaskMsgTypeNotification;

	msg.notification.type = notification;
	msg.notification.data = data;

	// now, send the message
	ok = xQueueSendToBackFromISR(gI2CState.msgQueue, &msg, higherPriorityWoken);

	return (ok == pdTRUE) ? kErrSuccess : kErrQueueSend;
}
