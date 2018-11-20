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
 * We run the bus at 400kHz.
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
static i2c_state_t gState;



/**
 * Initializes the I2C driver.
 */
int i2c_init(const i2c_callbacks_t *callbacks, i2c_register_t *regs, uint8_t numRegs) {
	// clear state and copy callbacks
	memset(&gState, 0, sizeof(gState));
	memcpy(&gState.cb, callbacks, sizeof(i2c_callbacks_t));

	LOG("I2C: initializing with %d regs at 0x%08x\n", numRegs, regs);

	gState.regs = regs;
	gState.numRegs = numRegs;

	// set up GPIOs and I2C
	i2c_init_gpio();
	i2c_init_peripheral();

	// set up the I2C task
	gState.task = xTaskCreateStatic(i2c_task, "I2C",
			kI2CStackSize, NULL, 1, (void *) &gState.taskStack,
			&gState.taskTCB);

	if(gState.task == NULL) {
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

	// enable Fm+ drive conttrol on GPIOs
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
	 * - Enable slave byte control
	 * - Enable error interrupts
	 * - Enable STOP interrupts
	 * - Enable address match interrupts
	 * - Enable RX interrupt
	 * - Enable TX interrupt
	 *
	 * CR2:
	 * -
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

	// slave byte control
	i2c_sbc_init_regnum();
}

/**
 * Performs final initialization of the I2C peripheral before attaching to the
 * bus: this also finally enables I2C interrupts.
 */
void i2c_init_begin(void) {
	// unmask the I2C interrupts
	NVIC_EnableIRQ(I2C1_IRQn);
	NVIC_SetPriority(I2C1_IRQn, kIRQPriorityI2C);

	// enable I2C peripheral
//	I2C1->CR1 |= I2C_CR1_PE;
}


/**
 * Initialize slave byte control for a single byte with manual ACK.
 */
void i2c_sbc_init_regnum(void) {
	uint32_t cr2 = I2C1->CR2;

	// read a single byte
	cr2 &= ~(I2C_CR2_NBYTES);
	cr2 |= (1 & 0xFF) << 16;

	// reload bit is set; we ack each byte by software
	cr2 |= I2C_CR2_RELOAD;

	I2C1->CR2 = cr2;
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
	uint32_t notification;

	LOG_PUTS("I2C ready");

	while(1) {
		// wait for the task notification
		ok = xTaskNotifyWait(0, kNotificationAny, &notification, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG("xTaskNotifyWait: %d\n", ok);
			continue;
		}

//		LOG("I2C notification: %08x\n", notification);
	}
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
	uint32_t bits = 0;
	BaseType_t ok, higherPriorityWoken = pdFALSE;

	// check the interrupt
	uint32_t isr = I2C1->ISR;

	// did we see our address?
	if(isr & I2C_ISR_ADDR) {
//		isr &= ~I2C_ISR_ADDR;

		I2C1->ICR = I2C_ICR_ADDRCF;

		bool read = (isr & I2C_ISR_DIR) ? true : false;
		LOG("direction: %s\n", read ? "read" : "write");

		// was a register latched?
		if(!gState.regLatched) {
			// if so, and this is a read, act as if reg 0 is being read
			if(isr & I2C_ISR_DIR) {
				gState.reg = 0x00;
				gState.regLatched = true;

				// transmit a single byte of data
				i2c_irq_tx();
			}
			// it's a write for the register number, do nothing here
			else { /* nothing */ }
		}
		// yes: we should only go to a read now and transmit the register
		else {
			// is this a read (we transmit)?
			if(isr & I2C_ISR_DIR) {
				// transmit a single byte of data
				i2c_irq_tx();
			}
			// it's a write… this shouldn't happen
			else { /* nothing */ }
		}

		// load NBYTES initially
		i2c_irq_sbc_reload();
	}
	// was there a bus error?
	if(isr & I2C_ISR_BERR) {
//		isr &= ~I2C_ISR_BERR;
		I2C1->ICR = I2C_ICR_BERRCF;

		// TODO: handle
	}
	// did a STOP condition occur?
	if(isr & I2C_ISR_STOPF) {
//		isr &= ~I2C_ISR_STOPF;
		I2C1->ICR = I2C_ICR_STOPCF;

		// TODO: call read callback if readCounter != 0
		// TODO: call write callback if writeCounter != 0

		// reset register state
		gState.regLatched = false;
		gState.reg = 0xFFFF;

		// reset the read (I2C tx) counters
		gState.readCounter = 0;

		// reset the write (I2C rx) counters
		gState.writeCounter = 0;
	}
	// did we receive a NACK?
	if(isr & I2C_ISR_NACKF) {
//		isr &= ~I2C_ISR_NACKF;
		I2C1->ICR = I2C_ICR_NACKCF;

		// TODO: handle
	}
	// did we receive a byte?
	if(isr & I2C_ISR_RXNE) {
//		isr &= ~I2C_ISR_RXNE;

		// read data (this clears the interrupt)
		uint8_t data = (I2C1->RXDR & 0xFF);

		// if no register was latched yet, this is the register number
		if(!gState.regLatched) {
			// make sure we have this register defined
			if(data >= gState.numRegs) {
				// we do not, so NACK the transmission
				LOG("I2C: invalid reg 0x%02x\n", data);
				i2c_irq_nack();
			}
			// otherwise, set up for a write to this register
			// this may change later if we get a RESTART
			else {
				gState.reg = data;
				gState.regLatched = true;
			}
		}
		// a register was latched, this data should be written to the register
		else {
			// registers will just loop after 4 bytes
			size_t byteOff = (gState.writeCounter & 0x03);

			// get a byte pointer into the register's read value and send it
			uint8_t *writePtr = (uint8_t *) &(gState.regs[gState.reg].write);
			writePtr[byteOff] = data;

			// increment counter
			gState.writeCounter++;
		}

		// reload NBYTES to ack transaction
		i2c_irq_sbc_reload();
	}
	// is the tx register is empty?
	if(isr & I2C_ISR_TXE) {
		// if so, transmit one byte
		i2c_irq_tx();

		// reload NBYTES to ack the transaction
		i2c_irq_sbc_reload();
	}
	/*if(isr & I2C_ISR_TCR) {
		LOG_PUTS("I2C TCR");

		// reload NBYTES to ack transaction
		I2C1->CR2 |= ((1 & 0xFF) << 16);
	}*/

//	LOG("ISR 0x%08x\n", isr);


	// send notification if needed
	if(bits) {
		ok = xTaskNotifyFromISR(gState.task, bits, eSetBits,
				&higherPriorityWoken);

		// we don't use the result of the call, can't do much anyways
		(void) ok;
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
 * Reloads the NBYTES counter for slave byte control.
 *
 * This is basically identical to i2c_sbc_init_regnum but for use in the IRQ
 */
static void i2c_irq_sbc_reload(void) {
	uint32_t cr2 = I2C1->CR2;

	// read a single byte
	cr2 &= ~(I2C_CR2_NBYTES);
	cr2 |= (1 & 0xFF) << 16;

	// reload bit is set; we ack each byte by software
	cr2 |= I2C_CR2_RELOAD;

	I2C1->CR2 = cr2;
}

/**
 * When new data needs to be transmitted (an I2C read txn is occurring), this
 * does that.
 */
static void i2c_irq_tx(void) {
	// was a register latched?
	if(gState.regLatched) {
		// registers will just loop after 4 bytes
		size_t byteOff = (gState.readCounter & 0x03);

		// get a byte pointer into the register's read value and send it
		uint8_t *readPtr = (uint8_t *) &(gState.regs[gState.reg].read);
		I2C1->TXDR = readPtr[byteOff];

		// increment counter
		gState.readCounter++;
	}
	// if not, output 0xFF
	else {
		I2C1->TXDR = 0xFF;
	}
}
