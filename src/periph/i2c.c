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
 * Data is received bytewise via interrupts and shoveled into the I2C task.
 *
 * Data is transmitted via DMA, which terminates either when all data has been
 * sent, or when parts of it have been sent but the master NACK'd early. In
 * both cases, the application's transmit call waits until the DMA finishes and
 * returns the number of bytes actually transferred.
 *
 * DMA channel 2 is used, I2C1_TX.
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
int i2c_init(const i2c_callbacks_t *callbacks) {
	// clear state and copy callbacks
	memset(&gState, 0, sizeof(gState));
	memcpy(&gState.cb, callbacks, sizeof(i2c_callbacks_t));

	// set up GPIOs and I2C
	i2c_init_gpio();
	i2c_init_peripheral();

	// set up DMA
	i2c_init_dma();

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

	// 400kHz clock, with 48MHz I2CCLK; 100ns rise, 10ns fall
//	I2C1->TIMINGR = 0x00900000;
	// 400kHz clock, with 48MHz I2CCLK; 140ns rise, 40ns fall
	I2C1->TIMINGR = 0x00B00000;

	/*
	 * Configure I2C peripheral:
	 *
	 * CR1:
	 * - Enable wakeup from STOP mode
	 * - Enable slave byte control
	 * - Enable TX DMA requests
	 * - Enable error interrupts
	 * - Enable STOP interrupts
	 * - Enable address match interrupts
	 * - Enable RX interrupt
	 *
	 * CR2:
	 * -
	 */
	I2C1->CR1 = (I2C_CR1_WUPEN |/*I2C_CR1_SBC|*//* I2C_CR1_TXDMAEN |*/
				 I2C_CR1_ERRIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE |
				 I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_PE);

	// Set I2C address
	I2C1->OAR1 |= (0x69 << 1);
	I2C1->OAR1 |= I2C_OAR1_OA1EN;
}

/**
 * Initializes the DMA channel for I2C transmission.
 */
void i2c_init_dma(void) {
	// enable DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
}

/**
 * Performs final initialization of the I2C peripheral before attaching to the
 * bus: this also finally enables I2C interrupts.
 */
void i2c_init_begin(void) {
	// unmask the interrupt
	NVIC_EnableIRQ(I2C1_IRQn);
	NVIC_SetPriority(I2C1_IRQn, kIRQPriorityI2C);

	// enable I2C peripheral
//	I2C1->CR1 |= I2C_CR1_PE;
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

		// handle bus errors
		if(notification & kNotificationI2CError) {
			LOG("I2C error %x\n", I2C1->ISR);
		}
		if(notification & kNotificationI2CBusError) {
			LOG("I2C bus error %x\n", I2C1->ISR);
		}

		// was a register number latched in?
		if(notification & kNotificationRegLatched) {
			LOG("I2C reg latched: (%d) 0x%02x\n", gState.regSelected, gState.reg);
		}
		// does this transaction match our address?
		if(notification & kNotificationSlaveSelect) {
			uint8_t reg = (uint8_t) gState.reg;

			// has the register been latched yet?
			if(gState.regSelected) {
				// is this a read?
				if(I2C1->ISR & I2C_ISR_DIR) {
					// call the read function
					err = gState.cb.reg_read(reg);

					if(err < kErrSuccess) {
						LOG("reg_read failed: %d\n", err);
					}
				} else {
					// get the number of bytes to write, max
					size_t maxWrite;
					err = gState.cb.reg_write_max(reg, &maxWrite);

					if(err < kErrSuccess) {
						LOG("reg_write_max failed: %d\n", err);
					}
					// callback was successful, set up the read buffer
					else {
						gState.rxBufferMax = maxWrite;
						gState.rxBufferSz = 0;

						memset(gState.rxBuffer, 0, sizeof(gState.rxBuffer));
					}
				}

				// if an error occurred at any point, NACK the transaction
				if(err < kErrSuccess) {
					I2C1->CR2 |= I2C_CR2_NACK;
				}
			}
		}

		// did we receive some data?
		if(notification & kNotificationRegDataRx) {
			LOG("I2C rx %u bytes (max %u)\n", gState.rxBufferSz, gState.rxBufferMax);
		}

		// did we receive a NACK? (end of read/write)
		if(notification & kNotificationI2CNACK) {
			LOG_PUTS("I2C NACK");
		}

		// did the transaction stop?
		if(notification & kNotificationI2CSTOP) {
			LOG("I2C STOP; rx'd %u bytes (max %u), tx'd %u\n\n",
					gState.rxBufferSz, gState.rxBufferMax,
					gState.txBufferTotal);

			// clear state
			gState.txBuffer = NULL;
			gState.regSelected = false;
		}

		// handle DMA state changes
		if(notification & kNotificationDMAError) {
			LOG("DMA error: %x\n", DMA1->ISR);
		}
		if(notification & kNotificationDMAComplete) {
			LOG_PUTS("DMA complete");
		}
	}
}



/**
 * Transmits the given bytes over I2C.
 *
 * @return the number of bytes that were actually written, or a negative error
 * code.
 */
int i2c_write(void *data, size_t length) {
	LOG("TX %u bytes (0x%x)\n", length, data);

	// copy the values
	gState.txBuffer = data;
	gState.txBufferSz = length;
	gState.txBufferTotal = 0;

	// TODO: wait for tx

	// return number of bytes transmitted
	return (gState.txBufferTotal & 0xFF);
}


/**
 * I2C interrupt handler
 */
void I2C1_IRQHandler(void) {
	uint32_t bits = 0;
	BaseType_t ok, higherPriorityWoken = pdFALSE;

	// check the interrupt
	uint32_t isr = I2C1->ISR;

	// did we see our address?
	if(isr & I2C_ISR_ADDR) {
		bits |= kNotificationSlaveSelect;
		I2C1->ICR = I2C_ICR_ADDRCF;
	}
	// was there a bus error?
	if(isr & I2C_ISR_BERR) {
		bits |= kNotificationI2CBusError;
		I2C1->ICR = I2C_ICR_BERRCF;
	}
	// did a STOP condition occur?
	if(isr & I2C_ISR_STOPF) {
		bits |= kNotificationI2CSTOP;
		I2C1->ICR = I2C_ICR_STOPCF;
	}
	// did we receive a NACK?
	if(isr & I2C_ISR_NACKF) {
		bits |= kNotificationI2CNACK;
		I2C1->ICR = I2C_ICR_NACKCF;
	}
	// did we receive a byte?
	if(isr & I2C_ISR_RXNE) {
		uint8_t data = (I2C1->RXDR & 0xFF);

		// if register was selected, read data into it
		if(gState.regSelected) {
			if(gState.rxBufferSz > gState.rxBufferMax) {
				gState.rxBuffer[gState.rxBufferSz++] = data;
			}

			// if full, notify task
			if(gState.rxBufferMax &&
					(gState.rxBufferSz == gState.rxBufferMax)) {
				bits |= kNotificationRegDataRx;
			}
		}
		// otherwise, the byte is the register number
		else {
			gState.reg = data;
			gState.regSelected = true;

			bits |= kNotificationRegLatched;
		}
	}
	// is the tx register is empty?
	if(isr & I2C_ISR_TXIS) {
		// copy data from TX buffer if we have it
		if(gState.txBuffer) {
			I2C1->TXDR = gState.txBuffer[gState.txBufferTotal];
		} else {
			// NACK immediately since there was an underflow
//			I2C1->CR2 |= I2C_CR2_NACK;

			// prepare garbage data
			I2C1->TXDR = 0x00;
		}

		// increment counter
		gState.txBufferTotal++;
	}


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
 * DMA channels 2 and 3 interrupt handler: we only use channel 2 for I2C1_TX.
 */
void DMA1_Channel2_3_IRQHandler(void) {
	BaseType_t ok, higherPriorityWoken = pdFALSE;

	uint32_t bits = 0, isr = DMA1->ISR;

	// did channel 2 error?
	if(isr & DMA_ISR_TEIF2) {
		bits |= kNotificationDMAError;
		DMA1->IFCR |= DMA_IFCR_CTEIF2;
	}

	// did channel 2 complete?
	if(isr & DMA_ISR_TCIF2) {
		bits |= kNotificationDMAComplete;
		DMA1->IFCR |= DMA_IFCR_CTCIF2;
	}

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
