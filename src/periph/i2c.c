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
int i2c_init(i2c_register_t *regs, uint8_t numRegs) {
	// clear state and copy callbacks
	memset(&gState, 0, sizeof(gState));

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
	// enable DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// disable I2C1_TX DMA request remap; it will be DMA1, Ch2
	SYSCFG->CFGR1 &= ~SYSCFG_CFGR1_I2C1_DMA_RMP;

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
	 * - Enable error interrupts
	 * - Enable STOP interrupts
	 * - Enable address match interrupts
	 * - Enable RX interrupt
	 * - Generate TX DMA request
	 *
	 * CR2:
	 * -
	 */
	I2C1->CR1 = (I2C_CR1_WUPEN | I2C_CR1_SBC| I2C_CR1_TXDMAEN |
				 I2C_CR1_ERRIE | I2C_CR1_STOPIE | I2C_CR1_ADDRIE |
				 I2C_CR1_RXIE /*| I2C_CR1_TXIE*/ | I2C_CR1_PE);

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

	// unmask DMA interrupts
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, kIRQPriorityI2CTxDMA);

	// enable I2C peripheral
//	I2C1->CR1 |= I2C_CR1_PE;
}


/**
 * Initialize slave byte control for just the register number.
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

		// handle bus errors
		if(notification & kNotificationI2CError) {
			LOG("I2C error %x\n", I2C1->ISR);
		}
		if(notification & kNotificationI2CBusError) {
			LOG("I2C bus error %x\n", I2C1->ISR);
		}


		// did we receive some data?
		if(notification & kNotificationRegDataRx) {
			LOG("I2C rx %u bytes (max %u)\n", gState.rxBufferSz, gState.rxBufferMax);

			i2c_register_t *reg = &gState.regs[gState.reg];
			if(reg->writeCb) {
				err = reg->writeCb((uint8_t) (gState.reg & 0xFF),
						&gState.rxBuffer, gState.rxBufferSz);

				if(err < kErrSuccess) {
					LOG("reg writeCb failed for 0x%02x: %d\n", gState.reg, err);
				}
			}
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

			// log
//			i2c_register_t *reg = &gState.regs[gState.reg];
//			LOG("reg 0x%02x at 0x%08x\n", gState.reg, reg);

			// clear state
			gState.txBuffer = NULL;
			gState.regSelected = false;

			// clear rx buffer
			gState.rxBufferMax = 0;
			gState.rxBufferSz = 0;

			memset(gState.rxBuffer, 0, sizeof(gState.rxBuffer));

			// clear tx buffer
			gState.txBuffer = NULL;
			gState.txBufferSz = 0;
			gState.txBufferTotal = 0;

			// reset slave byte control
			i2c_sbc_init_regnum();
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
 * I2C interrupt handler
 */
void I2C1_IRQHandler(void) {
	uint32_t bits = 0;
	BaseType_t ok, higherPriorityWoken = pdFALSE;

	// check the interrupt
	uint32_t isr = I2C1->ISR;

	// did we see our address?
	if(isr & I2C_ISR_ADDR) {
		I2C1->ICR = I2C_ICR_ADDRCF;

		bool read = (isr & I2C_ISR_DIR) ? true : false;
		LOG("direction: %s\n", read ? "read" : "write");

		// if reading, prepare TX IRQ
		if(isr & I2C_ISR_DIR) {
			i2c_irq_init_tx_dma();
		}
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

			// if rx buffer is full, notify task and NACK
			if(gState.rxBufferMax &&
					(gState.rxBufferSz == gState.rxBufferMax)) {
				bits |= kNotificationRegDataRx;

				I2C1->CR2 |= I2C_CR2_NACK;
			}
		}
		// otherwise, the byte is the register number
		else {
			// if the reg number is out of bounds, NACK
			if(data >= gState.numRegs) {
				I2C1->CR2 |= I2C_CR2_NACK;

				LOG("I2C: invalid reg 0x%02x\n", data);
			}
			// if so, set up the data
			else {
				gState.reg = data;
				gState.regSelected = true;

				i2c_register_t *reg = &(gState.regs[gState.reg]);

				// set up the tx buffer
				gState.txBuffer = reg->regReadBuffer;
				gState.txBufferTotal = 0;
				gState.txBufferSz = reg->readSize;

				// set up the rx buffer
				gState.rxBufferMax = reg->writeSize;
				gState.rxBufferSz = 0;

				/*LOG("I2C reg 0x%02x: read from %08x, write max %d\n", data,
						gState.txBuffer, gState.rxBufferMax);*/
			}
		}

		// reload NBYTES to ack transaction
		I2C1->CR2 |= ((1 & 0xFF) << 16);
	}
	// is the tx register is empty?
	/*if(isr & I2C_ISR_TXE) {
		i2c_irq_tx();
	}*/
	if(isr & I2C_ISR_TCR) {
		LOG_PUTS("I2C TCR");

		// reload NBYTES to ack transaction
		I2C1->CR2 |= ((1 & 0xFF) << 16);
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
 * When new data needs to be transmitted (an I2C read txn is occurring), this
 * does that.
 */
void i2c_irq_tx(void) {
	// copy data from TX buffer if we have it
	if(gState.txBuffer) {
		// output data if we have some
		if(gState.txBufferTotal < gState.txBufferSz) {
			uint8_t data = gState.txBuffer[gState.txBufferTotal];
			I2C1->TXDR = data;

			LOG("tx off %d = %02x (0x%08x)\n", gState.txBufferTotal, data, gState.txBuffer);

			// increment counter
			gState.txBufferTotal++;

			return;
		}
	}

	// data wasn't available for whatever reason. send garbage and NACK
	LOG("tx done (%d)\n", gState.txBufferTotal);

	I2C1->TXDR = 0x00;
	I2C1->CR2 |= I2C_CR2_NACK;
}

/**
 * Sets up the TX DMA request.
 *
 * We use DMA0 channel 2, I2C1_TX.
 */
void i2c_irq_init_tx_dma(void) {
	LOG("set up DMA size %d from 0x%08x\n", gState.txBufferSz, gState.txBuffer);

	// configure NBYTES
	uint32_t cr2 = I2C1->CR2;
	cr2 &= ~(I2C_CR2_NBYTES);
	cr2 |= (gState.txBufferSz & 0xFF) << 16;

//	cr2 &= ~(I2C_CR2_RELOAD);

	I2C1->CR2 = cr2;

	// set up the source/destination (memory -> peripheral)
	DMA1_Channel2->CPAR = (uint32_t) &I2C1->TXDR;
	DMA1_Channel2->CMAR = (uint32_t) gState.txBuffer;
	DMA1_Channel2->CNDTR = gState.txBufferSz;

	/**
	 * - Memory -> Peripheral
	 * - Read a single byte from memory
	 * - Write a single byte to peripheral
	 * - Increment memory address only
	 * - Issue transfer error and transfer complete interrupts
	 */
	DMA1_Channel2->CCR = DMA_CCR_DIR | DMA_CCR_MINC |
			DMA_CCR_TEIE | DMA_CCR_TCIE;

	// now, enable the DMA
	DMA1_Channel2->CCR |= DMA_CCR_EN;
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
