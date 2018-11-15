/*
 * i2c.c
 *
 * I2C1 is assigned as follows on both STM32F042 and STM32F072:
 * - SCL: PF1
 * - SDA: PF0
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
	// enable GPIO clock
	RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	// configure PF0 and PF1 as high speed alternate function outputs
	GPIOF->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1);
	GPIOF->OTYPER |= (GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1);
	GPIOF->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1);

	// configure PF0 and PF1 alternate function 1
	GPIOB->AFR[0] |= (0x01 << (0 * 4)) | (0x01 << (1 * 4));

	// enable I2C clock and reset
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
}

/**
 * Initializes the I2C peripheral.
 */
void i2c_init_peripheral(void) {

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
	I2C1->CR1 |= I2C_CR1_PE;
}



/**
 * Entry point for the I2C task.
 *
 * This task constantly waits on a notification, which is split into a bit
 * field indicating various actions:
 * - Bit 31: I2C error
 * - Bit 30: I2C NACK by master
 * - Bit 23: I2C byte received
 * - Bit 15: DMA error
 * - Bit 14: DMA complete
 */
__attribute__((noreturn)) void i2c_task(void *ctx __attribute__((unused))) {
	BaseType_t ok;
	uint32_t notification;

	while(1) {
		// wait for the task notification
		ok = xTaskNotifyWait(0, 0xC080C000, &notification, portMAX_DELAY);

		if(ok != pdTRUE) {
			LOG("xTaskNotifyWait: %d\n", ok);
			continue;
		}
	}
}



/**
 * I2C interrupt handler
 */
void I2C1_IRQHandler(void) {
	uint32_t bits = 0;
	BaseType_t ok, higherPriorityWoken = pdFALSE;


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
		bits |= 0x00008000;
		DMA1->IFCR |= DMA_IFCR_CTEIF2;
	}

	// did channel 2 complete?
	if(isr & DMA_ISR_TCIF2) {
		bits |= 0x00004000;
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
