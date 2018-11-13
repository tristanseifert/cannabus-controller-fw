#include "cmsis_device.h"
#include "diag/Trace.h"

#include "controller.h"

#define USE_STM32_LIB				0

// system core clock
//extern uint32_t SystemCoreClock;

/**
 * Initialize UART2 with TX on PA2. Output is 8N1 115200.
 */
void trace_initialize(void) {
#if USE_STM32_LIB
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

	//Configure USART2 pins:  Rx and Tx ----------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);
#else
	#ifdef STM32F042
		// enable UART2 and GPIOA clock
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		// configure PA2 as alternate function, AF1
		GPIOA->MODER  |= 0x02 << (2 * 2);
		GPIOA->AFR[0] |= 0x01 << (2 * 4);
	#endif
	#ifdef STM32F072
		// enable UART2 and GPIOA clock
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		// configure PA2 as alternate function, AF1
		GPIOA->MODER  |= 0x02 << (2 * 2);
		GPIOA->AFR[0] |= 0x01 << (2 * 4);
	#endif

	// reset UART2
	RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
	for(volatile int i = 0; i < 32; i++) {}
	RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

	// disable UART
	USART2->CR1 &= (uint32_t) ~USART_CR1_UE;

	// set baud rate: 115200 (USART2 clock is SYSCLK)
	RCC->CFGR3 |= (RCC_CFGR3_USART2SW_0) & RCC_CFGR3_USART2SW;

	USART2->BRR = 0x1A1; // with OVER8 = 0

	// TX enable, oversample 16x, even parity,
	USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
#endif
}


/**
 * Writes the given string to UART2.
 */
ssize_t trace_write(const char* buf, size_t nbyte) {
	// write bytes to the UART until done
	for(size_t i = 0; i < nbyte; i++) {
#if USE_STM32_LIB
	   while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);

	   USART_SendData(USART2, (uint16_t) *buf++);
#else
		// is the UART ready to accept more data?
		while(!(USART2->ISR & USART_ISR_TXE)) {}

		// write the byte of data
		USART2->TDR = (uint16_t) *buf++;
#endif
	}

	// return number of bytes written
	return (ssize_t) nbyte;
}
