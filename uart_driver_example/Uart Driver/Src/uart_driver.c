/*
 *	uart_driver.c - Uart library for STM32F411xC/xE ARM microcontrollers
 *
 *	Created on: Aug 13, 2024
 *      Author: Emircan Hızarcı
 */

#include <string.h>

#include "uart_driver.h"

/* Define buffers of UARTS */
#ifdef UART1
static uint8_t uart1_BufferTX[UART1_BUFFER_SIZE_TX] = {0U};
static ring_buffer_t UART1_BUFFER_TX = {.buffer = uart1_BufferTX,.mask = sizeof(uart1_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};

static uint8_t uart1_BufferRX[UART1_BUFFER_SIZE_RX] = {0U};
static ring_buffer_t UART1_BUFFER_RX = {.buffer = uart1_BufferRX,.mask = sizeof(uart1_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};
#endif

#ifdef UART2
static uint8_t uart2_BufferTX[UART2_BUFFER_SIZE_TX] = {0U};
static ring_buffer_t UART2_BUFFER_TX = {.buffer = uart2_BufferTX,.mask = sizeof(uart2_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};

static uint8_t uart2_BufferRX[UART2_BUFFER_SIZE_RX] = {0U};
static ring_buffer_t UART2_BUFFER_RX = {.buffer = uart2_BufferRX,.mask = sizeof(uart2_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};
#endif

#ifdef UART3
static uint8_t uart3_BufferTX[UART3_BUFFER_SIZE_TX] = {0U};
static ring_buffer_t UART3_BUFFER_TX = {.buffer = uart3_BufferTX,.mask = sizeof(uart3_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};

static uint8_t uart3_BufferRX[UART3_BUFFER_SIZE_RX] = {0U};
static ring_buffer_t UART3_BUFFER_RX = {.buffer = uart3_BufferRX,.mask = sizeof(uart3_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};
#endif

#ifdef UART4
static uint8_t uart4_BufferTX[UART4_BUFFER_SIZE_TX] = {0U};
static ring_buffer_t UART4_BUFFER_TX = {.buffer = uart4_BufferTX,.mask = sizeof(uart4_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};

static uint8_t uart4_BufferRX[UART4_BUFFER_SIZE_RX] = {0U};
static ring_buffer_t UART4_BUFFER_RX = {.buffer = uart4_BufferRX,.mask = sizeof(uart4_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};
#endif

#ifdef UART5
static uint8_t uart5_BufferTX[UART5_BUFFER_SIZE_TX] = {0U};
static ring_buffer_t UART5_BUFFER_TX = {.buffer = uart5_BufferTX,.mask = sizeof(uart5_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};

static uint8_t uart5_BufferRX[UART4_BUFFER_SIZE_RX] = {0U};
static ring_buffer_t UART5_BUFFER_RX = {.buffer = uart5_BufferRX,.mask = sizeof(uart5_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};
#endif

#ifdef UART6
static uint8_t uart6_BufferTX[UART6_BUFFER_SIZE_TX] = {0U};
static ring_buffer_t UART6_BUFFER_TX = {.buffer = uart6_BufferTX,.mask = sizeof(uart6_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};

static uint8_t uart6_BufferRX[UART6_BUFFER_SIZE_RX] = {0U};
static ring_buffer_t UART6_BUFFER_RX = {.buffer = uart6_BufferRX,.mask = sizeof(uart6_BufferTX) - 1, .head_pointer = 0, .tail_pointer = 0};
#endif

static uint32_t UART_String_TimeOut = STRING_TIMEOUT;
static uint32_t UART_ByteArray_TimeOut = BYTE_ARRAY_TIMEOUT;

/* Private Functions */
static uint16_t calc_USARTDIV(USART_TypeDef* USARTx, uint32_t baud);

static void UART_Config(UART_T* UART);

static void UART_GPIO_Init(UART_T* UARTx);
static ring_buffer_t* GET_UART_BUFFER_RX(USART_TypeDef* USARTx);
static ring_buffer_t* GET_UART_BUFFER_TX(USART_TypeDef* USARTx);

void UART_ADVANCE_Init(UART_T* UARTx)
{
	uint8_t subPriority = 0;
	IRQn_Type IRQn = 0;

#ifdef UART1
	if(UARTx->USARTx == USART1)
	{
		/* Enable USART1 clock */
		__HAL_RCC_USART1_CLK_ENABLE();

		subPriority = 0;
		IRQn = USART1_IRQn;
	}
#endif
#ifdef UART2
	if(UARTx->USARTx == USART2)
	{
		/* Enable USART2 clock */
		__HAL_RCC_USART2_CLK_ENABLE();

		subPriority = 1;
		IRQn = USART2_IRQn;
	}
#endif
#ifdef UART6
	if(UARTx->USARTx == USART6)
	{
		/* Enable USART6 clock */
		__HAL_RCC_USART6_CLK_ENABLE();

		subPriority = 2;
		IRQn = USART6_IRQn;
	}
#endif
	/* if IRQn equal 0 then UARTx not define so UARTx not initialize */
	if(IRQn != 0)
	{
		/* Disable Interrupt */
		HAL_NVIC_DisableIRQ(IRQn);

		/* Uart GPIO Pins Initialize */
		UART_GPIO_Init(UARTx);

		/* Configure UART Settings */
		UART_Config(UARTx);

		/* Set priority for NVIC interrupt */
		HAL_NVIC_SetPriority(IRQn, UART_NVIC_PRIORITY, subPriority);

		/* Enable NVIC interrupt for UART */
		HAL_NVIC_EnableIRQ(IRQn);
	}
}

void UART_Init(USART_TypeDef* USARTx,uint32_t baud)
{
	UART_T UART = {0U};

	//UART DEFAULT SETTINGS

	UART.USARTx = USARTx;
	UART.Baudrate = baud;
	UART.Mode = UART_MODE_TX_RX;
	UART.WordLength = UART_WORDLENGTH_8B;
	UART.Parity = UART_PARITY_NONE;
	UART.StopBits = UART_STOPBITS_1;
	UART.HWFlwControl = UART_HWCONTROL_NONE;
	UART.OverSampling = UART_OVERSAMPLING_16;

	//UART GPIO DEFAULT SETTINGS
#ifdef STM32F411xE
#ifdef UART1
	if(USARTx == USART1)
	{
		UART.GPIO_TX = GPIOA;
		UART.Pin_TX = GPIO_PIN_9;
		UART.GPIO_RX = GPIOA;
		UART.Pin_RX = GPIO_PIN_10;
	}
#endif
#ifdef UART2
	if(USARTx == USART2)
	{
		UART.GPIO_TX = GPIOA;
		UART.Pin_TX = GPIO_PIN_2;
		UART.GPIO_RX = GPIOA;
		UART.Pin_RX = GPIO_PIN_3;
	}
#endif
#ifdef UART6
	if(USARTx == USART6)
	{
		UART.GPIO_TX = GPIOC;
		UART.Pin_TX = GPIO_PIN_6;
		UART.GPIO_RX = GPIOC;
		UART.Pin_RX = GPIO_PIN_7;
	}
#endif
#endif
	//Initialise Uart
	UART_ADVANCE_Init(&UART);
}

void UART_WthPin_Init(USART_TypeDef* USARTx, uint32_t baud, GPIO_TypeDef* GPIO_RX, uint16_t Pin_RX, GPIO_TypeDef* GPIO_TX, uint16_t Pin_TX)
{
	UART_T UART = {0U};

	//UART DEFAULT SETTINGS

	UART.USARTx = USARTx;
	UART.Baudrate = baud;
	UART.Mode = UART_MODE_TX_RX;
	UART.WordLength = UART_WORDLENGTH_8B;
	UART.Parity = UART_PARITY_NONE;
	UART.StopBits = UART_STOPBITS_1;
	UART.HWFlwControl = UART_HWCONTROL_NONE;
	UART.OverSampling = UART_OVERSAMPLING_16;

	//UART GPIO SETTINGS

	UART.GPIO_TX = GPIO_TX;
	UART.Pin_TX = Pin_TX;
	UART.GPIO_RX = GPIO_RX;
	UART.Pin_RX = Pin_RX;

	//Initialise Uart
	UART_ADVANCE_Init(&UART);
}

void UART_GPIO_Init(UART_T* UARTx)
{
	uint8_t Alternate = 0;

#ifdef STM32F411xE
#ifdef UART1
	if(UARTx->USARTx == USART1)
	{
		Alternate = GPIO_AF7_USART1;
	}
#endif
#ifdef UART2
	if(UARTx->USARTx == USART2)
	{
		Alternate = GPIO_AF7_USART2;
	}
#endif
#ifdef UART3
	if(UARTx->USARTx == USART3)
	{
		Alternate = GPIO_AF7_USART3;
	}
#endif
#ifdef UART4
	if(UARTx->USARTx == USART4)
	{
		Alternate = GPIO_AF8_USART4;
	}
#endif
#ifdef UART5
	if(UARTx->USARTx == USART5)
	{
		Alternate = GPIO_AF8_USART5;
	}
#endif
#ifdef UART6
	if(UARTx->USARTx == USART6)
	{
		Alternate = GPIO_AF8_USART6;
	}
#endif
#else
#error "MCU NOT DEFINED"
#endif

	GPIO_init(UARTx->GPIO_RX, UARTx->Pin_RX, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO_PUPD_NOPULL, Alternate);
	GPIO_init(UARTx->GPIO_TX, UARTx->Pin_TX, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO_PUPD_NOPULL, Alternate);

	if(UARTx->HWFlwControl != UART_HWCONTROL_NONE)
	{
		GPIO_init(UARTx->GPIO_CTS, UARTx->Pin_CTS, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO_PUPD_NOPULL, Alternate);
		GPIO_init(UARTx->GPIO_RTS, UARTx->Pin_RTS, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO_PUPD_NOPULL, Alternate);
	}
}

ring_buffer_t* GET_UART_BUFFER_RX(USART_TypeDef* USARTx)
{
	ring_buffer_t* temp = 0;

#ifdef UART1
	if(USARTx == USART1)
	{
		temp = &UART1_BUFFER_RX;
	}
#endif
#ifdef UART2
	if(USARTx == USART2)
	{
		temp = &UART2_BUFFER_RX;
	}
#endif
#ifdef UART3
	if(USARTx == USART3)
	{
		temp = &UART3_BUFFER_RX;
	}
#endif
#ifdef UART4
	if(USARTx == USART4)
	{
		temp = &UART4_BUFFER_RX;
	}
#endif
#ifdef UART5
	if(USARTx == USART5)
	{
		temp = &UART5_BUFFER_RX;
	}
#endif
#ifdef UART6
	if(USARTx == USART6)
	{
		temp = &UART6_BUFFER_RX;
	}
#endif

	return temp;
}

ring_buffer_t* GET_UART_BUFFER_TX(USART_TypeDef* USARTx)
{
	ring_buffer_t* temp = 0;

#ifdef UART1
	if(USARTx == USART1)
	{
		temp = &UART1_BUFFER_TX;
	}
#endif
#ifdef UART2
	if(USARTx == USART2)
	{
		temp = &UART2_BUFFER_TX;
	}
#endif
#ifdef UART3
	if(USARTx == USART3)
	{
		temp = &UART3_BUFFER_TX;
	}
#endif
#ifdef UART4
	if(USARTx == USART4)
	{
		temp = &UART4_BUFFER_TX;
	}
#endif
#ifdef UART5
	if(USARTx == USART5)
	{
		temp = &UART5_BUFFER_TX;
	}
#endif
#ifdef UART6
	if(USARTx == USART6)
	{
		temp = &UART6_BUFFER_TX;
	}
#endif

	return temp;
}

void UART_Config(UART_T* UART)
{

	/*##Configure the UART peripheral##*/

	/* Disable UART */
	UART->USARTx->CR1 &= ~(1UL << 13U);

	/* UART CR2 Register Settings */
	UART->USARTx->CR2 &= ~(3UL << 12U);
	UART->USARTx->CR2 |= UART->StopBits;

	/* UART CR1 Register Settings */
	UART->USARTx->CR1 &= (~(1UL << 15U) | ~(1UL << 12U) | ~(3UL << 9U) | ~(3UL << 2U) | ~(1UL << 13U) | ~(1UL << 5U));
	UART->USARTx->CR1 |= (UART->OverSampling | UART->WordLength | UART->Parity | UART->Mode);

	/* UART CR3 Register Settings */
	UART->USARTx->CR3 &= ~(8UL << 9UL);
	UART->USARTx->CR3 |= UART->HWFlwControl;

	/* UART BRR Register Settings */
	UART->USARTx->BRR &= ~0xffff;
	UART->USARTx->BRR |= calc_USARTDIV(UART->USARTx, UART->Baudrate);

	/* UART Enable */
	UART->USARTx->CR1 |= (1UL << 13U);

	/* UART Reciever(RXNEIE) Interrupt Enable */
	UART->USARTx->CR1 |= (1UL << 5U);
}

uint16_t calc_USARTDIV(USART_TypeDef* USARTx, uint32_t baud){
	SystemCoreClockUpdate();

	uint32_t APB_CLOCK = 0;

#ifdef STM32F411xE
	if(USARTx == USART2)
	{
		APB_CLOCK = HAL_RCC_GetPCLK1Freq();
	}
	else if((USARTx == USART1) | (USARTx == USART6))
	{
		APB_CLOCK = HAL_RCC_GetPCLK2Freq();
	}
#else
#error "MCU NOT DEFINED"
#endif

	/* Calculate Mantis and Fract for BRR */
	uint32_t divider = ((APB_CLOCK * 25)/ (2 * ((2 - ((USARTx->CR1 >> 15U) & 1))) * baud));
	uint32_t mantissa = (uint32_t) divider / 100;
	uint32_t fractional = (uint32_t)((((divider - (mantissa * 100)) * (16 / (2 - !((USARTx->CR1 >> 15U) & 1)))) + 50) / 100);


	if((USARTx->CR1 >> 15U) & 1)
	{
		/* OverSampling 8 */
		return (mantissa << 4) + ((fractional & 0xF8U) << 1U) + (fractional & 0xF7);
	}
	else
	{
		/* OverSampling 16 */
		return (mantissa << 4) + (fractional & 0xF0U) + (fractional & 0x0F);
	}
}

uint16_t UART_BytesToRead(USART_TypeDef* USARTx)
{
	return RING_BUFFER_Bytes(GET_UART_BUFFER_RX(USARTx));
}

uint16_t UART_BytesToWrite(USART_TypeDef* USARTx)
{
	return RING_BUFFER_Bytes(GET_UART_BUFFER_TX(USARTx));
}

uint8_t UART_Read_Byte(USART_TypeDef* USARTx)
{
	uint8_t DataByte =  0;

	if(RING_BUFFER_Read(GET_UART_BUFFER_RX(USARTx), &DataByte) == 0)
	{
		return 0;
	}

	return DataByte;
}

void UART_Read_ByteArray(USART_TypeDef* USARTx, uint8_t* data, uint32_t size)
{
	uint32_t i = 0;
	uint32_t previousMillis = HAL_GetTick();

	while(i < size)
	{
		if(UART_BytesToRead(USARTx))
		{
			data[i++] = UART_Read_Byte(USARTx);
			previousMillis = HAL_GetTick();
		}
		else{
			if(HAL_GetTick() - previousMillis >= UART_ByteArray_TimeOut)
			{
				/* If a new byte does not arrive within UART_ByteArray_TimeOut(ms) time after 1 byte is received, return. */
				return;
			}
		}
	}
}

void UART_Read_String(USART_TypeDef* USARTx, uint8_t* string)
{
	uint32_t i = 0;
	uint8_t temp[STRING_READ_SIZE] = {0U};
	uint32_t pre = HAL_GetTick();

	while((i < STRING_READ_SIZE) && ((HAL_GetTick() - pre) < UART_String_TimeOut)){
		if(UART_BytesToRead(USARTx)){
			temp[i++] = UART_Read_Byte(USARTx);
		}
	}

	temp[i] = '\0';

	strcpy((char*)string, (char*)temp);
}

void UART_Send_Byte(USART_TypeDef* USARTx, uint8_t byte)
{
	ring_buffer_t* Buffer = GET_UART_BUFFER_TX(USARTx);

	while(RING_BUFFER_Full(Buffer) == 1);

	/// Add the data to TX buffer ///

	if(RING_BUFFER_Write(Buffer, byte) == 0)
	{
		return;
	}

	USARTx->CR1 |= (1UL << 7U);

}

void UART_Send_ByteArray(USART_TypeDef* USARTx, const uint8_t* buffer, uint32_t size)
{
	uint32_t i = 0;

	for( i = 0; i < size; i++)
	{
		UART_Send_Byte(USARTx, buffer[i]);
	}
}

void UART_Send_String(USART_TypeDef* USARTx, uint8_t* string)
{
    const uint8_t *byte = string;

	while(*byte)
	{
		UART_Send_Byte(USARTx, *byte);
		byte++;
	}
}

void UART_Set_StringTimeOut(uint32_t timeOut)
{
	UART_String_TimeOut = timeOut;
}

void UART_Set_ByteArray_TimeOut(uint32_t timeOut)
{
	UART_ByteArray_TimeOut = timeOut;
}

////UART1 ////
#ifdef UART1
void USART1_IRQHandler(void)
{
	uint32_t isrflags = USART1->SR;
	uint32_t control_reg1 = USART1->CR1;

	/* Clear Error Flags */
	if (((isrflags & USART_SR_IDLE) || (isrflags & USART_SR_ORE)))
	{
		volatile uint32_t temp = USART1->SR;
		temp = USART1->DR;
		(void)temp;
	}

	/* UART in mode Receiver */
	if(((isrflags & USART_SR_RXNE) != RESET) && ((control_reg1 & USART_CR1_RXNEIE) != RESET))
	{
		/* Read one byte from the receive data register */
		if(RING_BUFFER_Write(&UART1_BUFFER_RX, USART1->DR) == 0)
		{
		//buffer is full byte will lose
		}
		return;
	}

	/* UART in mode Transmitter */
	if(((isrflags & USART_SR_TXE) != RESET) && ((control_reg1 & USART_CR1_TXEIE) != RESET))
	{
		// Send one byte from Transmit buffer
		if(RING_BUFFER_Read(&UART1_BUFFER_TX, (uint8_t* )&USART1->DR) == 0)
		{
			/* Disable the UART Transmit Data Register Empty Interrupt
			 * when tx buffer is empty */
			USART1->CR1 &= ~(1UL << 7U);
		}

		return;
	}
}
#endif

/// UART2 ///
#ifdef UART2
void USART2_IRQHandler(void)
{
	uint32_t isrflags = USART2->SR;
	uint32_t control_reg1 = USART2->CR1;

	/* Clear Error Flags */
	if (((isrflags & USART_SR_IDLE) || (isrflags & USART_SR_ORE)))
	{
		volatile uint32_t temp = USART2->SR;
		temp = USART2->DR;
		(void)temp;
	}

	/* UART in mode Receiver */
	if(((isrflags & USART_SR_RXNE) != RESET) && ((control_reg1 & USART_CR1_RXNEIE) != RESET))
	{
		/* Read one byte from the receive data register */
		if(RING_BUFFER_Write(&UART2_BUFFER_RX, USART2->DR) == 0)
		{
		//buffer is full byte will lose
		}

		return;
	}

	/* UART in mode Transmitter */
	if(((isrflags & USART_SR_TXE) != RESET) && ((control_reg1 & USART_CR1_TXEIE) != RESET))
	{
		// Send one byte from Transmit buffer
		if(RING_BUFFER_Read(&UART2_BUFFER_TX, (uint8_t* )&USART2->DR) == 0)
		{
			/* Disable the UART Transmit Data Register Empty Interrupt
			 * when tx buffer is empty */
			USART2->CR1 &= ~(1UL << 7U);
		}

		return;
	}
}
#endif

/// UART6 ///
#ifdef UART6
void USART6_IRQHandler(void)
{
	uint32_t isrflags = USART6->SR;
	uint32_t control_reg1 = USART6->CR1;

	/* Clear Error Flags */
	if (((USART6->SR & USART_SR_IDLE) || (USART6->SR & USART_SR_ORE)))
	{
		volatile uint32_t temp = USART6->SR;
		temp = USART6->DR;
		(void)temp;
	}

	/* UART in mode Receiver */
	if(((isrflags & USART_SR_RXNE) != RESET) && ((control_reg1 & USART_CR1_RXNEIE) != RESET))
	{
		/* Read one byte from the receive data register */
		if(RING_BUFFER_Write(&UART6_BUFFER_RX, USART6->DR) == 0)
		{
		//buffer is full byte will lose
		}

		return;
	}

	/* UART in mode Transmitter */
	if(((isrflags & USART_SR_TXE) != RESET) && ((control_reg1 & USART_CR1_TXEIE) != RESET))
	{
		// Send one byte from Transmit buffer
		if(RING_BUFFER_Read(&UART6_BUFFER_TX, (uint8_t* )&USART6->DR) == 0)
		{
			/* Disable the UART Transmit Data Register Empty Interrupt
			 * when tx buffer is empty */
			USART6->CR1 &= ~(1UL << 7U);
		}

		return;
	}
}
#endif

