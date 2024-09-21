/*
 *	uart_driver.h - Uart library for STM32F411xC/xE ARM microcontrollers
 *
 *	Created on: Aug 13, 2024
 *      Author: Emircan Hızarcı
 */

#ifndef SRC_UART_DRIVER_H_
#define SRC_UART_DRIVER_H_

/* C++ */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

#include "ring_buffer.h"
#include "gpio_driver.h"

#ifdef STM32F411xE

/* if you define uart that you use just create defined uarts buffers */
#if !defined (UART1) && !defined (UART2) && !defined (UART6)
#define UART1
#define UART2
#define UART6
#endif

#ifndef UART_HWCONTROL_NONE
#define UART_HWCONTROL_NONE                 0x00000000U
#endif
#ifndef UART_HWCONTROL_RTS
#define UART_HWCONTROL_RTS	                ((uint32_t)USART_CR3_RTSE)
#endif
#ifndef UART_HWCONTROL_CTS
#define UART_HWCONTROL_CTS                  ((uint32_t)USART_CR3_CTSE)
#endif
#ifndef UART_HWCONTROL_RTS_CTS
#define UART_HWCONTROL_RTS_CTS              ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))
#endif

#ifndef	UART_OVERSAMPLING_16
#define UART_OVERSAMPLING_16                0x00000000U
#endif
#ifndef	UART_OVERSAMPLING_8
#define UART_OVERSAMPLING_8                 ((uint32_t)USART_CR1_OVER8)
#endif

#ifndef	UART_PARITY_NONE
#define UART_PARITY_NONE                    0x00000000U
#endif
#ifndef	UART_PARITY_EVEN
#define UART_PARITY_EVEN                    ((uint32_t)USART_CR1_PCE)
#endif
#ifndef	UART_PARITY_ODD
#define UART_PARITY_ODD                     ((uint32_t)(USART_CR1_PCE | USART_CR1_PS))
#endif

#ifndef	UART_WORDLENGTH_8B
#define UART_WORDLENGTH_8B                  0x00000000U
#endif
#ifndef	UART_WORDLENGTH_9B
#define UART_WORDLENGTH_9B                  ((uint32_t)USART_CR1_M)
#endif

#ifndef	UART_MODE_RX
#define UART_MODE_RX                        ((uint32_t)USART_CR1_RE)
#endif
#ifndef	UART_MODE_TX
#define UART_MODE_TX                        ((uint32_t)USART_CR1_TE)
#endif
#ifndef	UART_MODE_TX_RX
#define UART_MODE_TX_RX                     ((uint32_t)(USART_CR1_TE | USART_CR1_RE))
#endif

#ifndef	UART_STOPBITS_1
#define UART_STOPBITS_1                     0x00000000U
#endif
#ifndef	UART_STOPBITS_2
#define UART_STOPBITS_2                     ((uint32_t)USART_CR2_STOP_1)
#endif
#endif

/* Buffers Size Defines */

#ifndef	RING_BUFFER_SIZE
#define RING_BUFFER_SIZE 					(128U)
#endif

#ifdef	UART1
#ifndef UART1_BUFFER_SIZE_RX
#define UART1_BUFFER_SIZE_RX				RING_BUFFER_SIZE
#endif
#ifndef UART1_BUFFER_SIZE_TX
#define UART1_BUFFER_SIZE_TX				RING_BUFFER_SIZE
#endif
#endif

#ifdef	UART2
#ifndef UART2_BUFFER_SIZE_RX
#define UART2_BUFFER_SIZE_RX				RING_BUFFER_SIZE
#endif
#ifndef UART2_BUFFER_SIZE_TX
#define UART2_BUFFER_SIZE_TX				RING_BUFFER_SIZE
#endif
#endif

#ifdef	UART3
#ifndef UART3_BUFFER_SIZE_RX
#define UART3_BUFFER_SIZE_RX				RING_BUFFER_SIZE
#endif
#ifndef UART3_BUFFER_SIZE_TX
#define UART3_BUFFER_SIZE_TX				RING_BUFFER_SIZE
#endif
#endif

#ifdef	UART4
#ifndef UART4_BUFFER_SIZE_RX
#define UART4_BUFFER_SIZE_RX				RING_BUFFER_SIZE
#endif
#ifndef UART4_BUFFER_SIZE_TX
#define UART4_BUFFER_SIZE_TX				RING_BUFFER_SIZE
#endif
#endif

#ifdef	UART5
#ifndef UART5_BUFFER_SIZE_RX
#define UART5_BUFFER_SIZE_RX				RING_BUFFER_SIZE
#endif
#ifndef UART5_BUFFER_SIZE_TX
#define UART5_BUFFER_SIZE_TX				RING_BUFFER_SIZE
#endif
#endif

#ifdef	UART6
#ifndef UART6_BUFFER_SIZE_RX
#define UART6_BUFFER_SIZE_RX				RING_BUFFER_SIZE
#endif
#ifndef UART6_BUFFER_SIZE_TX
#define UART6_BUFFER_SIZE_TX				RING_BUFFER_SIZE
#endif
#endif

#ifndef STRING_TIMEOUT
#define STRING_TIMEOUT 						(60U)
#endif

#ifndef STRING_READ_SIZE
#define STRING_READ_SIZE					(RING_BUFFER_SIZE)
#endif

#ifndef	BYTE_ARRAY_TIMEOUT
#define BYTE_ARRAY_TIMEOUT 					(5U)
#endif

#ifndef	UART_NVIC_PRIORITY
#define UART_NVIC_PRIORITY 					(5U)
#endif

typedef struct{
	USART_TypeDef* USARTx;
	uint32_t Baudrate;
	uint32_t Mode;
	uint32_t WordLength;
	uint32_t Parity;
	uint32_t StopBits;
	uint32_t HWFlwControl;
	uint32_t OverSampling;

//GPIO Reg
	GPIO_TypeDef* GPIO_TX;
	GPIO_TypeDef* GPIO_RX;
	GPIO_TypeDef* GPIO_CTS;
	GPIO_TypeDef* GPIO_RTS;

//GPIO Pins
	uint16_t Pin_TX;
	uint16_t Pin_RX;
	uint16_t Pin_CTS;
	uint16_t Pin_RTS;

}UART_T;

/**
 * @brief	Initializes uart peripheral.
 * @param	UARTx: structure that containing uart settings.
 * @Note	This function enables the interrupts of the uart peripheral you initialize.
 * @Note	You can use this function for advance settings for asynchronous uart.
 * @retval	None.
 */
void UART_ADVANCE_Init(UART_T* UARTx);

/**
 * @brief	Initializes uart peripheral with defaul settings.
 * @default Parity: None, HWFLCtrl: None, StopBits: 1, Mode: RX TX.
 * @default WordLength: 8, OverSampling: 16.
 * @param	USARTx: Pointer of the uart peripheral that you want to use.
 * @param	baud: Baudrate of your uart communication.
 * @param	GPIO_RX: Receiver pins GPIO ports address for uart peripheral.
 * @param	Pin_RX: Receiver pin for uart peripheral.
 * @param	GPIO_TX: Receiver pins GPIO ports address for uart peripheral.
 * @param	Pin_TX: Transmitter pin for uart peripheral.
 * @Note	This function enables the interrupts of the uart peripheral you initialize.
 * @retval	None.
 */
 void UART_WthPin_Init(USART_TypeDef* USARTx, uint32_t baud, GPIO_TypeDef* GPIO_RX, uint16_t Pin_RX, GPIO_TypeDef* GPIO_TX, uint16_t Pin_TX);

/*
 * @brief	Initializes uart peripheral with defaul settings and default pins.
 * @default Parity: None, HWFLCtrl: None, StopBits: 1, Mode: RX TX.
 * @default WordLength: 8, OverSampling: 16.
 * @default For STM32F411xC/xE MCU
 * @default	For UART1 RX_pin: PA10, TX_pin: PA9
 * @default	For UART2 RX_pin: PA3, TX_pin: PA2
 * @default For UART6 RX_pin: PC7, TX_pin: PC6
 * */
 void UART_Init(USART_TypeDef* USARTx,uint32_t baud);

/**
 * @brief	Sends a byte of data via UART1
 * @Note	This function add the data to transmitter buffer of uart1. Then enable transmitter interrupt.
 * @param	data: Data to be sent via UART.
 * @retval	None.
 */
void UART_Send_Byte(USART_TypeDef* USARTx, uint8_t byte);

/**
 * @brief	Sends as many bytes as the size via UART1.
 * @Note	This function call UART1Send_Byte() for each byte.
 * @param	data: Pointer of data packet to be sent via UART.
 * @param	size: size of data packet.
 * @retval	None.
 */
void UART_Send_ByteArray(USART_TypeDef* USARTx, const uint8_t* buffer, uint32_t size);

/**
 * @brief	Sends a string via UART1.
 * @Note	This function call UART1Send_Byte() for each byte.
 * @Note	The length of the string is calculated inside the function.
 * @param	data: Pointer of string to be sent via UART.
 * @retval	None.
 */
void UART_Send_String(USART_TypeDef* USARTx, uint8_t* buffer);

/**
 * @brief	Read a byte via UART1.
 * @Note	This function read the data from Received buffer of uart1.
 * @param	None.
 * @retval	Returns data which read from Received buffer of uart1.
 */
uint8_t UART_Read_Byte(USART_TypeDef* USARTx);

/**
 * @brief	Reads as many bytes as the size via UART1.
 * @Note	This function call UART1Read_Byte() for each byte.
 * @param	data: pointer of data array.
 * @param   size: length of data array.
 * @retval	None
 */
void UART_Read_ByteArray(USART_TypeDef* USARTx, uint8_t* data, uint32_t size);

/**
 * @brief	Reads a string via UART1.
 * @Note	This function call UART1Read_Byte() for each byte.
 * @Note	The function has timeout for string.
 * @param	string: pointer to the variable to which the read string will be assigned.
 * @retval	None.
 */
void UART_Read_String(USART_TypeDef* USARTx, uint8_t* string);

/**
 * @brief	Gives how much data is in the receiver buffer.
 * @retval	Returns the amount of data to be read.
 */
uint16_t UART_BytesToRead(USART_TypeDef* USARTx);

/**
 * @brief	Gives how much data is in the transmitter buffer.
 * @retval	Returns the amount of data to be send.
 */
uint16_t UART_BytesToWrite(USART_TypeDef* USARTx);

/**
 * @brief	Set the timeout of readString.
 * @param	timeout: timeout value.
 * @retval	None.
 */
void UART_Set_StringTimeOut(uint32_t timeOut);

/**
 * @brief	Set the timeout of read byte array.
 * @param	timeout: timeout value.
 * @retval	None.
 */
void UART_Set_ByteArray_TimeOut(uint32_t timeOut);

/* UART INTERRUPTS */
#ifdef UART1
void USART1_IRQHandler(void);
#endif
#ifdef UART2
void USART2_IRQHandler(void);
#endif
#ifdef UART6
void USART6_IRQHandler(void);
#endif

/* C++ */
#ifdef __cplusplus
}
#endif

#endif /* SRC_UARTDRIVER_STM32F4XX_H_ */
