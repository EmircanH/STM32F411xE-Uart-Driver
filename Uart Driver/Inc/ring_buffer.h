/*
 *	ring_buffer.h
 *
 *	Created on: Aug 13, 2024
 *      Author: Emircan Hızarcı
 */

#ifndef SRC_RING_BUFFER_H_
#define SRC_RING_BUFFER_H_

/* C++ */
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

typedef struct{
	uint8_t *buffer;
	uint32_t mask;
	uint32_t head_pointer;
	uint32_t tail_pointer;
}ring_buffer_t;

/**
 * @brief	Gets how many bytes of data are in the buffer.
 * @param	ring_buffer: pointers of the buffer that you want to know how many data bytes.
 * @retval	Amount of bytes in the buffer.
 */
uint32_t RING_BUFFER_Bytes(ring_buffer_t *ring_buffer);

/**
 * @brief	Checks for is the buffer empty.
 * @param	ring_buffer: pointers of the buffer that you want to know is empty.
 * @retval	If the buffer is empty return 1, if is not empty return 0.
 */
uint8_t RING_BUFFER_Empty(ring_buffer_t *ring_buffer);

/**
 * @brief	Checks for is the buffer full.
 * @param	ring_buffer: pointers of the buffer that you want to know is full.
 * @retval	If the buffer is full return 1, if is not full return 0.
 */
uint8_t RING_BUFFER_Full(ring_buffer_t *ring_buffer);

/**
 * @brief	Look the next data in buffer.
 * @Note	Not to get just look.
 * @param	ring_buffer: pointers of the buffer that you want to look next data.
 * @retval	Returns the data pointed to by the tail point.
 */
uint8_t RING_BUFFER_Peek(ring_buffer_t *ring_buffer);

/**
 * @brief	Add data to buffer.
 * @Note	First checks whether the buffer is full. If buffer is full it is return 0. If buffer is not full it continues.
 * @Note	Adds data where the head points and increases the head point and masks the head point so it does not overflow.
 * @param	ring_buffer: pointers of the buffer that you want to add data.
 * @param	byte: The data you want to add to the buffer.
 * @retval	Returns 1 if data added successfully, if data could not be added returns 0.
 */
uint8_t RING_BUFFER_Write(ring_buffer_t *ring_buffer, uint8_t byte);

/**
 * @brief	Read data from buffer.
 * @Note	First checks whether the buffer is empty. If buffer is empty it is return 0. If buffer is not empty it continues.
 * @Note	Reads data where the tail points and increases the tail point and masks the tail point so it does not overflow.
 * @param	ring_buffer: pointer of the buffer that you want to read data.
 * @param	byte: Pointer of the variable to which the read value will be assigned.
 * @retval	Returns 1 if data read successfully, if data could not be read returns 0.
 */
uint8_t RING_BUFFER_Read(ring_buffer_t *ring_buffer, uint8_t *byte);


/**
 * @brief	Gets the string length.
 * @Note	It returns length without including '\0'.
 * @Note	There have to have '\0' value at the end of the str.
 * @param	str: pointer of the string that you want to know length of.
 * @retval	Returns length of the str.
 */

/* C++ */
#ifdef __cplusplus
}
#endif

#endif /* SRC_RING_BUFFER_H_ */
