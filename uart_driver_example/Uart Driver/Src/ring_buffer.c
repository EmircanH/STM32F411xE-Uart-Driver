/*
 *	ring_buffer.c
 *
 *	Created on: Aug 13, 2024
 *      Author: Emircan Hızarcı
 */

#include "ring_buffer.h"

uint8_t RING_BUFFER_Empty(ring_buffer_t *ring_buffer)
{
	if(ring_buffer == 0)
			return -1; // if buffer pointer is null return
	return ring_buffer->head_pointer == ring_buffer->tail_pointer;
}

uint32_t RING_BUFFER_Bytes(ring_buffer_t *ring_buffer)
{
	if(ring_buffer == 0)
		return 0; // if buffer pointer is null return
	return ((ring_buffer->mask + 1) + ring_buffer->head_pointer - ring_buffer->tail_pointer) & ring_buffer->mask;
}
uint8_t RING_BUFFER_Full(ring_buffer_t *ring_buffer)
{
	if(ring_buffer == 0)
		return -1; // if buffer pointer is null return

	uint32_t local_head = ring_buffer->head_pointer;
	uint32_t local_tail = ring_buffer->tail_pointer;
	uint32_t local_next_head = (local_head + 1) & ring_buffer->mask;
	if(local_next_head == local_tail)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
uint8_t RING_BUFFER_Write(ring_buffer_t *ring_buffer, uint8_t byte)
{
	if(ring_buffer == 0)
	{
		return 0; // if buffer pointer is null return
	}

	uint32_t local_head = ring_buffer->head_pointer;
	uint32_t local_tail = ring_buffer->tail_pointer;
	uint32_t local_next_head = (local_head + 1) & ring_buffer->mask;

	if(local_next_head == local_tail)
	{
		return 0;
	}

	ring_buffer->buffer[local_head] = byte;
	ring_buffer->head_pointer = local_next_head;

	return 1;

}
uint8_t RING_BUFFER_Read(ring_buffer_t *ring_buffer, uint8_t *byte)
{
	if(ring_buffer == 0)
	{
		return 0; // if buffer pointer is null return
	}
	uint32_t local_head = ring_buffer->head_pointer;
	uint32_t local_tail = ring_buffer->tail_pointer;

	if(local_head == local_tail) // if buffer is empty return
	{
		return 0;
	}

	*byte =  ring_buffer->buffer[local_tail];
	ring_buffer->tail_pointer = (local_tail + 1) & ring_buffer->mask;

	return 1;
}

uint8_t RING_BUFFER_Peek(ring_buffer_t *ring_buffer)
{
	if(ring_buffer == 0)
	{
		return 0; // if buffer pointer is null return
	}
	return ring_buffer->buffer[ring_buffer->tail_pointer];
}
