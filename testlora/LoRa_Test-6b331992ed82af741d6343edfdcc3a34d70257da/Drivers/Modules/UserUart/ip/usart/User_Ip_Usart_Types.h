/*
 * User_Ip_Usart_Types.h
 *
 *  Created on: Jul 30, 2021
 *  Author: Tran Duc Tue
 */



#ifndef TUEDRIVER_DEVICE_USERUART_IP_USART_USER_IP_USART_TYPES_H_
#define TUEDRIVER_DEVICE_USERUART_IP_USART_USER_IP_USART_TYPES_H_




#define USER_IP_USART_SIZE_RINGBUFFER 	(1024U)


#include "User_Ip_Usart_Register.h"






typedef enum {
	USER_BUFFER_UNLOCK	= 	0U,
	USER_BUFFER_LOCK	=	1U,
	USER_BUFFER_FULL	=	2U
}User_Ip_RingBufferStatus_t;

typedef struct {
	volatile uint8_t User_Usart_RxRingBufferChannel[USER_IP_USART_SIZE_RINGBUFFER];
	volatile uint8_t User_Usart_TxRingBufferChannel[USER_IP_USART_SIZE_RINGBUFFER];
	volatile uint32_t RxHead;
	volatile uint32_t RxTail;
	volatile uint32_t TxHead;
	volatile uint32_t TxTail;
	User_Ip_RingBufferStatus_t BufferStatus;
}User_Ip_RingBuffer_t;










#endif /* TUEDRIVER_DEVICE_USERUART_IP_USART_USER_IP_USART_TYPES_H_ */
