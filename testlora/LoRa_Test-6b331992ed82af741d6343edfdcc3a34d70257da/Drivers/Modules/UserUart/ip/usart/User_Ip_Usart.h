/*
 * User_Ip_Usart.h
 *
 *  Created on: Jul 30, 2021
 *      Author: Admin
 */

#ifndef TUEDRIVER_DEVICE_USERUART_IP_USART_USER_IP_USART_H_
#define TUEDRIVER_DEVICE_USERUART_IP_USART_USER_IP_USART_H_

#include "User_Uart_Ipw.h"
#include "User_Uart_Types.h"




#define USER_IP_USART_INSTANCE_COUNT		(3U)
#define USER_IP_USART_NUMBEROFBUFFER		(3U)
#define USER_IP_USART_PRIORITY 				(0U)


#define USER_IP_USART_CHANNELID_1 (0U)
#define USER_IP_USART_CHANNELID_2 (1U)
#define USER_IP_USART_CHANNELID_3 (3U)







extern void User_Ip_Usart_Init(User_Uart_HwConfigType *HwConfig);
extern Std_ReturnType User_Ip_Usart_WriteBuffer(uint8_t u8ChannelID, const uint8_t *u8Data, uint16_t u16DataLen);
extern Std_ReturnType User_Ip_Usart_ReadBuffer(uint8_t u8ChannelID, uint8_t *u8Data, uint16_t u16DataLen,int16_t *i16reamain);

#endif /* TUEDRIVER_DEVICE_USERUART_IP_USART_USER_IP_USART_H_ */
