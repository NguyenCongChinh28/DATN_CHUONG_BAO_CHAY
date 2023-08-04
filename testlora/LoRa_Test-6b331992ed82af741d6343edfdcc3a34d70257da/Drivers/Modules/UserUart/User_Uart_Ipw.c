/*
 * User_Uart_Ipw.c
 *
 *  Created on: Jul 30, 2021
 *      Author: Admin
 */
#include "User_Uart_Ipw.h"

void User_Uart_Ipw_Init(User_Uart_HwConfigType *HwConfig)
{

	if ((HwConfig->u8ChannelID >= USER_UART_CHANNEL_USART1) && (HwConfig->u8ChannelID <= USER_UART_CHANNEL_USART6))
	{
		HwConfig->u8ChannelID = HwConfig->u8ChannelID - USER_UART_CHANNEL_USART1;
	}
	User_Ip_Usart_Init(HwConfig);
}
Std_ReturnType User_Uart_Ipw_AsyncTransmit(uint8_t u8Channel, uint8_t *u8Data, uint16_t u16DataLen)
{
	Std_ReturnType ret = E_NOT_OK;

	if ((u8Channel >= USER_UART_CHANNEL_USART1) && (u8Channel <= USER_UART_CHANNEL_USART6))
	{
		u8Channel = u8Channel - USER_UART_CHANNEL_USART1;
		ret = User_Ip_Usart_WriteBuffer(u8Channel, u8Data, u16DataLen);
	}
	return ret;
}

Std_ReturnType User_Uart_Ipw_Received(uint8_t u8Channel, uint8_t *u8Data,uint16_t u16DataLen, int16_t *i16remain)
{
	Std_ReturnType ret = E_NOT_OK;

	if ((u8Channel >= USER_UART_CHANNEL_USART1) && (u8Channel<= USER_UART_CHANNEL_USART6))
	{
		u8Channel = u8Channel - USER_UART_CHANNEL_USART1;
		ret = User_Ip_Usart_ReadBuffer(u8Channel, u8Data, u16DataLen, i16remain);
	}
	return ret;
}

