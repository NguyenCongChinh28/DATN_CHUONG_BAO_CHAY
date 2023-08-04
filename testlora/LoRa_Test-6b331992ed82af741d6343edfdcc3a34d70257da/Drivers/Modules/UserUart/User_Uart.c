/*
 * User_Uart.c
 *
 *  Created on: Jul 30, 2021
 *      Author: Admin
 */
#include <User_Uart_Ipw.h>
#include "User_cfg.h"
#include "User_Uart.h"
static User_Uart_ConfigType *ConfigPtr = NULL;
static uint8_t User_Uart_RxBuffer[USER_UART_NUMBEROFCHANNEL_USED][USER_UART_MAX_MESSAGE_LENGHH];

void User_Uart_Init(const User_Uart_ConfigType *UartConfing)
{
	User_Uart_HwConfigType HwConfig;
	uint8_t u8CountChannel;
	ConfigPtr = (User_Uart_ConfigType*) UartConfing;

	for ( u8CountChannel = 0 ; u8CountChannel <UartConfing->NumberofChannel; u8CountChannel++)
	{
		HwConfig.ParityMode = UartConfing->ChannelConfigType[u8CountChannel].ParityMode;
		HwConfig.SlipStatus = UartConfing->ChannelConfigType[u8CountChannel].SlipStatus;
		HwConfig.StopBitCount = UartConfing->ChannelConfigType[u8CountChannel].StopBitCount;
		HwConfig.u32BaudRate = UartConfing->ChannelConfigType[u8CountChannel].u32BaudRate;
		HwConfig.u32SourceClock = UartConfing->ChannelConfigType[u8CountChannel].u32SourceClock;
		HwConfig.u8ChannelID = UartConfing->ChannelConfigType[u8CountChannel].u8ChannelID;
		User_Uart_Ipw_Init(&HwConfig);
	}
}

Std_ReturnType User_Uart_AsyncTransmit(uint8_t u8ChannelID, uint8_t *u8DatPtr,uint16_t u16Datalen)
{
	Std_ReturnType ret = E_NOT_OK;
	uint8_t u8Count;
	uint8_t ChannelID = 0xFFU;
	if (ConfigPtr == NULL)
	{
		return E_NOT_OK;
	}
	if (u8DatPtr == NULL)
	{
		return E_NOT_OK;
	}
	for( u8Count = 0;  u8Count < ConfigPtr->NumberofChannel ; u8Count++)
	{
		if(ConfigPtr->ChannelConfigType[u8Count].u8ChannelID == u8ChannelID )
		{
			ChannelID = u8Count;
			break;
		}
	}
	if (ChannelID == 0xFFU)
	{
		return E_NOT_OK;
	}
	if (ConfigPtr->ChannelConfigType[ChannelID].SlipStatus == USER_UART_SLIPSTATUS_ENABLE)
	{
	//	ret = User_Uart_Ipw_SlipSend();
	}
	else
	{
		ret = User_Uart_Ipw_AsyncTransmit(u8ChannelID,u8DatPtr,u16Datalen);
	}
	return ret;
}
Std_ReturnType User_Uart_Received(uint8_t u8ChannelID, uint8_t **u8DataPtr, uint16_t u16DataLen, int16_t *int16remain)
{
	uint8_t u8Count;
	uint8_t ChannelID = 0xFFU;
	Std_ReturnType ret = E_NOT_OK;
	if (ConfigPtr == NULL)
	{
		return E_NOT_OK;
	}
	for( u8Count = 0;  u8Count < ConfigPtr->NumberofChannel ; u8Count++)
	{
		if(ConfigPtr->ChannelConfigType[u8Count].u8ChannelID == u8ChannelID )
		{
			ChannelID = u8Count;
			break;
		}
	}
	if (ChannelID == 0xFFU)
	{
		return E_NOT_OK;
	}
	if (ConfigPtr->ChannelConfigType[ChannelID].SlipStatus == USER_UART_SLIPSTATUS_ENABLE)
	{
	//	ret = User_Uart_Ipw_SlipSend();
	}
	else
	{
		ret = User_Uart_Ipw_Received(u8ChannelID,&User_Uart_RxBuffer[ChannelID][0],u16DataLen,int16remain);
	}
	*u8DataPtr = &User_Uart_RxBuffer[ChannelID][0];
	return ret;

}




