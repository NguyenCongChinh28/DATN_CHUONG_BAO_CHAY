/*
 * Layer1.c
 *
 *  Created on: Nov 18, 2021
 *      Author: Admin
 */


#include "Layer1.h"
#include "Config.h"
#include "String.h"
//#include "app_debug.h"
#include "sx126x.h"
#include <stdlib.h>
#include <string.h>

#define MSG_PAYLOAD			0x01
#define MSG_ACK				0x02



uint8_t	Message_BroadCast[]	= "SYN";
lora_mode mode;

resend_struct resend;
Std_ReturnType Receive_Handle(uint8_t *payload);

uint32_t mac[3];
extern uint8_t ACK_Check[10];
extern uint8_t ACK_Send[10];

static uint8_t currentmode = 0;

extern packet_ringbuffer DataBuffer;

extern slotted_time_struct slot;


uint8_t isMaster = 0;






void L1_Init()
{
	mac[0] = *(uint32_t *)(0x1FFFF7E8); // high address
	mac[1] = *(uint32_t  *)(0x1FFFF7E8 + 4); //
	mac[2] = *(uint32_t *)(0x1FFFF7E8 + 8); //  Low address
	for(uint8_t Count = 0; Count < MAX_BUFFER_SIZE; Count++)
	{
		DataBuffer.RxBuffer[Count].payload_ptr = (uint8_t*)calloc(3,sizeof(uint8_t));
	}
	if(isMaster == 0)
	{
		currentmode = 0;
	}
	else
	{
		currentmode = 1;
	}
}


uint8_t LoRa_WriteBuffer(User_ID receiver,uint8_t msg_type, uint8_t *payload)
{
	packet_ringbuffer *buff = &DataBuffer;
	if(buff->TxHead != buff->TxTail)
	{
		if((buff->TxHead + 1) == buff->TxTail)
		{
			return 0;
		}
	}
	buff->TxBuffer[buff->TxHead].receiver = receiver;
	buff->TxBuffer[buff->TxHead].sender = NODEID;
	buff->TxBuffer[buff->TxHead].msg_type = msg_type;
	buff->TxBuffer[buff->TxHead].macaddr = mac[2];
	if(msg_type == MSG_PAYLOAD)
	{
		ACK_Check[receiver] = 1;
	}

	buff->TxBuffer[buff->TxHead].payload_ptr = payload;
	buff->TxHead = (buff->TxHead + 1) % (MAX_BUFFER_SIZE - 1);
	return E_OK;
}
static Std_ReturnType Send_Packet()
{
	packet_ringbuffer *buff = &DataBuffer;
	if(buff->TxHead == buff->TxTail)
	{
		return E_NOT_OK;
	}
	uint8_t TxSend[10];
	//memcpy(TxSend,buff[buff->TxTail].TxBuffer,10);
	TxSend[0] = buff->TxBuffer[buff->TxTail].sender;
	TxSend[1] = buff->TxBuffer[buff->TxTail].receiver;
	TxSend[2] = buff->TxBuffer[buff->TxTail].msg_type;
	TxSend[3] = (buff->TxBuffer[buff->TxTail].macaddr >> 24);
	TxSend[4] = (buff->TxBuffer[buff->TxTail].macaddr >> 16);
	TxSend[5] = (buff->TxBuffer[buff->TxTail].macaddr >> 8);
	TxSend[6] = (buff->TxBuffer[buff->TxTail].macaddr);
	memcpy(&TxSend[7],buff->TxBuffer[buff->TxTail].payload_ptr,3);
	Radio.Send(TxSend,10);
	buff->TxTail = (buff->TxTail+1) % MAX_BUFFER_SIZE;
	return E_OK;
}








Std_ReturnType LoRa_ASynTransmit(User_ID receiver,uint8_t msg_type, uint8_t *payload)
{
	if(slot.timer > slot.rnd_num)
	{
		Radio.StartCad();
		HAL_ADC_Start_IT(&hadc1);
		slot.timer = 0;
	}
	if(slot.send_enable == true)
	{
		if(LoRa_WriteBuffer(receiver,msg_type,payload) == E_OK)
		{
			Send_Packet();
			slot.timer= 0;
			slot.send_enable = 0;
			return E_OK;
		}
		return E_NOT_OK;

	}

	return E_NOT_OK;
}

Std_ReturnType Receive_Handle(uint8_t *payload)
{
	uint8_t data[3] = {0};
	if(payload == NULL)
	{
		return E_NOT_OK;
	}
	if(DataBuffer.RxHead != DataBuffer.RxTail)
	{
		if(DataBuffer.RxHead++ == DataBuffer.RxTail )
		{
			return E_NOT_OK;
		}
	}
	DataBuffer.RxBuffer[DataBuffer.RxHead].sender = *payload;
	DataBuffer.RxBuffer[DataBuffer.RxHead].receiver = *(payload+1);
	DataBuffer.RxBuffer[DataBuffer.RxHead].msg_type = *(payload+2);
	DataBuffer.RxBuffer[DataBuffer.RxHead].macaddr = *(payload+3);
	DataBuffer.RxBuffer[DataBuffer.RxHead].macaddr = (DataBuffer.RxBuffer[DataBuffer.RxHead].macaddr << 8) + *(payload+4);
	DataBuffer.RxBuffer[DataBuffer.RxHead].macaddr = (DataBuffer.RxBuffer[DataBuffer.RxHead].macaddr << 8) + *(payload+5);
	DataBuffer.RxBuffer[DataBuffer.RxHead].macaddr = (DataBuffer.RxBuffer[DataBuffer.RxHead].macaddr << 8) + *(payload+6);
	//strcpy(DataBuffer.RxBuffer[DataBuffer.RxHead].payload_ptr,(payload+7));
	data[0] = *(payload+7);
	data[1] = *(payload+8);
	data[2] = *(payload+9);
	memcpy(DataBuffer.RxBuffer[DataBuffer.RxHead].payload_ptr,data,3);
	DataBuffer.RxHead = (DataBuffer.RxHead + 1) % MAX_BUFFER_SIZE;
	return E_OK;
}

Std_ReturnType LoRa_BufferHandle(uint8_t *payload)
{
	if( Receive_Handle(payload) == E_NOT_OK )
	{
		return E_NOT_OK;
	}
	if(DataBuffer.RxHead == DataBuffer.RxTail)
	{
		return E_NOT_OK;
	}
	if(DataBuffer.RxBuffer[DataBuffer.RxTail].sender == NODEID ||
	   (DataBuffer.RxBuffer[DataBuffer.RxTail].receiver != NODEID &&
	   DataBuffer.RxBuffer[DataBuffer.RxTail].receiver != 0xFF ))
	{
		Radio.Rx(1000,1);
		DataBuffer.RxTail = (DataBuffer.RxTail+1) % MAX_BUFFER_SIZE;
		return E_NOT_OK;
	}

	if(DataBuffer.RxBuffer[DataBuffer.RxTail].msg_type == MSG_ACK)
	{
		ACK_Check[DataBuffer.RxBuffer[DataBuffer.RxTail].sender] = 0;
		DEBUG_INFO("Received ACK from node %d\r\n",DataBuffer.RxBuffer[DataBuffer.RxTail].sender);
		if(isMaster == 0)
		{
			Radio.Rx(1000,1);
			currentmode = 0;
		}
	}
	else if( DataBuffer.RxBuffer[DataBuffer.RxTail].msg_type == MSG_PAYLOAD)
	{
			currentmode = 4;
			resend.last_node = DataBuffer.RxBuffer[DataBuffer.RxTail].sender;
			if(strncmp(DataBuffer.RxBuffer[DataBuffer.RxTail].payload_ptr,"SYN",3) == 0)
			{
				DEBUG_INFO("Received Synchronization Message/r/n");
				Radio.Standby();
			}

	}
	DataBuffer.RxTail = (DataBuffer.RxTail+1) % MAX_BUFFER_SIZE;
	return E_OK;
}

void LoRa_SendBroadCast()
{
	while(LoRa_ASynTransmit(User_BroadCast,MSG_PAYLOAD,Message_BroadCast) == E_NOT_OK);
	DEBUG_INFO("BroadCastInforSend\r\n");
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	//HAL_Delay(500);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	for(uint8_t NodeCount = 0 ; NodeCount < MAXNODES ; NodeCount++)
	{
		ACK_Check[NodeCount] = 1;
	}
	resend.last_message = Message_BroadCast;
	resend.last_message_type = MSG_PAYLOAD;
	resend.last_node = User_BroadCast;
	//DEBUG_INFO("SendBroadCast\r\n");
	//currentmode = 2;
}
void LoRa_SendACK(User_ID rec)
{
	while(LoRa_ASynTransmit(rec,MSG_ACK,"ACK") == E_NOT_OK);
	DEBUG_INFO("RESEND PAYLOAD to %d",rec);
}












void LoRa_Polling()
{
	uint32_t time32 = 0;
	switch(currentmode)
	{
		case 0:
		{
			Radio.Rx(1000,1);
			while(1)
			{
				if(currentmode != 0)
				{
					break;
				}
			}
		}
		break;
		case 1:
		{
			time32 = HAL_GetTick();
			while(1)
			{
				if( (HAL_GetTick() - time32) >= 3000)
				{
					LoRa_SendBroadCast();
					//Radio.Rx(1,1);
					//currentmode = 2;
					break;
				}
			}
		}
		break;
		case 2:
		{
			uint8_t condition = 1;
			Radio.Rx(10000,1);
			time32 = HAL_GetTick();
			while(condition)
			{
				if(resend.last_node == User_BroadCast)
				{
					if((HAL_GetTick() - time32) > 300000 )
					{
						for(uint8_t Ncount = 0 ; Ncount < MAXNODES; Ncount++)
						{
							if(ACK_Check[Ncount] == 1)
							{
								currentmode = 1;
								time32 = HAL_GetTick();
								DEBUG_INFO("Resend_BroadCast\r\n");
								condition = 0;
								break;
							}
						}
					}
				}
			}
		}
		break;
		case 4:
		{
			Radio.Standby();
			HAL_Delay(2000);
			LoRa_SendACK(resend.last_node);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
			if(isMaster == 1)
			{
				currentmode = 2;
				resend.last_node = User_BroadCast;
			}
			else
			{
				currentmode = 0;
			}
		}break;
		default:
			break;
	}
}



void Timer_Callback()
{
//
}













































