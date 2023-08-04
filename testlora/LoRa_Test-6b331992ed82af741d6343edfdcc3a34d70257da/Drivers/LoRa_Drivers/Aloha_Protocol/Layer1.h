/*
 * Layer1.h
 *
 *  Created on: Nov 18, 2021
 *      Author: Admin
 */

#ifndef LORA_DRIVERS_ALOHA_PROTOCOL_LAYER1_H_
#define LORA_DRIVERS_ALOHA_PROTOCOL_LAYER1_H_

#include "Radio.h"
#include "App_Test.h"
#include "Std_Types.h"


//packet_ringbuffer DataBuffer;

typedef enum
{
	User_BroadCast 	=		0xFF,
	User_Gateway	=		0xA0,
	User_Node_0		= 		0,
	User_Node_1		= 		1,
	User_Node_2		= 		2,
	User_Node_3		= 		3,
	User_Node_4		= 		4,
	User_Node_5		= 		5,
	User_Node_6		= 		6,
	User_Node_7		= 		7,
	User_Node_8		= 		8,
	User_Node_9		= 		9,
	User_Node_10	= 		10,
	User_Node_11	= 		11,
	User_Node_12	= 		12,
	User_Node_13	= 		13,
	User_Node_14	= 		14,
	User_Node_15	= 		15
}User_ID;




#define 	SYNCH_MESSAGE		0x05
#define 	PAIR_MESSAGE		0x06
#define 	SENSOR_MESSAGE		0x07
#define 	CONTROl_MESSAGE		0x08
#define		ACK_MESSAGE			0x09



Std_ReturnType L1_AsynTransmit(User_ID receiver,uint8_t *payload_data);
Std_ReturnType L1_ReceiveData(uint8_t *data);



extern void LoRa_Polling();
extern void L1_Init();


#endif /* LORA_DRIVERS_ALOHA_PROTOCOL_LAYER1_H_ */
