/*
 * Types.h
 *
 *  Created on: Nov 18, 2021
 *      Author: Admin
 */

#ifndef LORA_DRIVERS_ALOHA_PROTOCOL_TYPES_H_
#define LORA_DRIVERS_ALOHA_PROTOCOL_TYPES_H_

#define MAX_BUFFER_SIZE 		10

#include "Radio.h"
#include "Config.h"
#include "sx126x.h"

typedef struct
{
	uint8_t sender;
	uint8_t receiver;
	uint8_t msg_type;
	uint32_t macaddr;
	uint8_t *payload_ptr;
}packet_struct_t;


typedef struct
{
	packet_struct_t TxBuffer[MAX_BUFFER_SIZE];
	packet_struct_t RxBuffer[MAX_BUFFER_SIZE];
	uint8_t TxHead;
	uint8_t TxTail;
	uint8_t RxHead;
	uint8_t	RxTail;
}packet_ringbuffer;

typedef struct
{
	uint8_t timer;
	uint8_t rnd_num;
	uint8_t synchro_enable : 1;
	uint8_t send_enable : 1;
	uint8_t pair_enable : 1;
}slotted_time_struct;





typedef struct
{
	uint8_t					current_mode;
	uint8_t 			 	last_mode;
	RadioOperatingModes_t	next_mode;


}lora_mode;



typedef struct
{
	uint8_t *last_message;
	uint8_t last_message_type;
	uint8_t last_node;
}resend_struct;

extern uint8_t ack_enable;









#endif /* LORA_DRIVERS_ALOHA_PROTOCOL_TYPES_H_ */
