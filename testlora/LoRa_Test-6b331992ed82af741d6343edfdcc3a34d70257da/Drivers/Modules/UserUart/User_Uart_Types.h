/*
 * User_Uart_Types.h
 *
 *  Created on: Jul 30, 2021
 *      Author: Admin
 */

#ifndef TUEDRIVER_DEVICE_USERUART_USER_UART_TYPES_H_
#define TUEDRIVER_DEVICE_USERUART_USER_UART_TYPES_H_
#include "Std_Types.h"

#define NUMBER_OF_CHANNEL_USED			(2U)
#define NUMBER_OF_HARDWARE_USED			(2U)

typedef enum {
	USER_UART_PARITYMODE_NONE			= 2U,
	USER_UART_PARITYMODE_EVEN			= 0U,
	USER_UART_PARITYMODE_ODD			= 1U
}User_Uart_ParityMode_t;

typedef enum {
	USER_UART_STOPBITCOUNT_1			= 0U,
	USER_UART_STOPBITCOUNT_0_5			= 1U,
	USER_UART_STOPBITCOUNT_2			= 2U,
	USER_UART_STOPBITCOUNT_1_5			= 3u
}User_Uart_StopBitCount_t;
typedef enum{
	USER_UART_SLIPSTATUS_ENABLE			= 0U,
	USER_UART_SLIPSTATUS_DISABLE		= 1U
}User_Uart_SlipStatus_t;


typedef struct {
	uint8_t 					u8ChannelID;
	uint32_t 					u32SourceClock;
	uint32_t 					u32BaudRate;
	User_Uart_ParityMode_t		ParityMode;
	User_Uart_StopBitCount_t	StopBitCount;
	User_Uart_SlipStatus_t		SlipStatus;
}User_Uart_HwConfigType;
typedef struct {
	uint8_t NumberofChannel;
	uint8_t NumberofHardware;
	User_Uart_HwConfigType  *ChannelConfigType;
}User_Uart_ConfigType;













#endif /* TUEDRIVER_DEVICE_USERUART_USER_UART_TYPES_H_ */
