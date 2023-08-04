/*
 * User_Uart.h
 *
 *  Created on: Jul 30, 2021
 *      Author: Admin
 */
#include "User_cfg.h"
#ifndef TUEDRIVER_DEVICE_USERUART_USER_UART_H_
#define TUEDRIVER_DEVICE_USERUART_USER_UART_H_



#define USER_UART_MAX_MESSAGE_LENGHH		128
extern void User_Uart_Init(const User_Uart_ConfigType *UartConfing);

extern Std_ReturnType User_Uart_AsyncTransmit(uint8_t u8ChannelID, uint8_t *u8DatPtr,uint16_t u16Datalen);

extern Std_ReturnType User_Uart_Received(uint8_t u8ChannelID, uint8_t **u8DataPtr, uint16_t u16DataLen, int16_t *int16remain);




#endif /* TUEDRIVER_DEVICE_USERUART_USER_UART_H_ */
