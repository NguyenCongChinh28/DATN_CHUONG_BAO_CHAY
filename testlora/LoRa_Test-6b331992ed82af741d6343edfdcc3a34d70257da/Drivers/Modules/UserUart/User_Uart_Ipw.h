/*
 * User_Uart_Iwp.h
 *
 *  Created on: Jul 30, 2021
 *      Author: Admin
 */

#ifndef TUEDRIVER_DEVICE_USERUART_USER_UART_IPW_H_
#define TUEDRIVER_DEVICE_USERUART_USER_UART_IPW_H_
#include "User_cfg.h"
#include "User_Uart_Types.h"
#include "User_Ip_Usart.h"






extern void User_Uart_Ipw_Init(User_Uart_HwConfigType *HwConfig);


extern Std_ReturnType User_Uart_Ipw_AsyncTransmit(uint8_t u8Channel, uint8_t *u8Data, uint16_t u16DataLen);

extern Std_ReturnType User_Uart_Ipw_Received(uint8_t u8Channel, uint8_t *u8Data,uint16_t u16DataLen, int16_t *i16remain);
#endif /* TUEDRIVER_DEVICE_USERUART_USER_UART_IWP_H_ */
