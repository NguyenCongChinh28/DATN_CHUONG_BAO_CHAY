/*
 * App_Test.h
 *
 *  Created on: Nov 16, 2021
 *      Author: Admin
 */

#ifndef APP_TEST_APP_TEST_H_
#define APP_TEST_APP_TEST_H_

#include "Radio.h"
#include "adc.h"
#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "tim.h"
#include "User_Uart.h"
#include "app_debug.h"
#define NUMBER_OF_NODE			5

#define GSM_PWR_KEY_Pin GPIO_PIN_12
#define GSM_PWR_KEY_GPIO_Port GPIOB
#define GSM_RESET_Pin GPIO_PIN_3
#define GSM_RESET_GPIO_Port GPIOC
//#define GSM_RX_Pin GPIO_PIN_9
//#define GSM_RX_GPIO_Port GPIOA
//#define GSM_TX_Pin GPIO_PIN_10
//#define GSM_TX_GPIO_Port GPIOA
#define GSM_PWR_EN_Pin GPIO_PIN_12
#define GSM_PWR_EN_GPIO_Port GPIOA

//static bool m_usart1_is_enabled = true;
/*void usart1_control(bool enable)
{
	if (m_usart1_is_enabled == enable)
	{
		// DEBUG_PRINTF("UART state : no changed\r\n");
		return;
	}

	m_usart1_is_enabled = enable;
	User_Uart_Init(&User_Uart_ConfigPtr);
}

uint32_t sys_rand()
{
    return SysTick->VAL;
}

uint32_t sys_get_tick_ms(void)
{
    return HAL_GetTick();
}*/




#define GSM_PWR_EN(x)			{	if (x) \
											HAL_GPIO_WritePin(GSM_PWR_EN_GPIO_Port, GSM_PWR_EN_Pin,1);	\
									else	\
											HAL_GPIO_WritePin(GSM_PWR_EN_GPIO_Port, GSM_PWR_EN_Pin,0);	\
								}

#define GSM_PWR_RESET(x)		{	if (x) \
										HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin,1);	\
									else	\
										HAL_GPIO_WritePin(GSM_RESET_GPIO_Port, GSM_RESET_Pin,0);	\
								}

#define GSM_PWR_KEY(x)			{	if (x) \
										HAL_GPIO_WritePin(GSM_PWR_KEY_GPIO_Port, GSM_PWR_KEY_Pin,1);	\
									else	\
										HAL_GPIO_WritePin(GSM_PWR_KEY_GPIO_Port, GSM_PWR_KEY_Pin,0);	\
								}

#define  GSM_UART_CONTROL(x)







void App_TestMain();

Std_ReturnType LoRa_BufferHandle(uint8_t *payload);





#endif /* APP_TEST_APP_TEST_H_ */

