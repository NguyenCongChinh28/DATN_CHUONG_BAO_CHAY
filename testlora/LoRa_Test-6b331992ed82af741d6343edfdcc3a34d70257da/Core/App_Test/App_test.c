/*
 * App_Test.c
 *
 *  Created on: Oct 25, 2021
 *      Author: Admin
 */

#include "App_Test.h"
#include "Layer1.h"
#include "Types.h"
#include "Button.h"

uint8_t test_buzz = 0;

 packet_ringbuffer DataBuffer;

bool EnableMaster=true;//Ö÷´ÓÑ¡Ôñ

#define RF_FREQUENCY                                434000000 // Hz
#define TX_OUTPUT_POWER                             20        // 22 dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       10        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false



#define PING_MESSAGE			0x01
#define PONG_MESSAGE			0x02



#define SESSION_DEFAULT			0x00
#define SESSION_RESPONSE		0x01

typedef void(*Session_Ptr)(void);
void Session_Default();
void Session_Response();
void Session_Receive();
Session_Ptr Session_Current[2] = { Session_Default,Session_Response};
uint8_t SessionID = 0;











uint32_t slotcount = 0;

uint8_t ACK_Check[MAXNODES];
uint8_t ACK_Send[MAXNODES];
uint8_t ack_enable ;


void Payload_Handle();
void LoRa_IRQHandle();



slotted_time_struct slot;







#define ACK_MESSAGE_T			0x03




uint8_t TransmitID = 0;

//uint8_t* data[100];
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;






void OnCaDDetectted(bool channelActivityDetected )
{

	if( channelActivityDetected )
	{
		slot.send_enable = 0;
		HAL_ADC_Start_IT(&hadc1);
	}
	else
	{
		slot.send_enable = 1;
	}
}
void OnTxDone( void )
{
	DEBUG_INFO("Message Sended\r\n");
	//  Radio.Rx(1,0); dung de pollings
	Radio.Rx(1,1); // dung de test
}
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
	uint8_t data[10];
	memcpy(data,payload,10);
	DEBUG_INFO("Message Comes\r\nSize:%d\r\nRSSI:%d\r\nSNR:%d",size,rssi,snr);
	if(strncmp(payload,"Hello",5) == 0)
	{
		test_buzz = 1;
	}
	/*if(LoRa_BufferHandle(payload) == E_NOT_OK)
	{
		Radio.Rx(1,1);
	}
	else
	{
		Radio.Standby();
	}*/
	Radio.Rx(1,1);
}




void OnRxTimeout( void )
{
	DEBUG_WARN("Rx Timeout\r\n");
	Radio.Rx(10000,0);
}
void OnRxError( void )
{
}


void Send_Check()
{
	Radio.Send("Hello",5);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
}


void App_TestMain()
{
	User_Uart_Init(&User_Uart_ConfigPtr);
	Button btn7;

	RadioEvents_t RadioEvents = {
		.TxDone = OnTxDone,
		.RxDone = OnRxDone,
		.TxTimeout = NULL,
		.RxTimeout = OnRxTimeout,
		.RxError = OnRxError,
		.CadDone = OnCaDDetectted
	};
	//Radio.Standby();
	Radio.Init( &RadioEvents );
	Radio.SetChannel( RF_FREQUENCY );
	Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
	Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, false );
	SX126xSetCadParams(0x02, 23, 10, 0, 00);
	HAL_ADC_Start_IT(&hadc1);
	HAL_TIM_Base_Start_IT(&htim6);	// Slotted timer
	HAL_TIM_Base_Start_IT(&htim2); //  LoRa Irq Scan
	button_init(&btn7,GPIOA,GPIO_PIN_2,BUTTON_ACTIVE_LOW,7);
	Button_AddEvent(7, BUTTON_PRESSED_EVENT, Send_Check);
	Radio.Standby();
	L1_Init();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	Radio.Rx(1,1);
	while(1)
	{

		//LoRa_Polling();
		button_check(&btn7);
		if(test_buzz == 1)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
			test_buzz = 0;
		}
	}
}






void LoRa_IRQHandle()
{
	Radio.IrqProcess( );
}

void Session_Response()
{
	while(1)
	{
		if(SessionID != SESSION_RESPONSE)
		{
			break;
		}

	HAL_Delay(1);
	}
}


void Session_Default()
{
	Radio.Rx(1000,1);
	while(1)
	{
		if(SessionID != SESSION_DEFAULT)
		{
			break;
		}

		HAL_Delay(1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if(htim->Instance == TIM2)
  	  Radio.IrqProcess();
  if(htim->Instance == TIM6)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
		if(slot.synchro_enable == false)
		{
			slot.timer++;
		}
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if(hadc->Instance == ADC1)
  {
		  uint16_t temp = HAL_ADC_GetValue(&hadc1);
		  slot.rnd_num = ((( temp & 0x0003) ^ (SysTick->VAL))&(0x0F));
  }
}



