/*
 * 	File name: 	User_Ip_Usart.c
 *
 *  Created on: Jul 30, 2021
 *  Author: 	Tran Duc Tue
*/
#include "stm32f1xx_hal.h"
#include "User_Ip_Usart_Register.h"
#include "User_Ip_Usart_Types.h"
#include "User_Ip_Usart.h"


User_Ip_RingBuffer_t RingBufferUsart1Channel;
User_Ip_RingBuffer_t RingBufferUsart2Channel;
User_Ip_RingBuffer_t RingBufferUsart3Channel;



static User_Ip_RingBuffer_t *RingBuffer_Ptr_Arr[USER_IP_USART_NUMBEROFBUFFER] = {&RingBufferUsart1Channel,\
																			  	&RingBufferUsart2Channel,\
																				&RingBufferUsart3Channel};





static User_Ip_Usart_Type *const User_Ip_Usart_BaseAddress[USER_IP_USART_INSTANCE_COUNT] = USER_IP_USART_BASE_PTRS;
static User_UsartIRQn_Type User_Ip_IRQn[USER_IP_USART_INSTANCE_COUNT] = USER_IP_USART_RXTX_IRQS;



static User_Ip_RingBuffer_t *User_Ip_Usart_GetBuffer(uint8_t u8ChannelID);

static Std_ReturnType User_Ip_Usart_SetBaurate(User_Ip_Usart_Type *HwChannel, uint32_t u32SoureClock, uint32_t u32Baudrate)
{
	Std_ReturnType ret = E_NOT_OK;
	uint32_t DIV_Mantissa = 0;
	uint16_t DIV_Fraction = 0;
	if (u32Baudrate < 4800)
	{
		ret = E_NOT_OK;
	}
	else
	{
		DIV_Fraction = (u32SoureClock/u32Baudrate)%16;
		DIV_Mantissa = (u32SoureClock/(u32Baudrate*16));
		HwChannel->BRR |= (DIV_Mantissa << USER_IP_USART_BRR_DIV_MANTISSA_SHIFT) & (USER_IP_USART_BRR_DIV_MANTISSA_MASK);
		HwChannel->BRR |= DIV_Fraction & USER_IP_USART_BRR_DIV_FRACTION_MASK;
		ret = E_OK;
	}
	return ret;
}
static void User_Ip_Usart_Set_ParityMode(User_Ip_Usart_Type *HwChannel, User_Uart_ParityMode_t ParityMode)
{
	if(ParityMode == USER_UART_PARITYMODE_NONE)
	{
		HwChannel->CR1 &= ~USER_IP_USART_CR1_PCE_MASK;
	}
	else
	{
		HwChannel->CR1 |= USER_IP_USART_CR1_PCE_MASK;
		HwChannel->CR1 &= ~USER_IP_USART_CR1_PS_MASK;
		HwChannel->CR1 |= (ParityMode << USER_IP_USART_CR1_PS_SHIFT) & (USER_IP_USART_CR1_PS_MASK);
	}
}
static void User_Ip_Usart_Set_StopBit(User_Ip_Usart_Type *HwChannel,User_Uart_StopBitCount_t StopBitCouunt)
{
	HwChannel->CR2 &= ~USER_IP_USART_CR2_STOP_MASK;
	HwChannel->CR2 |= (StopBitCouunt << USER_IP_USART_CR2_STOP_SHIFT) &USER_IP_USART_CR2_STOP_MASK;
}


void static User_Ip_Usart_DeInit(User_Ip_Usart_Type *HwConfig)
{
	uint16_t u16Count;
	uint8_t  u8Count;
	User_Ip_RingBuffer_t *BufferChannel;

	HwConfig->CR1 = USER_IP_USART_RESET_VALUE;
	HwConfig->CR2 = USER_IP_USART_RESET_VALUE;
	HwConfig->CR3 = USER_IP_USART_RESET_VALUE;
	HwConfig->GTPR = USER_IP_USART_RESET_VALUE;

	for ( u8Count = 0; u8Count < USER_IP_USART_INSTANCE_COUNT; u8Count++)
	{
		BufferChannel = RingBuffer_Ptr_Arr[u8Count];
		BufferChannel->RxHead = 0U;
		BufferChannel->RxTail = 0U;
		BufferChannel->TxHead = 0U;
		BufferChannel->TxTail = 0U;
		BufferChannel->BufferStatus = USER_BUFFER_UNLOCK;
		for( u16Count = 0 ; u16Count < USER_IP_USART_SIZE_RINGBUFFER; u16Count++ )
		{
			BufferChannel->User_Usart_RxRingBufferChannel[u16Count] = 0x00U;
			BufferChannel->User_Usart_TxRingBufferChannel[u16Count] = 0x00U;
		}

	}

}

void User_Ip_Usart_Init(User_Uart_HwConfigType *HwConfig)
{
	uint8_t u8ChannelID = HwConfig->u8ChannelID;
	User_Ip_Usart_Type *Usart = User_Ip_Usart_BaseAddress[u8ChannelID];
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	User_Ip_Usart_DeInit(Usart);

	Usart->CR1 &= ~USER_IP_USART_CR1_TE_MASK;
	Usart->CR1 &= ~USER_IP_USART_CR1_RE_MASK;
	User_Ip_Usart_SetBaurate(Usart,HwConfig->u32SourceClock,HwConfig->u32BaudRate);
	User_Ip_Usart_Set_ParityMode(Usart,HwConfig->ParityMode);
	User_Ip_Usart_Set_StopBit(Usart,HwConfig->StopBitCount);

	Usart->CR3 |= USER_IP_USART_CR1_IDLEIE_MASK;
	Usart->CR1 |= (USER_IP_USART_CR1_TE_MASK|
				   USER_IP_USART_CR1_RE_MASK|
				   USER_IP_USART_CR1_RXNEIE_MASK|
				   USER_IP_USART_CR1_UE_MASK);
	HAL_NVIC_SetPriority(User_Ip_IRQn[u8ChannelID],USER_IP_USART_PRIORITY,0u);
	HAL_NVIC_EnableIRQ(User_Ip_IRQn[u8ChannelID]);
}

static inline void User_Ip_Usart_StartAsyncTransmit(User_Ip_Usart_Type *HwChannel)
{
	HwChannel->CR1 |= USER_IP_USART_CR1_TXEIE_MASK;
}
static inline void User_Ip_Usart_StoptAsyncTransmit(User_Ip_Usart_Type *HwChannel)
{
	HwChannel->CR1 &= ~USER_IP_USART_CR1_TXEIE_MASK;
}





static User_Ip_RingBuffer_t *User_Ip_Usart_GetBuffer(uint8_t u8ChannelID)
{
	User_Ip_RingBuffer_t *RingBuffer;
	if (u8ChannelID == USER_IP_USART_CHANNELID_1)
	{
		RingBuffer = &RingBufferUsart1Channel;
	}
	else if (u8ChannelID == USER_IP_USART_CHANNELID_2)
	{
		RingBuffer = &RingBufferUsart2Channel;
	}
	else if (u8ChannelID == USER_IP_USART_CHANNELID_3)
	{
		RingBuffer = &RingBufferUsart3Channel;
	}
	else
	{
		RingBuffer = NULL;
	}
	return RingBuffer;
}

Std_ReturnType User_Ip_Usart_WriteBuffer(uint8_t u8ChannelID, const uint8_t *u8Data, uint16_t u16DataLen)
{
	Std_ReturnType ret = E_NOT_OK;
	uint16_t u16DataAvailable = 0;
	uint16_t u16Count;

	User_Ip_RingBuffer_t *BufferChannel;
	BufferChannel = User_Ip_Usart_GetBuffer(u8ChannelID);
	if(BufferChannel == NULL)
	{
		return E_NOT_OK;
	}
	if (u8Data == NULL || u16DataLen == 0)
	{
		return E_NOT_OK;
	}
	if (BufferChannel->BufferStatus ==  USER_BUFFER_UNLOCK)
	{
		if(BufferChannel->TxHead > BufferChannel->TxTail)
		{
			u16DataAvailable = BufferChannel->TxHead - BufferChannel->TxTail;
		}
		else
		{
			u16DataAvailable = ( USER_IP_USART_SIZE_RINGBUFFER - BufferChannel->TxHead) + BufferChannel->TxTail;
		}
		if ( u16DataAvailable > u16DataLen)
		{
			BufferChannel->BufferStatus = USER_BUFFER_LOCK;
			for (u16Count = 0;u16Count < u16DataLen; u16Count++)
			{
				BufferChannel->User_Usart_TxRingBufferChannel[BufferChannel->TxTail] = *u8Data;
				BufferChannel->TxTail = (BufferChannel->TxTail + 1) & (USER_IP_USART_SIZE_RINGBUFFER - 1);
				u8Data++;
			}
			BufferChannel->BufferStatus = USER_BUFFER_UNLOCK;
			User_Ip_Usart_StartAsyncTransmit(User_Ip_Usart_BaseAddress[u8ChannelID]);
			ret = E_OK;
		}
	}
		return ret;
}
static inline void User_Ip_Usart_PutChar(User_Ip_Usart_Type *HwChannel,uint8_t u8Data)
{
	HwChannel->DR = u8Data;
}

static inline void User_Ip_Usart_TransmitBuffer(uint8_t u8ChannelID)
{
	User_Ip_RingBuffer_t *BufferChannel;
	BufferChannel = User_Ip_Usart_GetBuffer(u8ChannelID);
	User_Ip_Usart_Type *HwChannel = User_Ip_Usart_BaseAddress[u8ChannelID];

	if (BufferChannel == NULL)
	{
		return ;
	}

	if (BufferChannel->TxHead != BufferChannel->TxTail )
	{
		User_Ip_Usart_PutChar(HwChannel,BufferChannel->User_Usart_TxRingBufferChannel[BufferChannel->TxHead]);
		BufferChannel->TxHead = (BufferChannel->TxHead + 1)& (USER_IP_USART_SIZE_RINGBUFFER - 1);
	}
	else
	{
		User_Ip_Usart_StoptAsyncTransmit(HwChannel);
	}
}
static inline uint8_t User_Ip_Usart_GetChar(User_Ip_Usart_Type* HwChannel)
{
	return (HwChannel->DR);
}

Std_ReturnType User_Ip_Usart_ReadBuffer(uint8_t u8ChannelID, uint8_t *u8Data, uint16_t u16DataLen,int16_t *i16reamain)
{
	User_Ip_RingBuffer_t *BufferChannel;
	BufferChannel = User_Ip_Usart_GetBuffer(u8ChannelID);
	uint16_t u16Count = 0u;
	if(BufferChannel == NULL)
	{
		return E_NOT_OK;
	}
	if(u8Data == NULL || u16DataLen == 0)
	{
		return E_NOT_OK;
	}
	if(BufferChannel->RxHead == BufferChannel->RxTail)
	{
		return E_NOT_OK;
	}
	for(u16Count = 0 ; u16Count < u16DataLen ; u16Count++ )
	{
		if(BufferChannel->RxTail != BufferChannel->RxHead)
		{
			*u8Data = BufferChannel->User_Usart_RxRingBufferChannel[BufferChannel->RxHead];
			BufferChannel->RxHead = (BufferChannel->RxHead + 1)&(USER_IP_USART_SIZE_RINGBUFFER - 1);
			u8Data++;
		}
		else
		{
			*i16reamain = u16Count - u16DataLen;
			//return E_OK;
			break;
		}
	}
	if( BufferChannel->RxTail > BufferChannel->RxHead )
	{
		*i16reamain = BufferChannel->RxTail - BufferChannel->RxHead;
	}
	else if ( BufferChannel->RxTail < BufferChannel->RxHead )
	{
		*i16reamain = ( USER_IP_USART_SIZE_RINGBUFFER - BufferChannel->RxHead) +  BufferChannel->RxTail;
	}
	return E_OK;
}



static void User_Ip_Usart_ReceiveBuffer(uint8_t u8ChannelUD, uint8_t u8data)
{
	User_Ip_RingBuffer_t *BufferChannel;
	BufferChannel = User_Ip_Usart_GetBuffer(u8ChannelUD);
	if (BufferChannel == NULL)
	{
		return;
	}
	BufferChannel->User_Usart_RxRingBufferChannel[BufferChannel->RxTail] = u8data;
	BufferChannel->RxTail = ( BufferChannel->RxTail + 1 ) & (USER_IP_USART_SIZE_RINGBUFFER -1 );
	if(BufferChannel->RxTail == BufferChannel->RxHead )
	{
		if(BufferChannel->RxHead == 0)
		{
			BufferChannel->RxTail = USER_IP_USART_SIZE_RINGBUFFER - 1;
		}
		else
		{
			BufferChannel->RxTail = (BufferChannel->RxTail  - 1) & (USER_IP_USART_SIZE_RINGBUFFER - 1);
		}
	}
}
































void User_Ip_Usart_IrqHandler(uint8_t u8ChannelID)
{
	uint8_t receive_data = 0;
	User_Ip_Usart_Type *HwChannel = User_Ip_Usart_BaseAddress[u8ChannelID];
	if( (HwChannel->SR & USER_IP_USART_SR_TXE_MASK)  == USER_IP_USART_SR_TXE_MASK)
	{
		User_Ip_Usart_TransmitBuffer(u8ChannelID);
	}
	if( (HwChannel->SR & USER_IP_USART_SR_RXNE_MASK)  == USER_IP_USART_SR_RXNE_MASK)
	{
		receive_data = User_Ip_Usart_GetChar(HwChannel);
		User_Ip_Usart_ReceiveBuffer(u8ChannelID,receive_data);
	}
	if( (HwChannel->SR & USER_IP_USART_SR_ORE_MASK)  == USER_IP_USART_SR_ORE_MASK)
	{
		User_Ip_Usart_GetChar(HwChannel);
	}
	if( (HwChannel->SR & USER_IP_USART_SR_PE_MASK)  == USER_IP_USART_SR_PE_MASK)
	{

	}
	if( (HwChannel->SR & USER_IP_USART_SR_FE_MASK)  == USER_IP_USART_SR_FE_MASK)
	{

	}
}










void USART1_IRQHandler(void)
{
	User_Ip_Usart_IrqHandler(USER_IP_USART_CHANNELID_1);
}
void USART2_IRQHandler(void)
{
	User_Ip_Usart_IrqHandler(USER_IP_USART_CHANNELID_2);
}
void USART3_IRQHandler(void)
{
	User_Ip_Usart_IrqHandler(USER_IP_USART_CHANNELID_3);
}





