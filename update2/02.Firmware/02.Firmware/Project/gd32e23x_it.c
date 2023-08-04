/*!
    \file    gd32e23x_it.c
    \brief   interrupt service routines
    
    \version 2019-02-19, V1.0.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e23x_it.h"
#include "main.h"
#include "systick.h"
#include "systime.h"
#include "board_hw.h"
#include "app_debug.h"

extern volatile uint32_t m_sys_tick;
extern void RadioOnDioIrq(void);

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    APP_DBG_ERR("Hardfault\r\n");
    board_hw_reset();
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
extern volatile uint32_t delay_remain;
extern volatile bool feed_wdt;
volatile uint32_t poll_line_detect = 0;
extern void board_hw_internal_poll(void);
uint32_t fake_ac_line_detect = 0;
extern volatile uint32_t timeout_not_received_msg_reset_mcu;
extern volatile int32_t timeout_not_received_msg_reset_lora;
void SysTick_Handler(void)
{
    m_sys_tick++;
    systime_set_value(m_sys_tick);
    if (delay_remain > 0)
    {
        delay_remain--;
        feed_wdt = true;
    }
    else
    {
        feed_wdt = false;
    }
    
    if (poll_line_detect++ >= 10)
    {
//        if (fake_ac_line_detect)
//        {
//            board_hw_reset_contactor_monitor_timeout();
//        }
        poll_line_detect = 0;
        board_hw_internal_poll();
    }
	
	if (timeout_not_received_msg_reset_mcu)
	{
		if (timeout_not_received_msg_reset_mcu-- == 0)
			board_hw_reset();
	}
	
	if (timeout_not_received_msg_reset_lora)
	{
		timeout_not_received_msg_reset_lora--;
	}
}

void Default_Handler(void)
{
    APP_DBG_ERR("Default handle\r\n");
    while (1);
}

void EXTI4_15_IRQHandler(void)
{
    bool err = true;
    if (RESET != exti_interrupt_flag_get(EXTI_12)) // Phase lost interrupt
    {
        board_hw_reset_phase_lost_timeout();
        exti_interrupt_flag_clear(EXTI_12);
        err = false;
    }
    
    if (RESET != exti_interrupt_flag_get(EXTI_11)) // Contactor interrupt
    {
        board_hw_reset_contactor_monitor_timeout();
        exti_interrupt_flag_clear(EXTI_11);
        err = false;
    }
    
    if (RESET != exti_interrupt_flag_get(EXTI_15)) // Radio IRQ
    {
        RadioOnDioIrq();
        exti_interrupt_flag_clear(EXTI_15);
        err = false;
    }
}


void EXTI0_1_IRQHandler(void)
{
    if (RESET != exti_interrupt_flag_get(EXTI_1)) // Phase lost interrupt
    {
        RadioOnDioIrq();
        exti_interrupt_flag_clear(EXTI_1);
    }
    
    if (RESET != exti_interrupt_flag_get(EXTI_0)) // Contactor interrupt
    {
        exti_interrupt_flag_clear(EXTI_0);
    }
}

void USART1_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
    {
        uint8_t rx = usart_data_receive(USART1);
#if DEBUG_UART
        extern bool ringbuffer_insert(uint8_t data);
        ringbuffer_insert(rx);
#endif
    }
}

