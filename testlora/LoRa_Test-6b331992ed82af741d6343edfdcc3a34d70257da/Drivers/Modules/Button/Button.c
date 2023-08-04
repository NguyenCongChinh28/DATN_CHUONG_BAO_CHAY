/*
 * Button.c
 *
 *  Created on: Nov 23, 2021
 *      Author: Admin
 */
#include "Button.h"




static Button_Event	ButtonCallBack[NUMBER_BUTTON][NUMBER_EVENT];
void Button_AddEvent(uint8_t button_id,button_event event,void (*func)())
{
	ButtonCallBack[button_id][event] = func;
}
void Button_ResetEvent(uint8_t button_id)
{
	for(uint8_t eventcount = 0; eventcount < NUMBER_EVENT; eventcount++)
	{
		ButtonCallBack[button_id][eventcount] = NULL;
	}
}
/*
 * Config CubeMX for Button GPIO
 */
void button_init(Button *btn,GPIO_TypeDef *port,uint16_t pin,Button_active_t button_active,uint8_t button_id)
{
	btn->last_status = 1;
	btn->button_id = button_id;
	btn->gpio_pin = pin;
	btn->gpio_port = port;
	btn->current_status = 1;
	btn->button_state = BUTTON_READ;
	btn->button_active = button_active;
}
void button_check(Button *button)
{
	button->current_status = HAL_GPIO_ReadPin(button->gpio_port, button->gpio_pin);
	if(button->button_active == BUTTON_ACTIVE_HIGH)
	{
		button->current_status = !button->current_status;
	}
	switch(button->button_state)
	{
		case BUTTON_READ:
		{
			if(button->current_status == 0 && button->last_status == 1)
			{
				button->time_debounce = HAL_GetTick();
				button->button_state = BUTTON_WAIT_DEBOUND;
			}
		}
		break;
		case BUTTON_WAIT_DEBOUND:
		{
			if( (HAL_GetTick() - button->time_debounce) > BUTTON_DEBOUND_WAIT)
			{
				if(button->current_status == 0 && button->last_status == 1) // Button is pressed
				{
					if( ButtonCallBack[button->button_id][BUTTON_PRESSED_EVENT] != NULL)
					{
						ButtonCallBack[button->button_id][BUTTON_PRESSED_EVENT]();
					}

					button->t_long_press = HAL_GetTick();
					button->button_state = BUTTON_WAIT_RELEASE_AND_CHECK_LONG_PRESS;
					button->last_status = 0; //
				}
				else if(button->current_status == 1 && button->last_status == 0)
				{
					button->t_long_press = HAL_GetTick() - button->t_long_press;
					if(button->t_long_press >= BUTTON_SHORTHPRESS)
					{
						if( ButtonCallBack[button->button_id][BUTTON_SHORTPRESSED_EVENT] != NULL)
						{
							ButtonCallBack[button->button_id][BUTTON_SHORTPRESSED_EVENT]();
						}
						//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
					}
					button->last_status = 1;
					button->button_state = BUTTON_READ;
				}
				else
				{
					button->last_status = 1;
					button->button_state = BUTTON_READ;
				}
			}
		}
		break;
		case BUTTON_WAIT_RELEASE_AND_CHECK_LONG_PRESS:
		{
			if(button->current_status == 1 && button->last_status == 0)// No pressing
			{
				//button->last_status = 1;
				button->button_state = BUTTON_WAIT_DEBOUND;
			}
			else if((HAL_GetTick() - button->t_long_press) > BUTTON_LONGPRESS)
			{
				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
				if( ButtonCallBack[button->button_id][BUTTON_LONGPRESSED_EVENT] != NULL)
				{
					ButtonCallBack[button->button_id][BUTTON_LONGPRESSED_EVENT]();
				}
				button->last_status = 0;
				button->button_state = BUTTON_WAIT_RELEASE;
			}
		}
		break;
		case BUTTON_WAIT_RELEASE:
		{
			if(button->current_status == 1 && button->last_status == 0)
			{

				button->button_state = BUTTON_READ;
				button->last_status = 1;
				button->time_debounce = HAL_GetTick();
			}
		}
		break;
		default :
		break;
	}
}






