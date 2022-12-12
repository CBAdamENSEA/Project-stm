/*
 * motor.c
 *
 *  Created on: Dec 5, 2022
 *      Author: cheik
 */

#include "motor.h"


extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;


uint8_t recule_r(uint16_t alpha)// alpha de 0 à 1023
{
	htim15.Instance->CCR1=alpha;
	if(HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1)==HAL_OK)
	{
		if(HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1)==HAL_OK)
		{
			return 1;
		}
		else
			return 0;
	}
	return 0;
}
uint8_t avance_r(uint16_t alpha)// alpha de 0 à 1023
{
	htim15.Instance->CCR1=alpha;
	if(HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1)==HAL_OK)
	{
		if(HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1)==HAL_OK)
		{
			return 1;
		}
		else
			return 0;
	}
	return 0;
}
uint8_t stop_r()// alpha de 0 à 1023
{
	if (HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1)==HAL_OK)
	{
		if (HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1)==HAL_OK)
		{
			return 1;
		}
		else
			return 0;
	}
	return 0;
}
uint8_t avance_l(uint16_t alpha)// alpha de 0 à 1023
{
	htim16.Instance->CCR1=alpha;
	if(HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1)==HAL_OK)
	{
		if(HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1)==HAL_OK)
		{
			return 1;
		}
		else
			return 0;
	}
	return 0;
}
uint8_t recule_l(uint16_t alpha)// alpha de 0 à 1023
{
	htim16.Instance->CCR1=alpha;
	if(HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1)==HAL_OK)
	{
		if(HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1)==HAL_OK)
		{
			return 1;
		}
		else
			return 0;
	}
	return 0;
}
uint8_t stop_l()// alpha de 0 à 1023
{
	if (HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1)==HAL_OK)
	{
		if (HAL_TIMEx_PWMN_Stop(&htim16, TIM_CHANNEL_1)==HAL_OK)
		{
			return 1;
		}
		else
			return 0;
	}
	return 0;
}

uint8_t init_motors(motors_t * motors)
{
	motors->right.h_tim=htim15;
	motors->left.h_tim=htim16;
	motors->left.position=0;
	motors->right.position=0;
	motors->left.speed=0;
	motors->right.speed=0;
	motors->left.drv_motor.drv_avance=avance_l;
	motors->left.drv_motor.drv_recule=recule_l;
	motors->left.drv_motor.drv_stop=stop_l;
	motors->right.drv_motor.drv_avance=avance_r;
	motors->right.drv_motor.drv_recule=recule_r;
	motors->right.drv_motor.drv_stop=stop_r;
	return 1;
}

uint8_t init_encoders(encoders_t * encoders)
{
	encoders->left.nbr_ticks=0;
	encoders->right.nbr_ticks=0;
	if(HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL)!=HAL_OK)
	{
		printf("Right encoder did not start\r\n");
	}
	if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL)!=HAL_OK)
	{
		printf("Left encoder did not start\r\n");
	}
	return 1;
}

uint8_t get_ticks(encoders_t * encoders)
{
	encoders->left.nbr_ticks=__HAL_TIM_GET_COUNTER(&htim1);
	encoders->right.nbr_ticks=__HAL_TIM_GET_COUNTER(&htim3);
	htim1.Instance->CNT=0;
	htim3.Instance->CNT=0;
	return 1;
}






