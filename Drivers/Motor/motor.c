/*
 * motor.c
 *
 *  Created on: Dec 5, 2022
 *      Author: cheik
 */

#include "motor.h"


extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;


uint8_t avance_r(uint16_t alpha)// alpha de 0 à 1023
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
uint8_t recule_r(uint16_t alpha)// alpha de 0 à 1023
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


uint8_t turn_motor(motors_t * motors,uint16_t alpha, Motor_direction direction)
{
	htim15.Instance->CCR1=RESOLUTION*alpha/100;
	if (direction==Forward)
	{

		HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
	}


	return 1;
}
uint8_t stop_motor()
{

	HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);


	return 1;
}



