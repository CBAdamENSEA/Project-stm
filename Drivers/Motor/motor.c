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
	encoders->left.nbr_ticks=0;
	encoders->left.error=0;
	encoders->right.error=0;
	encoders->left.sum_erreur=0;
	encoders->right.sum_erreur=0;
	encoders->left.old_command=0;
	encoders->right.old_command=0;
	encoders->left.new_command=0;
	encoders->right.new_command=0;
	encoders->left.consigne=0;
	encoders->right.consigne=0;
	encoders->left.speed=0;
	encoders->right.speed=0;
	encoders->left.distance=0;
	encoders->right.distance=0;

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
	encoders->left.nbr_ticks=-__HAL_TIM_GET_COUNTER(&htim3);
	encoders->right.nbr_ticks=__HAL_TIM_GET_COUNTER(&htim1);
	htim1.Instance->CNT=0;
	htim3.Instance->CNT=0;

	encoders->left.speed=encoders->left.nbr_ticks*10*TS_TO_MIN/(ENC_RESOLUTION);
	encoders->right.speed=encoders->right.nbr_ticks*10*TS_TO_MIN/(ENC_RESOLUTION);

	encoders->left.distance+=encoders->left.nbr_ticks*10*ROUE/(ENC_RESOLUTION);
	encoders->right.distance+=encoders->right.nbr_ticks*10*ROUE/(ENC_RESOLUTION);



	encoders->left.error=encoders->left.consigne-encoders->left.speed;
	encoders->right.error=encoders->right.consigne-encoders->right.speed;

	if (abs(encoders->left.error)<700)
		encoders->left.sum_erreur+=encoders->left.error;
	if (abs(encoders->right.error)<700)
		encoders->right.sum_erreur+=encoders->right.error;

	encoders->left.new_command=(KP*encoders->left.error)+(encoders->left.sum_erreur*KI);
	encoders->right.new_command=(KP*encoders->left.error)+(encoders->left.sum_erreur*KI);

	if (encoders->left.new_command<-853)
		encoders->left.new_command=-853;
	if (encoders->left.new_command>853)
		encoders->left.new_command=853;
	if (encoders->right.new_command<-853)
		encoders->right.new_command=-853;
	if (encoders->right.new_command>853)
		encoders->right.new_command=853;

	if (encoders->left.old_command<abs(encoders->left.new_command))
		encoders->left.old_command+=16;
	else encoders->left.old_command=abs(encoders->left.new_command);
	if (encoders->right.old_command<abs(encoders->right.new_command))
		encoders->right.old_command+=16;
	else encoders->right.old_command=abs(encoders->right.new_command);


	if (encoders->right.new_command>0)
		avance_r((uint16_t)(encoders->right.old_command));
	else recule_r((uint16_t)(encoders->right.old_command));

	if (encoders->left.new_command>0)
		avance_l(encoders->left.old_command);
	else recule_l(encoders->left.old_command);








	return 1;
}






