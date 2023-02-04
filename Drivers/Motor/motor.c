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

int32_t left_dist=0;

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
	encoders->left.consigne_distance=0;
	encoders->right.consigne_distance=0;
	encoders->left.consigne_angle=0;
	encoders->right.consigne_angle=0;
	encoders->angle=0; // angle initiale du robot dans le repère de la table
	encoders->distance=0;
	encoders->theta=0;
	encoders->x=200; // position initiale du robot dans le repère de la table
	encoders->y=300; // position initiale du robot dans le repère de la table
	encoders->ddelta=0;
	encoders->dalpha=0;
	encoders->dl=0;
	encoders->dr=0;
	encoders->right.nbr_ticks_odom=0;
	encoders->left.nbr_ticks_odom=0;
	encoders->distance_done=1;
	encoders->angle_done=1;
	encoders->sem_angle_done=xSemaphoreCreateBinary();
	if (encoders->sem_angle_done == NULL)
	{
		printf("Error semaphore angle\r\n");
		while(1);
	}
	encoders->sem_distance_done=xSemaphoreCreateBinary();
	if (encoders->sem_distance_done == NULL)
	{
		printf("Error semaphore distance\r\n");
		while(1);
	}
	encoders->sem_angle_check=xSemaphoreCreateBinary();
	if (encoders->sem_angle_check == NULL)
	{
		printf("Error semaphore angle check\r\n");
		while(1);
	}
	encoders->sem_distance_check=xSemaphoreCreateBinary();
	if (encoders->sem_distance_check == NULL)
	{
		printf("Error semaphore distance check\r\n");
		while(1);
	}

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
	encoders->left.nbr_ticks+=(int16_t)(-__HAL_TIM_GET_COUNTER(&htim3));
	encoders->right.nbr_ticks+=(int16_t)(__HAL_TIM_GET_COUNTER(&htim1));

	htim1.Instance->CNT=0; // garder le cnt et faire le calcul de la différence
	htim3.Instance->CNT=0;
	// change 10*TS_TO_MIN/(ENC_RESOLUTION) to a constant

	encoders->left.speed=(encoders->left.nbr_ticks-encoders->left.nbr_ticks_old)*10*TS_TO_MIN/(ENC_RESOLUTION);
	encoders->right.speed=(encoders->right.nbr_ticks-encoders->right.nbr_ticks_old)*10*TS_TO_MIN/(ENC_RESOLUTION);

	encoders->left.nbr_ticks_old=encoders->left.nbr_ticks;
	encoders->right.nbr_ticks_old=encoders->right.nbr_ticks;
	// séparer les calculs d'odométrie




	encoders->left.error=encoders->left.consigne-encoders->left.speed;
	encoders->right.error=encoders->right.consigne-encoders->right.speed;

	if (abs(encoders->left.error)<700)
		encoders->left.sum_erreur+=encoders->left.error;
	if (abs(encoders->right.error)<700)
		encoders->right.sum_erreur+=(encoders->right.error);

	if (encoders->left.consigne==0)
		encoders->left.sum_erreur=0;
	if (encoders->right.consigne==0)
		encoders->right.sum_erreur=0;

	encoders->left.new_command=(((encoders->left.error*KP)+(encoders->left.sum_erreur*KI))/10000);
	encoders->right.new_command=(((encoders->right.error*KP)+(encoders->right.sum_erreur*KI))/10000);



	if (encoders->left.new_command<0)
	{
		if (encoders->left.new_command<-853)
			encoders->left.new_command=-853;
		if (encoders->left.new_command>-1)
			encoders->left.new_command=-1;
	}
	else
	{
		if (encoders->left.new_command>853)
			encoders->left.new_command=853;
		if (encoders->left.new_command<1)
			encoders->left.new_command=1;
	}
	if (encoders->right.new_command<0)
	{
		if (encoders->right.new_command<-853)
			encoders->right.new_command=-853;
		if (encoders->right.new_command>-1)
			encoders->right.new_command=-1;
	}
	else
	{
		if (encoders->right.new_command>853)
			encoders->right.new_command=853;
		if (encoders->right.new_command<1)
			encoders->right.new_command=1;
	}


	if (encoders->left.consigne==0)
		encoders->left.new_command=0;
	if (encoders->right.consigne==0)
		encoders->right.new_command=0;

	if (encoders->left.old_command<abs(encoders->left.new_command))
		encoders->left.old_command+=16;//16
	else encoders->left.old_command=abs(encoders->left.new_command);
	if (encoders->right.old_command<abs(encoders->right.new_command))
		encoders->right.old_command+=16; //24
	else encoders->right.old_command=abs(encoders->right.new_command);

	//	encoders->left.old_command=abs(encoders->left.new_command);
	//	encoders->right.old_command=abs(encoders->right.new_command);


	if (encoders->right.new_command>0)
		avance_r((uint16_t)(encoders->right.old_command));
	else recule_r((uint16_t)(encoders->right.old_command));

	if (encoders->left.new_command>0)
		avance_l(encoders->left.old_command);
	else recule_l(encoders->left.old_command);








	return 1;
}

uint8_t command_cartesien(int32_t x_dest,int32_t y_dest,encoders_t * encoders)
{
	int32_t distance=0;
	double angle=0;
	distance=sqrt(((x_dest-encoders->x)*(x_dest-encoders->x))+((y_dest-encoders->y)*(y_dest-encoders->y)));
	if(x_dest-encoders->x==0)
	{
		if (y_dest-encoders->y>0)
			angle=+90;
		else
			angle=-90;
	}
	else if (x_dest-encoders->x>0)
	{
		//if (((encoders->angle%360)>-90)&((encoders->angle%360)<90))
		angle=-encoders->angle+atan(((double)(y_dest-encoders->y)/(x_dest-encoders->x)))*RAD_TO_DEG;

	}

	else if(x_dest-encoders->x<0)
	{
		angle=180-encoders->angle+atan(((double)(y_dest-encoders->y)/(x_dest-encoders->x)))*RAD_TO_DEG;
	}
//	printf("x=%d and y=%d and x_dest=%d and y_dest=%d\n\r",encoders->x,encoders->y,x_dest,y_dest);
//	printf("Dist=%d and angle=%d\n\r",distance,(int)angle);


	command_angle(encoders,angle);
	vTaskDelay(20);
	//while(encoders->angle_done==0); // semaphore or notification
	command_distance(encoders,distance);
	vTaskDelay(20);
	//while(encoders->distance_done==0);
}

uint8_t odom(encoders_t * encoders)
{
	encoders->dr=(encoders->right.nbr_ticks-encoders->right.nbr_ticks_odom)*TICKS_TO_MM;
	encoders->dl=(encoders->left.nbr_ticks-encoders->left.nbr_ticks_odom)*TICKS_TO_MM;
	encoders->dalpha=atan(((double)(encoders->dr-encoders->dl)/ENTRAXE))*RAD_TO_DEG*0.95;
	encoders->ddelta=(encoders->dr+encoders->dl)/2;

	encoders->angle+=encoders->dalpha;

	// Polaire
	encoders->distance+=encoders->ddelta;
	encoders->theta+=encoders->dalpha;

	// Cartésien
	encoders->x+=encoders->ddelta*cos(encoders->angle/RAD_TO_DEG);
	encoders->y+=encoders->ddelta*sin(encoders->angle/RAD_TO_DEG);





	//	if(abs(encoders->left.consigne_angle-encoders->theta)<5) // à changer avec distance != consigne_distance
	//	{
	//		if(abs(encoders->left.consigne_distance-encoders->distance)<10)
	//		{
	//			encoders->left.consigne_distance=0;
	//			encoders->left.consigne=0;
	//			encoders->right.consigne=0;
	//			encoders->distance=0;
	//			encoders->done=1;
	//			 // semaphore or notification
	//
	//		}
	//	}
	//	if(abs(encoders->left.consigne_distance-encoders->distance)<10)
	//	{
	//		if(abs(encoders->left.consigne_angle-encoders->theta)<5) //encoders->angle est déjà en degré (*180/M_PI) à supprimer
	//		{
	//			encoders->left.consigne_angle=0;
	//			encoders->left.consigne=0;
	//			encoders->right.consigne=0;
	//			encoders->theta=0;
	//			encoders->done=1;
	//			 // semaphore or notification
	//
	//		}
	//	}


	//
	//	if(encoders->angle_done==1)  // à changer avec distance != consigne_distance
	//	{
	//		if(abs(encoders->distance)>=abs(encoders->left.consigne_distance))
	//		{
	//			encoders->left.consigne_distance=0;
	//			encoders->left.consigne=0;
	//			encoders->right.consigne=0;
	//			encoders->distance=0;
	//			encoders->distance_done=1;
	//			// add printf here
	//			// semaphore or notification
	//
	//		}
	//	}
	//	if(encoders->distance_done==1)
	//	{
	//		if(abs(encoders->theta)>=abs(encoders->left.consigne_angle)) //encoders->angle est déjà en degré (*180/M_PI) à supprimer
	//		{
	//			encoders->left.consigne_angle=0;
	//			encoders->left.consigne=0;
	//			encoders->right.consigne=0;
	//			encoders->theta=0;
	//			encoders->angle_done=1;
	//			// semaphore or notification
	//
	//		}
	//	}

	//printf("angle=%d and angle_dest=%d\n\r",(int)encoders->theta,(int)encoders->left.consigne_angle);
	//printf("dist=%d and dist_dest=%d\n\r",(int)encoders->distance,(int)encoders->left.consigne_distance);
	encoders->left.nbr_ticks_odom=encoders->left.nbr_ticks;
	encoders->right.nbr_ticks_odom=encoders->right.nbr_ticks;


	return 0;
}
uint8_t command_distance(encoders_t * encoders, int32_t distance)
{
	if (encoders->distance_done==1)
	{

	encoders->left.consigne_distance=distance;
	if (distance >0)
	{
		encoders->left.consigne=50;
		encoders->right.consigne=50;
	}
	else
	{
		encoders->left.consigne=-50;
		encoders->right.consigne=-50;
	}
	//encoders->distance_done=0;
	xSemaphoreGive(encoders->sem_distance_check);
	xSemaphoreTake(encoders->sem_distance_done, portMAX_DELAY);
	//xSemaphoreGive(encoders->sem_distance_done);
	}

	return 0;
}
uint8_t command_angle(encoders_t * encoders,double angle)
{
	if (encoders->angle_done==1)
	{
		encoders->left.consigne_angle=angle;
		if (angle<encoders->theta)
		{
			encoders->left.consigne=50;
			encoders->right.consigne=-50;
		}
		else
		{
			encoders->left.consigne=-50;
			encoders->right.consigne=50;
		}
		//encoders->angle_done=0;
		xSemaphoreGive(encoders->sem_angle_check);
		xSemaphoreTake(encoders->sem_angle_done, portMAX_DELAY);
	}

	return 0;
}
uint8_t command_angle_stop(encoders_t * encoders)
{
	encoders->left.consigne_angle=0;
	encoders->left.consigne=0;
	encoders->right.consigne=0;
	encoders->theta=0;
	encoders->angle_done=1;
	return 0;
}
uint8_t command_distance_stop(encoders_t * encoders)
{
	encoders->left.consigne_distance=0;
	encoders->left.consigne=0;
	encoders->right.consigne=0;
	encoders->distance=0;
	encoders->distance_done=1;
	return 0;
}
uint8_t command_stop(encoders_t * encoders)
{
	encoders->left.consigne_distance=0;
	encoders->left.consigne=0;
	encoders->right.consigne=0;
	encoders->distance=0;
	encoders->distance_done=1;
	encoders->left.consigne_angle=0;
	encoders->theta=0;
	encoders->angle_done=1;
	//	vTaskDelay(100);
	//	command_distance(encoders,-200);
	//	while(encoders->distance_done==0);
	//	command_angle(encoders,180);
	//	while(encoders->angle_done==0); // semaphore or notification


	return 0;
}





