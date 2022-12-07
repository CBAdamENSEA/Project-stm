/*
 * motor.h
 *
 *  Created on: Dec 5, 2022
 *      Author: cheik
 */

#ifndef MOTOR_MOTOR_H_
#define MOTOR_MOTOR_H_

#include <stdio.h>
#include <stdint.h>
#include "tim.h"

#define RESOLUTION 1023

typedef uint8_t (* drv_avance_t)(uint16_t alpha);// pointeur sur fonction
typedef uint8_t (* drv_recule_t)(uint16_t alpha);
typedef uint8_t (* drv_stop_t)(void);

typedef struct drv_motor_struct
{
	drv_avance_t drv_avance;
	drv_recule_t drv_recule;
	drv_stop_t drv_stop;
} drv_motor_t;

typedef enum
{
	Backward = 0,
	Forward = 1,
} Motor_direction;

typedef struct{

	uint16_t position;
	uint16_t speed;
	uint16_t alpha;
	TIM_HandleTypeDef h_tim;
	drv_motor_t drv_motor;


} motor_t;

typedef struct{

	motor_t right;
	motor_t left;

} motors_t;



uint8_t turn_motor(motors_t * motors,uint16_t alpha, Motor_direction direction);
uint8_t stop_motor();




#endif /* MOTOR_MOTOR_H_ */
