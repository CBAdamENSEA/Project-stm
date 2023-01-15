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
#include <stdlib.h>
#include "math.h"
#include "tim.h"


#define RESOLUTION 1023
#define KI 20
#define KP 20000
#define ENC_RESOLUTION (3412*4)
#define TS_TO_MIN 12000
#define DIAMETER 46
#define ROUE 144
#define ENTRAXE 167
#define MM_TO_TICK (ENC_RESOLUTION/(10*ROUE))
#define TICKS_TO_MM (ROUE*10.0/ENC_RESOLUTION)
#define RAD_TO_DEG (180/M_PI)
#define DEG_TO_TICK ((M_PI*ENTRAXE/360)*MM_TO_TICK)


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

typedef struct{

	int32_t nbr_ticks;
	int32_t nbr_ticks_old;
	int32_t nbr_ticks_odom;

	int32_t speed;
	int32_t distance;
	int32_t error;
	int32_t sum_erreur;
	int32_t delta_erreur; // à voir
	int32_t new_command;
	int32_t old_command;
	int32_t consigne;
	int32_t consigne_distance;
	double consigne_angle;


} encoder_t;

typedef struct{

	encoder_t right;
	encoder_t left;
	int32_t distance;
	double angle;
	double theta;
	int32_t dr;
	int32_t dl;
	double dalpha;
	int32_t ddelta;
	int32_t x;
	int32_t y;
	uint8_t distance_done;
	uint8_t angle_done;


} encoders_t;

uint8_t avance_r(uint16_t alpha);// alpha de 0 à 1023
uint8_t recule_r(uint16_t alpha);
uint8_t stop_r();
uint8_t avance_l(uint16_t alpha);
uint8_t recule_l(uint16_t alpha);
uint8_t stop_l();
uint8_t init_motors(motors_t * motors);
uint8_t init_encoders(encoders_t * encoders);
uint8_t get_ticks(encoders_t * encoders);
uint8_t command_cartesien(int32_t x_dest,int32_t y_dest,encoders_t * encoders);
uint8_t odom(encoders_t * encoders);
uint8_t command_angle(encoders_t * encoders,double angle);
uint8_t command_distance(encoders_t * encoders, int32_t distance);
uint8_t command_angle_stop(encoders_t * encoders);
uint8_t command_distance_stop(encoders_t * encoders);
uint8_t command_stop(encoders_t * encoders);




#endif /* MOTOR_MOTOR_H_ */
