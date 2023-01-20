/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "shell.h"
#include "motor.h"
#include <string.h>
#include "TCS3200.h"
#include "bordure.h"
#include "VL53L0X.h"



#include "../../Drivers/Servo/XL_320.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_SHELL_STACK_DEPTH 1024
#define TASK_SHELL_PRIORITY 1
#define TASK_ENCODER_STACK_DEPTH 512
#define TASK_ENCODER_PRIORITY 10
#define TASK_ODOM_STACK_DEPTH 512
#define TASK_ODOM_PRIORITY 4
#define TASK_BORDURE_STACK_DEPTH 64
#define TASK_BORDURE_PRIORITY 8
#define TASK_ANGLE_STACK_DEPTH 256
#define TASK_ANGLE_PRIORITY 4
#define TASK_DISTANCE_STACK_DEPTH 256
#define TASK_DISTANCE_PRIORITY 4
#define TASK_COLOR_STACK_DEPTH 512
#define TASK_COLOR_PRIORITY 3
#define TASK_TOF_STACK_DEPTH 1024
#define TASK_TOF_PRIORITY 1
#define TASK_SEARCHING_STACK_DEPTH 256
#define TASK_SEARCHING_PRIORITY 1
#define ENCODER_TICKS 3412
#define TS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t h_task_shell = NULL;
TaskHandle_t h_task_encoder = NULL;
TaskHandle_t h_task_odom = NULL;
TaskHandle_t h_task_color = NULL;
TaskHandle_t h_task_bordure = NULL;
TaskHandle_t h_task_angle = NULL;
TaskHandle_t h_task_distance = NULL;
TaskHandle_t h_task_tof = NULL;
TaskHandle_t h_task_searching = NULL;

SemaphoreHandle_t sem_searching;




h_shell_t h_shell;
uint8_t buffer[BUFFER_LENGTH] = {0};
uint16_t buffer_index = 0;
uint8_t Receive_length=0;
char Stat_buffer[250];
char msgBuffers[50];
statInfo_t_VL53L0X distanceStr_r;
statInfo_t_VL53L0X distanceStr_l;
tofs_t tofs;
servo_t servo;
motors_t motors;
uint32_t cnt_encoder1;
uint32_t cnt_encoder2;
uint16_t distance_droit;
uint16_t distance_gauche;


encoders_t encoders;
color_sensor_t color_sensor;
bords_t bords;

int tof_count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int fonction(h_shell_t * h_shell, int argc, char ** argv)
{
	printf("Je suis une fonction bidon\r\n");

	printf("argc = %d\r\n", argc);

	for (int i = 0 ; i < argc ; i++)
	{
		printf("arg %d = %s\r\n", i, argv[i]);
	}

	return 0;
}


int motor(h_shell_t * h_shell,int argc, char ** argv)
{
	if (argc == 4)
	{
		uint16_t alpha=atoi(argv[3]);

		if(strncmp(argv[1],"r",1)==0)
		{
			if(strncmp(argv[2],"f",1)==0)
			{
				motors.right.drv_motor.drv_avance(alpha);
			}
			else if(strncmp(argv[2],"b",1)==0)
			{
				motors.right.drv_motor.drv_recule(alpha);
			}
			else
			{
				motors.right.drv_motor.drv_stop();
			}
		}
		if(strncmp(argv[1],"l",1)==0)
		{
			if(strncmp(argv[2],"f",1)==0)
			{
				motors.left.drv_motor.drv_avance(alpha);
			}
			else if(strncmp(argv[2],"b",1)==0)
			{
				motors.left.drv_motor.drv_recule(alpha);
			}
			else
			{
				motors.left.drv_motor.drv_stop();
			}
		}
		if(strncmp(argv[1],"b",1)==0)
		{
			encoders.left.consigne=atoi(argv[3]);
			encoders.right.consigne=encoders.left.consigne;

			//			if(strncmp(argv[2],"f",1)==0)
			//			{
			//				motors.left.drv_motor.drv_avance(alpha);
			//				motors.right.drv_motor.drv_avance(alpha);
			//
			//			}
			//			else if(strncmp(argv[2],"b",1)==0)
			//			{
			//				motors.left.drv_motor.drv_recule(alpha);
			//				motors.right.drv_motor.drv_recule(alpha);
			//			}
			//			else
			//			{
			//				motors.left.drv_motor.drv_stop();
			//				motors.right.drv_motor.drv_stop();
			//			}
		}
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}

int speed(h_shell_t * h_shell,int argc, char ** argv)
{
	if (argc == 1)
	{
		printf("R Ticks = %d\r\n",encoders.right.nbr_ticks);
		printf("L Ticks = %d\r\n",encoders.left.nbr_ticks);

		printf("R speed = %d Rpm\r\n",encoders.right.speed);
		printf("L speed = %d Rpm\r\n",encoders.left.speed);

		//		printf("R distance = %d mm\r\n",encoders.right.distance);
		//		printf("L distance = %d mm\r\n",encoders.left.distance);

		printf("R error = %d \r\n",encoders.right.error);
		printf("L error = %d \r\n",encoders.left.error);

		printf("R command = %d \r\n",encoders.right.new_command);
		printf("L command = %d \r\n",encoders.left.new_command);
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}
int demo(h_shell_t * h_shell,int argc, char ** argv)
{
	if (argc == 1)
	{
		//		uint16_t alpha=150;
		//		uint16_t position=350;
		//		uint16_t final_position=0;
		//		uint16_t error=5;
		//		motors.left.drv_motor.drv_avance(alpha);
		//		motors.right.drv_motor.drv_avance(alpha+12);
		//		vTaskDelay(1000);
		//		printf("R speed = %d Rpm\r\n",encoders.right.speed);
		//		printf("L speed = %d Rpm\r\n",encoders.left.speed);
		//		printf("R distance = %d mm\r\n",encoders.right.distance);
		//		printf("L distance = %d mm\r\n",encoders.left.distance);
		//		vTaskDelay(800);
		//		motors.left.drv_motor.drv_stop();
		//		motors.right.drv_motor.drv_stop();
		//		vTaskDelay(500);
		//		XL_320_set_goal_position(&servo,0x01, final_position);
		//		while (abs(position-final_position)<error)
		//		{
		//			position=XL_320_read_present_position(&servo,0x01);
		//		}
		//		vTaskDelay(4000);
		//		xSemaphoreGive(color_sensor.sem_color_read);
		//		TCS3200_Detected_Color(&color_sensor);
		//		vTaskDelay(4000);
		//		xSemaphoreGive(color_sensor.sem_color_read);
		//		TCS3200_Detected_Color(&color_sensor);
		//		vTaskDelay(1000);
		//
		//		position=0;
		//		final_position=350;
		//		error=5;
		//
		//		XL_320_set_goal_position(&servo, 0x01, final_position);
		//		while (abs(position-final_position)<error)
		//		{
		//			position=XL_320_read_present_position(&servo,0x01);
		//		}
		//		vTaskDelay(2000);
		//		motors.left.drv_motor.drv_recule(alpha);
		//		motors.right.drv_motor.drv_recule(alpha+12);
		//		vTaskDelay(2000);
		//		motors.left.drv_motor.drv_stop();
		//		motors.right.drv_motor.drv_stop();
		//		vTaskDelay(500);
		//		motors.left.drv_motor.drv_avance(alpha);
		//		motors.right.drv_motor.drv_recule(alpha);
		//		vTaskDelay(1150); // Tourner
		//		motors.left.drv_motor.drv_stop();
		//		motors.right.drv_motor.drv_stop();
		//		vTaskDelay(400); // Arrêter
		//
		//
		//		position=350;
		//		final_position=0;
		//		error=5;
		//		XL_320_set_goal_position(&servo,0x01, final_position);
		//		while (abs(position-final_position)<error)
		//		{
		//			position=XL_320_read_present_position(&servo,0x01);
		//		}
		//		vTaskDelay(1000);
		//		motors.left.drv_motor.drv_recule(alpha);
		//		motors.right.drv_motor.drv_recule(alpha+12);
		//		vTaskDelay(800);
		//		motors.left.drv_motor.drv_stop();
		//		motors.right.drv_motor.drv_stop();
		//		position=0;
		//		final_position=350;
		//		error=5;
		//
		//		XL_320_set_goal_position(&servo, 0x01, final_position);
		//		while (abs(position-final_position)<error)
		//		{
		//			position=XL_320_read_present_position(&servo,0x01);
		//		}
		//		vTaskDelay(1000);









		//		int32_t distance=300;
		//		double angle=20;
		//		uint16_t position=350;
		//		uint16_t final_position=0;
		//		uint16_t error=5;
		//
		//		command_angle(&encoders,angle);
		//		while(encoders.done==0);
		//		command_distance(&encoders,distance);
		//		while(encoders.done==0);
		//
		//		XL_320_set_goal_position(&servo,0x01, final_position);
		//		while (abs(position-final_position)<error)
		//		{
		//			position=XL_320_read_present_position(&servo,0x01);
		//		}
		//		vTaskDelay(2000);
		//		distance=200;
		//		command_distance(&encoders,distance);
		//		while(encoders.done==0);
		//
		//		position=0;
		//		final_position=350;
		//		XL_320_set_goal_position(&servo, 0x01, final_position);
		//		while (abs(position-final_position)<error)
		//		{
		//			position=XL_320_read_present_position(&servo,0x01);
		//		}
		//		vTaskDelay(2000);
		//
		//		distance=500;
		//		angle=180;
		//
		//		command_angle(&encoders,angle);
		//		while(encoders.done==0);
		//		command_distance(&encoders,distance);
		//		while(encoders.done==0);
		//
		//		position=350;
		//		final_position=0;
		//		XL_320_set_goal_position(&servo, 0x01, final_position);
		//		while (abs(position-final_position)<error)
		//		{
		//			position=XL_320_read_present_position(&servo,0x01);
		//		}
		//		vTaskDelay(2000);
		//		distance=-200;
		//		command_distance(&encoders,distance);
		//		while(encoders.done==0);

		command_cartesien(200,0,&encoders);
		vTaskDelay(1000);
		command_cartesien(200,200,&encoders);
		vTaskDelay(1000);
		command_cartesien(0,200,&encoders);
		vTaskDelay(1000);
		command_cartesien(0,0,&encoders);
		vTaskDelay(1000);
		return 0;


	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}

int couleurs(h_shell_t * h_shell,int argc, char ** argv)
{
	if (argc == 1)
	{

		xSemaphoreGive(color_sensor.sem_color_read);
		TCS3200_Detected_Color(&color_sensor);
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}

int statistiques(h_shell_t * h_shell,int argc, char ** argv)
{
	if (argc == 1)
	{
		printf("Task            Abs Time        per Time\r\n");
		vTaskGetRunTimeStats(Stat_buffer);
		HAL_UART_Transmit(&huart2, (uint8_t*)Stat_buffer, strlen(Stat_buffer), HAL_MAX_DELAY); //huart3 stlink
		printf("\r\nTask            State   Prior.  Stack   Num\r\n");
		vTaskList(Stat_buffer);
		HAL_UART_Transmit(&huart2, (uint8_t*)Stat_buffer, strlen(Stat_buffer), HAL_MAX_DELAY); //huart3 stlink

		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}

int open_gate(h_shell_t * h_shell,int argc, char ** argv)
{
	uint16_t position=350;
	uint16_t final_position=0;
	uint16_t error=5;
	if (argc == 1)
	{
		XL_320_set_goal_position(&servo,0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(&servo,0x01);
		}
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}

int close_gate(h_shell_t * h_shell,int argc, char ** argv)
{
	uint16_t position=0;
	uint16_t final_position=350;
	uint16_t error=5;
	if (argc == 1)
	{
		XL_320_set_goal_position(&servo, 0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(&servo,0x01);
		}
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}
int test_pwm(h_shell_t * h_shell,int argc, char ** argv)
{
	uint16_t pwm_initial=50;
	uint16_t pwm_final=50;
	uint16_t pwm=pwm_initial;
	if (argc == 2)
	{
		pwm=atoi(argv[1]);
		avance_r(0);
		vTaskDelay(1000);
		avance_r(pwm);
		printf ("pwm=%d\n\r",pwm);


		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}
int test_dist(h_shell_t * h_shell,int argc, char ** argv)
{
	int32_t distance=atoi(argv[1]);
	if (argc == 2)
	{
		command_distance(&encoders,distance);
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}
int test_tof(h_shell_t * h_shell,int argc, char ** argv)
{
	if (argc == 1)
	{
		printf("Distance L= %d\r\n",tofs.left.distance);
		printf("Distance R= %d\r\n",tofs.right.distance);
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}
int test_angle(h_shell_t * h_shell,int argc, char ** argv)
{
	double angle=atoi(argv[1]);
	if (argc == 2)
	{
		command_angle(&encoders,angle);
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}

int test_xy(h_shell_t * h_shell,int argc, char ** argv)
{
	int32_t x=atoi(argv[1]);
	int32_t y=atoi(argv[2]);

	if (argc == 3)
	{
		command_cartesien(x,y,&encoders);
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}


void task_color(void * unused)
{
	vTaskDelay(1000);
	//	if (TCS3200_Init(&color_sensor))
	//	{
	//		printf("Color sensor initialized\r\n");
	//	}
	while(1)
	{
		xSemaphoreTake(color_sensor.sem_color_read, portMAX_DELAY);
		if(TCS3200_Read_Color(&color_sensor,FILTER_RED))
		{
			//printf("red color is : %d\r\n", color_sensor.red);
		}
		//TCS3200_Detected_Color(&color_sensor);
	}
}

void task_shell(void * unused)
{
	shell_init(&h_shell);
	if (init_servo(&servo))
	{
		printf("Servo initialized\r\n");
	}
	if (init_motors(&motors))
	{
		printf("motors initialized\r\n");
	}
	if (TCS3200_Init(&color_sensor))
	{
		printf("Color sensor initialized\r\n");
	}






	//	if (TCS3200_Init(&color_sensor))
	//	{
	//		printf("Color sensor initialized\r\n");
	//	}

	//shell_add(&h_shell,'f', fonction, (char *)"Une fonction inutile");
	shell_add(&h_shell,'s', statistiques, "Afficher les stat");
	//shell_add(&h_shell,'t', test_pwm, "Tester le pwm du moteur right");
	shell_add(&h_shell,'o', open_gate, "ouvrir la porte");
	shell_add(&h_shell,'c', close_gate, "fermer la porte");
	shell_add(&h_shell,'m', motor, "tourner les moteurs");
	shell_add(&h_shell,'e', speed, "vitesse des moteurs");
	shell_add(&h_shell,'r', couleurs, "couleur de la canette");
	shell_add(&h_shell,'d', demo, "Démonstration");
	shell_add(&h_shell,'a', test_dist, "tester asserv dist");
	shell_add(&h_shell,'b', test_angle, "tester asserv angle");
	shell_add(&h_shell,'x', test_xy, "tester asserv xy");
	shell_add(&h_shell,'t', test_tof, "tester les tofs");
	shell_run(&h_shell);	// boucle infinie
}
void task_encoder(void * unused)
{
	if (init_encoders(&encoders))
	{
		printf("Encoders Initialized\r\n");
	}
	while (1)
	{
		get_ticks(&encoders);
		vTaskDelay(5);
	}
}

void task_odom(void * unused)
{

	while(1)
	{
		odom(&encoders);

		vTaskDelay(50);
	}
}
void task_bordure(void * unused)
{
	init_bords(&bords);
//	sem_searching=xSemaphoreCreateBinary();
//	if (sem_searching == NULL)
//	{
//		printf("Error semaphore searching\r\n");
//		while(1);
//	}
	while(1)
	{

		if(update_bords(&bords)&(bords.detect==0))
		{
			command_stop(&encoders);
			xSemaphoreGive(encoders.sem_angle_done);
			xSemaphoreGive(encoders.sem_distance_done);
			bords.detect=1;
			vTaskDelay(10);
		}
		if(bords.detect)
		{
			command_distance(&encoders, -100);
			//command_angle(&encoders, 180);
			bords.detect=0;
		}
		vTaskDelay(20);
	}
}
void task_angle(void * unused)
{
	while(1)
	{
		// Sémaphore
		xSemaphoreTake(encoders.sem_angle_check, portMAX_DELAY);
		if(abs(encoders.theta)>abs(encoders.left.consigne_angle))
		{
			command_angle_stop(&encoders);
			//printf("angle=%d\r\n",(int)encoders.angle);
			xSemaphoreGive(encoders.sem_angle_done);
		}
		else
		{
			xSemaphoreGive(encoders.sem_angle_check);
		}
		vTaskDelay(10);
	}
}
void task_distance(void * unused)
{
	while(1)
	{
		// Sémaphore
		xSemaphoreTake(encoders.sem_distance_check, portMAX_DELAY);
		if(abs(encoders.distance)>abs(encoders.left.consigne_distance))
		{
			command_distance_stop(&encoders);
			xSemaphoreGive(encoders.sem_distance_done);
		}
		else
		{
			xSemaphoreGive(encoders.sem_distance_check);
		}

		vTaskDelay(10);
	}
}
void task_tof(void *unused)
{

	int left=0;
	int right=0;
	while(1)
	{
		tofs.left.distance=tofs.left.drv_tof.readRangeSingleMillimeters(&distanceStr_l,TOF_LEFT);
		tofs.right.distance=tofs.right.drv_tof.readRangeSingleMillimeters(&distanceStr_r,TOF_RIGHT);
		tof_count++;

		if ((tofs.left.distance>8000)&(tofs.right.distance<8000))
			tofs.distance=tofs.right.distance;
		else if ((tofs.left.distance<8000)&(tofs.right.distance>8000))
			tofs.distance=tofs.left.distance;
		else
			tofs.distance=(tofs.left.distance+tofs.right.distance)/2;
		xTaskNotifyGive(h_task_searching);
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );



		//printf("Distance L= %d\r\n",tofs.left.distance);
		//printf("Distance R= %d\r\n",tofs.right.distance);
		vTaskDelay(10);
	}
}
void task_searching(void * unused)
{
	double pas=5.0;
	int angle=0;
	int cmpt=0;
	int on=1;
	int on_stop=1;
	uint16_t position=350;
	uint16_t final_position=0;
	uint16_t error=5;
	uint8_t detected=0;
	uint8_t approche=0;
	uint8_t positioning=0;
	uint8_t positioning2=0;
	uint8_t open=0;

	while(1)
	{
		//xSemaphoreTake(sem_searching, portMAX_DELAY);
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		if (tofs.distance==0)
		{

		}
		else if ((tofs.distance>500)&(detected==0))
		{
			command_angle(&encoders,pas);
		}
		else if ((tofs.distance<500)&(detected==0))
		{
			detected=1;

		}
		//		else if ((detected)&(approche==0))
		//		{
		//			command_distance(&encoders,tofs.distance-300);
		//			approche=1;
		//		}
		else if ((detected)&(positioning==0))
		{
			if (abs(tofs.left.distance-tofs.right.distance)<150)
				positioning=1;
//			else
//				//command_angle(&encoders,-pas);
//			else if (tofs.left.distance>tofs.right.distance)
//				command_angle(&encoders,-2.0);
//			else if (tofs.left.distance>tofs.right.distance)
//				command_angle(&encoders,2.0);
		}
		else if ((positioning==1)&(approche==0))
		{
			command_distance(&encoders,tofs.distance-200);
			approche=1;
		}
		else if ((approche)&(open==0))
		{
			XL_320_set_goal_position(&servo,0x01, final_position);
			while (abs(position-final_position)<error)
			{
				position=XL_320_read_present_position(&servo,0x01);
			}
			open=1;
		}
		else if ((open)&(positioning2==0))
		{
			if (abs(tofs.left.distance-tofs.right.distance)<20)
				positioning2=1;
			else if (tofs.left.distance>tofs.right.distance)
				command_angle(&encoders,-pas);
			else if (tofs.left.distance>tofs.right.distance)
				command_angle(&encoders,pas);
		}

		printf("left=%d , right=%d, distance=%d\r\n",tofs.left.distance,tofs.right.distance,tofs.distance);
		//vTaskDelay(500);
		xTaskNotifyGive( h_task_tof );

	}
}




/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	h_shell.shell_func_list_size=0;
	h_shell.sem_uart_read=NULL;
	servo.sem_packet_read=NULL;
	color_sensor.sem_color_read=NULL;
	encoders.sem_angle_done=NULL;
	encoders.sem_distance_done=NULL;
	encoders.sem_angle_check=NULL;
	encoders.sem_distance_check=NULL;
	sem_searching=NULL;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM15_Init();
	MX_USART3_UART_Init();
	MX_ADC1_Init();
	MX_TIM17_Init();
	MX_TIM7_Init();
	MX_TIM16_Init();
	/* USER CODE BEGIN 2 */

	HAL_Delay(5000);
	if (initVXL_tofs(&tofs))
	{
		printf("tofs Initialized\r\n");
	}

	//	char msg_blue[50];
	//	sprintf(msg_blue,"Hello\r\n");
	//	while (1)
	//	{
	//		HAL_GPIO_WritePin(BAT_LED_GPIO_Port, BAT_LED_Pin, GPIO_PIN_SET);
	//		if (HAL_UART_Transmit(&huart2, (uint8_t *)msg_blue, sizeof(msg_blue), 0xFFFF)==HAL_OK)
	//		{
	//
	//			HAL_GPIO_WritePin(BAT_LED_GPIO_Port, BAT_LED_Pin, GPIO_PIN_RESET);
	//
	//		}
	//		HAL_Delay(1000);
	//	}



	//	uint16_t model_number=0;
	//	uint8_t firmware_version=0;

	//  htim15.Instance->CCR1=512;
	//  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	//  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);



	//	if(XL_320_ping(0x01,&model_number,&firmware_version)==1)
	//	{
	//		printf("model_number= 0x%04x \r\n", model_number);
	//		printf("firmware_version= 0x%04x \r\n", firmware_version);
	//	}
	//	if (XL_320_set_torque_enable(0x01, 1))
	//	{
	//		printf("torque enabled\r\n");
	//	}
	//HAL_Delay(100);
	//	if (XL_320_set_speed_position(0x01, 100))
	//	{
	//		printf ("speed position = 100\r\n");
	//	}
	//HAL_Delay(100);
	//printf("Creating task shell\r\n");



	if (xTaskCreate(task_shell, "Shell", TASK_SHELL_STACK_DEPTH, NULL, TASK_SHELL_PRIORITY, &h_task_shell) != pdPASS)
	{
		printf("Error creating task shell\r\n");
		Error_Handler();
	}
	if (xTaskCreate(task_odom, "odom", TASK_ODOM_STACK_DEPTH, NULL, TASK_ODOM_PRIORITY, &h_task_odom) != pdPASS)
	{
		printf("Error creating task odom\r\n");
		Error_Handler();
	}
	if (xTaskCreate(task_bordure, "bordure", TASK_BORDURE_STACK_DEPTH, NULL, TASK_BORDURE_PRIORITY, &h_task_bordure) != pdPASS)
	{
		printf("Error creating task bordure\r\n");
		Error_Handler();
	}
	if (xTaskCreate(task_angle, "angle", TASK_ANGLE_STACK_DEPTH, NULL, TASK_ANGLE_PRIORITY, &h_task_angle) != pdPASS)
	{
		printf("Error creating task angle\r\n");
		Error_Handler();
	}
	if (xTaskCreate(task_distance, "distance", TASK_DISTANCE_STACK_DEPTH, NULL, TASK_DISTANCE_PRIORITY, &h_task_distance) != pdPASS)
	{
		printf("Error creating task distance\r\n");
		Error_Handler();
	}

	if (xTaskCreate(task_encoder, "Encoder", TASK_ENCODER_STACK_DEPTH, NULL, TASK_ENCODER_PRIORITY, &h_task_encoder) != pdPASS)
	{
		printf("Error creating task shell\r\n");
		Error_Handler();
	}
	if (xTaskCreate(task_color, "Color", TASK_COLOR_STACK_DEPTH, NULL, TASK_COLOR_PRIORITY, &h_task_color) != pdPASS)
	{
		printf("Error creating task color\r\n");
		Error_Handler();
	}
	if (xTaskCreate(task_tof, "TOF", TASK_TOF_STACK_DEPTH, NULL, TASK_TOF_PRIORITY, &h_task_tof) != pdPASS)
	{
		printf("Error creating task TOF\r\n");
		Error_Handler();
	}
	if (xTaskCreate(task_searching, "Searching", TASK_SEARCHING_STACK_DEPTH, NULL, TASK_SEARCHING_PRIORITY, &h_task_searching) != pdPASS)
	{
		printf("Error creating task searching\r\n");
		Error_Handler();
	}
	vTaskStartScheduler();


	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//		XL_320_set_goal_position(0x01, 0);
		//		HAL_Delay(2000);
		//		position=XL_320_read_present_position(0x01);
		//		printf("Current position is %d \r\n",position);
		//		HAL_Delay(100);
		//		XL_320_set_goal_position(0x01, 350);
		//		HAL_Delay(2000);
		//		position=XL_320_read_present_position(0x01);
		//		printf("Current position is %d \r\n",position);
		//		HAL_Delay(100);


		//HAL_GPIO_TogglePin(BAT_LED_GPIO_Port, BAT_LED_Pin);
		//printf("Hello World! I am CARY\r\n");


		//HAL_Delay(1000);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM17)
	{
		TCS3200_CaptureCallback(&color_sensor);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) //USART3 stlink
	{
		shell_uart_receive_irq_cb(&h_shell);	// C'est la fonction qui donne le sémaphore!
	}
	else
	{
		buffer_index=Receive_length;

		BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(servo.sem_packet_read, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	printf("Stack Overflow\r\n"); // pile éclaté, pas de printf
	Error_Handler();
}

void configureTimerForRunTimeStats(void)
{
	HAL_TIM_Base_Start(&htim7);
}
unsigned long getRunTimeCounterValue(void)
{
	return htim7.Instance->CNT;
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	if (htim->Instance == TIM17)
	{
		TCS3200_PeriodElapsedCallback(&color_sensor);
	}
	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

