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
#define TASK_ENCODER_PRIORITY 1
#define TASK_COLOR_STACK_DEPTH 512
#define TASK_COLOR_PRIORITY 3
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
TaskHandle_t h_task_color = NULL;
h_shell_t h_shell;
uint8_t buffer[BUFFER_LENGTH] = {0};
uint16_t buffer_index = 0;
uint8_t Receive_length=0;
char Stat_buffer[250];
servo_t servo;
motors_t motors;
uint32_t cnt_encoder1;
uint32_t cnt_encoder2;
encoders_t encoders;
color_sensor_t color_sensor;
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
			encoders.right.consigne=atoi(argv[3]);
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
		uint16_t alpha=150;
		uint16_t position=350;
		uint16_t final_position=0;
		uint16_t error=5;
		motors.left.drv_motor.drv_avance(alpha);
		motors.right.drv_motor.drv_avance(alpha+12);
		vTaskDelay(1000);
		printf("R speed = %d Rpm\r\n",encoders.right.speed);
		printf("L speed = %d Rpm\r\n",encoders.left.speed);
		printf("R distance = %d mm\r\n",encoders.right.distance);
		printf("L distance = %d mm\r\n",encoders.left.distance);
		vTaskDelay(800);
		motors.left.drv_motor.drv_stop();
		motors.right.drv_motor.drv_stop();
		vTaskDelay(500);
		XL_320_set_goal_position(&servo,0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(&servo,0x01);
		}
		vTaskDelay(4000);
		xSemaphoreGive(color_sensor.sem_color_read);
		TCS3200_Detected_Color(&color_sensor);
		vTaskDelay(4000);
		xSemaphoreGive(color_sensor.sem_color_read);
		TCS3200_Detected_Color(&color_sensor);
		vTaskDelay(1000);

		position=0;
		final_position=350;
		error=5;

		XL_320_set_goal_position(&servo, 0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(&servo,0x01);
		}
		vTaskDelay(2000);
		motors.left.drv_motor.drv_recule(alpha);
		motors.right.drv_motor.drv_recule(alpha+12);
		vTaskDelay(2000);
		motors.left.drv_motor.drv_stop();
		motors.right.drv_motor.drv_stop();
		vTaskDelay(500);
		motors.left.drv_motor.drv_avance(alpha);
		motors.right.drv_motor.drv_recule(alpha);
		vTaskDelay(1150); // Tourner
		motors.left.drv_motor.drv_stop();
		motors.right.drv_motor.drv_stop();
		vTaskDelay(400); // Arr??ter


		position=350;
		final_position=0;
		error=5;
		XL_320_set_goal_position(&servo,0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(&servo,0x01);
		}
		vTaskDelay(1000);
		motors.left.drv_motor.drv_recule(alpha);
		motors.right.drv_motor.drv_recule(alpha+12);
		vTaskDelay(800);
		motors.left.drv_motor.drv_stop();
		motors.right.drv_motor.drv_stop();
		position=0;
		final_position=350;
		error=5;

		XL_320_set_goal_position(&servo, 0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(&servo,0x01);
		}
		vTaskDelay(1000);


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
		HAL_UART_Transmit(&huart3, (uint8_t*)Stat_buffer, strlen(Stat_buffer), HAL_MAX_DELAY);
		printf("\r\nTask            State   Prior.  Stack   Num\r\n");
		vTaskList(Stat_buffer);
		HAL_UART_Transmit(&huart3, (uint8_t*)Stat_buffer, strlen(Stat_buffer), HAL_MAX_DELAY);

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
	shell_add(&h_shell,'t', test_pwm, "Tester le pwm du moteur right");
	shell_add(&h_shell,'o', open_gate, "ouvrir la porte");
	shell_add(&h_shell,'c', close_gate, "fermer la porte");
	shell_add(&h_shell,'m', motor, "tourner les moteurs");
	shell_add(&h_shell,'e', speed, "vitesse des moteurs");
	shell_add(&h_shell,'r', couleurs, "couleur de la canette");
	shell_add(&h_shell,'d', demo, "D??monstration");
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
		vTaskDelay(50);
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
	if (huart->Instance == USART3)
	{
		shell_uart_receive_irq_cb(&h_shell);	// C'est la fonction qui donne le s??maphore!
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
	printf("Stack Overflow\r\n"); // pile ??clat??, pas de printf
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

