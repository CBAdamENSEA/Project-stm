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
#include "XL_320.h"
#include "shell.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_SHELL_STACK_DEPTH 512
#define TASK_SHELL_PRIORITY 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t h_task_shell = NULL;
h_shell_t h_shell;
uint8_t buffer[BUFFER_LENGTH] = {0};
uint16_t buffer_index = 0;
uint8_t Receive_length=0;
char Stat_buffer[250];
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
		XL_320_set_goal_position(0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(0x01);
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
		XL_320_set_goal_position(0x01, final_position);
		while (abs(position-final_position)<error)
		{
			position=XL_320_read_present_position(0x01);
		}
		return 0;
	}
	else
	{
		printf("Erreur, pas le bon nombre d'arguments\r\n");
		return -1;
	}
}



void task_shell(void * unused)
{
	shell_init(&h_shell);
	shell_add(&h_shell,'f', fonction, "Une fonction inutile");
	shell_add(&h_shell,'s', statistiques, "Afficher les stat");
	shell_add(&h_shell,'o', open_gate, "ouvrir la porte");
	shell_add(&h_shell,'c', close_gate, "fermer la porte");
	shell_run(&h_shell);	// boucle infinie
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
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM15_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM17_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	uint16_t model_number=0;
	uint8_t firmware_version=0;

	if(XL_320_ping(0x01,&model_number,&firmware_version)==1)
	{
		printf("model_number= 0x%04x \r\n", model_number);
		printf("firmware_version= 0x%04x \r\n", firmware_version);
	}
	XL_320_set_torque_enable(0x01, 1);
	HAL_Delay(100);
	XL_320_set_speed_position(0x01, 100);
	HAL_Delay(100);
	int position;
	printf("Creating task shell\r\n");
	if (xTaskCreate(task_shell, "Shell", TASK_SHELL_STACK_DEPTH, NULL, TASK_SHELL_PRIORITY, &h_task_shell) != pdPASS)
	{
		printf("Error creating task shell\r\n");
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		shell_uart_receive_irq_cb(&h_shell);	// C'est la fonction qui donne le sémaphore!
	}
	else
	{
		buffer_index=Receive_length;
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

