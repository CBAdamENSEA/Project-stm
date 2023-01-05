/*
 * TCS3200.c
 *
 *  Created on: Oct 21, 2022
 *      Author: cheik
 */


#include "TCS3200.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "tim.h"

extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart3;
char display_msg[100];
extern uint32_t freq;
extern uint8_t check_freq;

uint8_t TCS3200_Enable(TCS3200_state_enable_t state_enable)
{
	HAL_GPIO_WritePin(CLR_OE_GPIO_Port, CLR_OE_Pin, state_enable);
	return 1;
}

uint8_t TCS3200_LED_Enable(GPIO_PinState led_enable)
{
	HAL_GPIO_WritePin(CLR_LED_GPIO_Port, CLR_LED_Pin, led_enable);
	return 1;
}

uint8_t TCS3200_Set_Scale(TCS3200_fo_t fo_scale)
{
	switch(fo_scale)
	{
	case SCALE_FO_2:
		HAL_GPIO_WritePin(CLR_S0_GPIO_Port, CLR_S0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CLR_S1_GPIO_Port, CLR_S1_Pin, GPIO_PIN_SET);
		break;
	case SCALE_FO_20:
		HAL_GPIO_WritePin(CLR_S0_GPIO_Port, CLR_S0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLR_S1_GPIO_Port, CLR_S1_Pin, GPIO_PIN_RESET);
		break;
	case SCALE_FO_100:
		HAL_GPIO_WritePin(CLR_S0_GPIO_Port, CLR_S0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLR_S1_GPIO_Port, CLR_S1_Pin, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(CLR_S0_GPIO_Port, CLR_S0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CLR_S1_GPIO_Port, CLR_S1_Pin, GPIO_PIN_RESET);
		break;
	}
	return 1;
}

uint8_t TCS3200_Set_Filter(TCS3200_filter_t filter)
{
	switch(filter)
	{
	case FILTER_RED:
		HAL_GPIO_WritePin(CLR_S2_GPIO_Port, CLR_S2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CLR_S3_GPIO_Port, CLR_S3_Pin, GPIO_PIN_RESET);
		break;
	case FILTER_GREEN:
		HAL_GPIO_WritePin(CLR_S2_GPIO_Port, CLR_S2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLR_S3_GPIO_Port, CLR_S3_Pin, GPIO_PIN_SET);
	case FILTER_BLUE:
		HAL_GPIO_WritePin(CLR_S2_GPIO_Port, CLR_S2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CLR_S3_GPIO_Port, CLR_S3_Pin, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(CLR_S2_GPIO_Port, CLR_S2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLR_S3_GPIO_Port, CLR_S3_Pin, GPIO_PIN_RESET);
		break;
	}
	return 1;
}

uint8_t TCS3200_Init(color_sensor_t * color_sensor)
{
	color_sensor->drv_input_capture.freq=0;
	color_sensor->drv_input_capture.input_freq_start=0;
	color_sensor->drv_input_capture.gu8_State = IDLE;
	color_sensor->drv_input_capture.gu32_T1 = 0;
	color_sensor->drv_input_capture.gu32_T2 = 0;
	color_sensor->drv_input_capture.gu32_Ticks = 0;
	color_sensor->drv_input_capture.gu16_TIM10_OVC = 0;
	color_sensor->drv_input_capture.gu32_Freq = 0;

	color_sensor->red=0;
	color_sensor->green=0;
	color_sensor->blue=0;
	color_sensor->sem_color_read=xSemaphoreCreateBinary();
	if (color_sensor->sem_color_read == NULL)
	{
		printf("Error semaphore color sensor\r\n");
		while(1);
	}
	if (TCS3200_Enable(STATE_ENABLE)==0)
	{
		return 0;
	}
	if (TCS3200_Set_Filter(FILTER_CLEAR)==0)
	{
		return 0;
	}
	if (TCS3200_Set_Scale(SCALE_FO_20)==0)
	{
		return 0;
	}
	if (TCS3200_LED_Enable(LED_DISABLE)==0)
	{
		return 0;
	}
	return 1;
}

uint8_t TCS3200_Calibration(TCS3200_filter_t filter,uint32_t freq)
{
	uint8_t filter_output;
	switch (filter){
	case FILTER_RED:
		filter_output = (255.0/(MAX_RED-MIN_RED))*(freq-MIN_RED);
		break;
	case FILTER_GREEN:
		filter_output = (255.0/(MAX_GREEN-MIN_GREEN))*(freq-MIN_GREEN);
		break;
	case FILTER_BLUE:
		filter_output = (255.0/(MAX_BLUE-MIN_BLUE))*(freq-MIN_BLUE);
		break;
	}
	if (filter_output > 255) filter_output = 255;
	if (filter_output < 0) filter_output = 0;
	return filter_output;
}

void TCS3200_Display_Frequency(TCS3200_filter_t filter,uint32_t freq)
{
	switch (filter){
	case FILTER_RED:
		printf("RED = %lu Hz\r\n",freq);
		break;
	case FILTER_GREEN:
		printf("GREEN = %lu Hz\r\n",freq);
		break;
	case FILTER_BLUE:
		printf("BLUE = %lu Hz\r\n",freq);
		break;
	}
}

void TCS3200_Display_Color(TCS3200_filter_t filter,uint8_t filter_output)
{
	switch (filter){
	case FILTER_RED:
		printf("RED = %u \r\n",filter_output);
		break;
	case FILTER_GREEN:
		printf("GREEN = %u \r\n",filter_output);
		break;
	case FILTER_BLUE:
		printf("BLUE = %u \r\n",filter_output);
		break;
	}
}

void TCS3200_Display_Colors(color_sensor_t color_sensor)
{
	printf("(%d,%d,%d)\r\n",color_sensor.red,color_sensor.green,color_sensor.blue);
}

void TCS3200_Detected_Color(color_sensor_t *color_sensor)
{
	if (color_sensor->red>30000)
	{
		color_sensor->output=1;
		printf("RED\r\n");
	}
	else
	{
		color_sensor->output=0;
		printf("GREEN\r\n");
	}

}

uint8_t TCS3200_Read_Color(color_sensor_t * color_sensor,TCS3200_filter_t filter)
{
	uint8_t filter_output;


	if (HAL_TIM_Base_Start_IT(&htim17)!=HAL_OK)
	{
		return 0;
	}
	if(HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1)!=HAL_OK)
	{
		return 0;
	}

	if(TCS3200_LED_Enable(LED_ENABLE)==0)
	{
		return 0;
	}
	if (TCS3200_Set_Filter(filter)==0)
	{
		return 0;
	}
	//vTaskDelay(100); // A changer
	// ajouter un sémaphore au lieu de la variable input_freq_start
	color_sensor->drv_input_capture.input_freq_start=1; //So that the input capture timer can start calculating the frequency
	while(color_sensor->drv_input_capture.input_freq_start); // Wait until the frequency is calculated
	if(TCS3200_LED_Enable(LED_DISABLE)==0)
	{
		return 0;
	}
	if(TCS3200_Set_Filter(FILTER_CLEAR)==0)
	{
		return 0;
	}
	if(HAL_TIM_IC_Stop_IT(&htim17, TIM_CHANNEL_1)!=HAL_OK)
	{
		return 0;
	}
	if(HAL_TIM_Base_Stop_IT(&htim17)!=HAL_OK)
	{
		return 0;
	}

	switch (filter){
	case FILTER_RED:

		color_sensor->red=color_sensor->drv_input_capture.freq;
		break;
	case FILTER_GREEN:
		color_sensor->green=color_sensor->drv_input_capture.freq;

		break;
	case FILTER_BLUE:
		color_sensor->blue=color_sensor->drv_input_capture.freq;
		break;
	}
	if (TCS3200_Set_Filter(FILTER_CLEAR)==0)
	{
		return 0;
	}



	return 1;
}
void TCS3200_CaptureCallback(color_sensor_t * color_sensor)
{
	if((color_sensor->drv_input_capture.gu8_State == IDLE)&(color_sensor->drv_input_capture.input_freq_start==1))
	{
		color_sensor->drv_input_capture.gu32_T1 = htim17.Instance->CCR1;
		color_sensor->drv_input_capture.gu16_TIM10_OVC = 0;
		color_sensor->drv_input_capture.gu8_State = DONE;

	}
	else if((color_sensor->drv_input_capture.gu8_State == DONE)&(color_sensor->drv_input_capture.input_freq_start==1))
	{
		color_sensor->drv_input_capture.gu32_T2 = htim17.Instance->CCR1;
		color_sensor->drv_input_capture.gu32_Ticks = (color_sensor->drv_input_capture.gu32_T2 +
				(color_sensor->drv_input_capture.gu16_TIM10_OVC * 65535)) - color_sensor->drv_input_capture.gu32_T1;
		color_sensor->drv_input_capture.freq = (uint32_t)(F_CLK/color_sensor->drv_input_capture.gu32_Ticks);
		if(color_sensor->drv_input_capture.freq != 0)
		{
			//sprintf(freq_msg, "Frequency = %lu Hz\r\n",gu32_Freq);
			//HAL_UART_Transmit(&huart1,freq_msg,strlen(freq_msg), HAL_MAX_DELAY);
			color_sensor->drv_input_capture.input_freq_start=0;
		}
		color_sensor->drv_input_capture.gu8_State = IDLE;
	}

}
void TCS3200_PeriodElapsedCallback(color_sensor_t * color_sensor)
{
	color_sensor->drv_input_capture.gu16_TIM10_OVC++;
}
