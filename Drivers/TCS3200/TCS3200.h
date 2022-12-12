/*
 * TCS3200.h
 *
 *  Created on: Oct 21, 2022
 *      Author: cheik
 */
#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"

#ifndef INC_TCS3200_H_
#define INC_TCS3200_H_

////Private Defines
#define IDLE   0
#define DONE   1
#define F_CLK  64000000UL

// For input frequency calculation

typedef struct drv_input_capture_struct
{
	uint8_t gu8_State;
	uint32_t gu32_T1 ;
	uint32_t gu32_T2;
	uint32_t gu32_Ticks;
	uint16_t gu16_TIM10_OVC;
	uint32_t gu32_Freq;
	uint8_t input_freq_start;
	uint32_t freq;
}drv_input_capture_t;

typedef struct color_sensor_struct
{
	drv_input_capture_t drv_input_capture;
	uint32_t red;
	uint32_t green;
	uint32_t blue;
	uint8_t output;
	SemaphoreHandle_t sem_color_read;
} color_sensor_t;


// Filters
typedef enum TCS3200_filter_enum
{
	FILTER_RED       = 0,
	FILTER_BLUE      = 1,
	FILTER_CLEAR  	= 2,
	FILTER_GREEN 	= 3
} TCS3200_filter_t;

// f_o scales
typedef enum TCS3200_fo_enum
{
	SCALE_FO_0     = 0,
	SCALE_FO_2      = 1,
	SCALE_FO_20  	= 2,
	SCALE_FO_100 	= 3
} TCS3200_fo_t;
typedef enum TCS3200_state_enable
{
	STATE_ENABLE     = GPIO_PIN_RESET,
	STATE_DISABLE	= GPIO_PIN_SET
} TCS3200_state_enable_t;

typedef enum TCS3200_led_enable
{
	LED_ENABLE     = GPIO_PIN_SET,
	LED_DISABLE		=GPIO_PIN_RESET
} TCS3200_led_enable_t;

// Calibration
#define MIN_RED 7611
#define MAX_RED 56600
#define MIN_GREEN 9831
#define MAX_GREEN 55000
#define MIN_BLUE 9853
#define MAX_BLUE 58000

//// Typedef



//// Functions

uint8_t TCS3200_Enable(TCS3200_state_enable_t state_enable);
uint8_t TCS3200_LED_Enable(GPIO_PinState led_enable);
uint8_t TCS3200_Set_Scale(TCS3200_fo_t fo_scale);
uint8_t TCS3200_Set_Filter(TCS3200_filter_t filter);
uint8_t TCS3200_Init(color_sensor_t * color_sensor);
uint8_t TCS3200_Calibration(TCS3200_filter_t filter,uint32_t freq);
void TCS3200_Display_Frequency(TCS3200_filter_t filter,uint32_t freq);
void TCS3200_Display_Color(TCS3200_filter_t filter,uint8_t Output_Color);
void TCS3200_Display_Colors(color_sensor_t color_sensor);
void TCS3200_Detected_Color(color_sensor_t *color_sensor);
uint8_t TCS3200_Read_Color(color_sensor_t * color_sensor,TCS3200_filter_t filter);
void TCS3200_CaptureCallback(color_sensor_t * color_sensor);
void TCS3200_PeriodElapsedCallback(color_sensor_t * color_sensor);


#endif /* INC_TCS3200_H_ */
