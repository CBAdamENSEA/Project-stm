/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define BAT_STATE_Pin GPIO_PIN_0
#define BAT_STATE_GPIO_Port GPIOA
#define BAT_LED_Pin GPIO_PIN_1
#define BAT_LED_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_2
#define ESP_TX_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_3
#define ESP_RX_GPIO_Port GPIOA
#define ESP_EN_Pin GPIO_PIN_4
#define ESP_EN_GPIO_Port GPIOA
#define ESP_STATE_Pin GPIO_PIN_5
#define ESP_STATE_GPIO_Port GPIOA
#define SERV_DATA_Pin GPIO_PIN_4
#define SERV_DATA_GPIO_Port GPIOC
#define TOF2_GPIO1_Pin GPIO_PIN_1
#define TOF2_GPIO1_GPIO_Port GPIOB
#define TOF2_XSHUT_Pin GPIO_PIN_2
#define TOF2_XSHUT_GPIO_Port GPIOB
#define TOF2_SCL_Pin GPIO_PIN_10
#define TOF2_SCL_GPIO_Port GPIOB
#define TOF2_SDA_Pin GPIO_PIN_11
#define TOF2_SDA_GPIO_Port GPIOB
#define DRV_1_REV_Pin GPIO_PIN_13
#define DRV_1_REV_GPIO_Port GPIOB
#define DRV_1_FWD_Pin GPIO_PIN_14
#define DRV_1_FWD_GPIO_Port GPIOB
#define ENC_1_A_Pin GPIO_PIN_8
#define ENC_1_A_GPIO_Port GPIOA
#define ENC_1_B_Pin GPIO_PIN_9
#define ENC_1_B_GPIO_Port GPIOA
#define ENC_2_A_Pin GPIO_PIN_6
#define ENC_2_A_GPIO_Port GPIOC
#define ENC_2_B_Pin GPIO_PIN_7
#define ENC_2_B_GPIO_Port GPIOC
#define STLINK_TX_Pin GPIO_PIN_8
#define STLINK_TX_GPIO_Port GPIOD
#define STLINK_RX_Pin GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOD
#define SYS_SWDIO_Pin GPIO_PIN_13
#define SYS_SWDIO_GPIO_Port GPIOA
#define SYS_SWCLK_Pin GPIO_PIN_14
#define SYS_SWCLK_GPIO_Port GPIOA
#define OPTQ2_Pin GPIO_PIN_15
#define OPTQ2_GPIO_Port GPIOA
#define OPTQ1_Pin GPIO_PIN_8
#define OPTQ1_GPIO_Port GPIOC
#define DRV_2_FWD_Pin GPIO_PIN_0
#define DRV_2_FWD_GPIO_Port GPIOD
#define CLR_OUT_Pin GPIO_PIN_1
#define CLR_OUT_GPIO_Port GPIOD
#define CLR_S0_Pin GPIO_PIN_2
#define CLR_S0_GPIO_Port GPIOD
#define CLR_S1_Pin GPIO_PIN_3
#define CLR_S1_GPIO_Port GPIOD
#define CLR_S2_Pin GPIO_PIN_4
#define CLR_S2_GPIO_Port GPIOD
#define CLR_S3_Pin GPIO_PIN_5
#define CLR_S3_GPIO_Port GPIOD
#define CLR_OE_Pin GPIO_PIN_6
#define CLR_OE_GPIO_Port GPIOD
#define DRV_2_REV_Pin GPIO_PIN_6
#define DRV_2_REV_GPIO_Port GPIOB
#define TOF1_SDA_Pin GPIO_PIN_7
#define TOF1_SDA_GPIO_Port GPIOB
#define TOF1_SCL_Pin GPIO_PIN_8
#define TOF1_SCL_GPIO_Port GPIOB
#define TOF1_XSHUT_Pin GPIO_PIN_9
#define TOF1_XSHUT_GPIO_Port GPIOB
#define TOF1_GPIO1_Pin GPIO_PIN_10
#define TOF1_GPIO1_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
