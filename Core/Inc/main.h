/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef uint8_t u8;
typedef uint32_t u32;
typedef unsigned char uch;
typedef volatile uint8_t v8;

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
#define led_panel_Pin GPIO_PIN_13
#define led_panel_GPIO_Port GPIOC
#define rgb0_Pin GPIO_PIN_1
#define rgb0_GPIO_Port GPIOA
#define rgb1_Pin GPIO_PIN_2
#define rgb1_GPIO_Port GPIOA
#define led0_Pin GPIO_PIN_3
#define led0_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_4
#define led1_GPIO_Port GPIOA
#define led2_Pin GPIO_PIN_5
#define led2_GPIO_Port GPIOA
#define led3_Pin GPIO_PIN_6
#define led3_GPIO_Port GPIOA
#define led4_Pin GPIO_PIN_10
#define led4_GPIO_Port GPIOB
#define ledrow0_Pin GPIO_PIN_11
#define ledrow0_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_12
#define LCD_E_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_13
#define LCD_RS_GPIO_Port GPIOB
#define LCD_DATA_4_Pin GPIO_PIN_14
#define LCD_DATA_4_GPIO_Port GPIOB
#define LCD_DATA_5_Pin GPIO_PIN_15
#define LCD_DATA_5_GPIO_Port GPIOB
#define LCD_DATA_6_Pin GPIO_PIN_8
#define LCD_DATA_6_GPIO_Port GPIOA
#define LCD_DATA_7_Pin GPIO_PIN_9
#define LCD_DATA_7_GPIO_Port GPIOA
#define ledrow1_Pin GPIO_PIN_11
#define ledrow1_GPIO_Port GPIOA
#define ledrow2_Pin GPIO_PIN_12
#define ledrow2_GPIO_Port GPIOA
#define ledrow3_Pin GPIO_PIN_15
#define ledrow3_GPIO_Port GPIOA
#define ledrow4_Pin GPIO_PIN_3
#define ledrow4_GPIO_Port GPIOB
#define ledrow5_Pin GPIO_PIN_4
#define ledrow5_GPIO_Port GPIOB
#define ledrow6_Pin GPIO_PIN_5
#define ledrow6_GPIO_Port GPIOB
#define ledrow7_Pin GPIO_PIN_6
#define ledrow7_GPIO_Port GPIOB
#define ledrow8_Pin GPIO_PIN_7
#define ledrow8_GPIO_Port GPIOB
#define ledrow9_Pin GPIO_PIN_8
#define ledrow9_GPIO_Port GPIOB
#define in_btn_Pin GPIO_PIN_9
#define in_btn_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
