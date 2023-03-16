/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Encode_4A_Pin GPIO_PIN_0
#define Encode_4A_GPIO_Port GPIOA
#define Encode_4B_Pin GPIO_PIN_1
#define Encode_4B_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_4
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_RS_Pin GPIO_PIN_6
#define LCD_DC_RS_GPIO_Port GPIOA
#define Step1_dir_Pin GPIO_PIN_7
#define Step1_dir_GPIO_Port GPIOE
#define Step2_dir_Pin GPIO_PIN_8
#define Step2_dir_GPIO_Port GPIOE
#define Step1_Pin GPIO_PIN_9
#define Step1_GPIO_Port GPIOE
#define Step3_dir_Pin GPIO_PIN_10
#define Step3_dir_GPIO_Port GPIOE
#define Step2_Pin GPIO_PIN_11
#define Step2_GPIO_Port GPIOE
#define Step1_key_Pin GPIO_PIN_12
#define Step1_key_GPIO_Port GPIOE
#define Step3_Pin GPIO_PIN_13
#define Step3_GPIO_Port GPIOE
#define Step2_key_Pin GPIO_PIN_14
#define Step2_key_GPIO_Port GPIOE
#define Step3_key_Pin GPIO_PIN_15
#define Step3_key_GPIO_Port GPIOE
#define D_3_Pin GPIO_PIN_12
#define D_3_GPIO_Port GPIOB
#define D_4_Pin GPIO_PIN_13
#define D_4_GPIO_Port GPIOB
#define D_1_Pin GPIO_PIN_14
#define D_1_GPIO_Port GPIOB
#define D_2_Pin GPIO_PIN_15
#define D_2_GPIO_Port GPIOB
#define C_3_Pin GPIO_PIN_8
#define C_3_GPIO_Port GPIOD
#define C_4_Pin GPIO_PIN_9
#define C_4_GPIO_Port GPIOD
#define C_1_Pin GPIO_PIN_10
#define C_1_GPIO_Port GPIOD
#define C_2_Pin GPIO_PIN_11
#define C_2_GPIO_Port GPIOD
#define Encode_3A_Pin GPIO_PIN_12
#define Encode_3A_GPIO_Port GPIOD
#define Encode_3B_Pin GPIO_PIN_13
#define Encode_3B_GPIO_Port GPIOD
#define B_3_Pin GPIO_PIN_14
#define B_3_GPIO_Port GPIOD
#define B_4_Pin GPIO_PIN_15
#define B_4_GPIO_Port GPIOD
#define B_1_Pin GPIO_PIN_2
#define B_1_GPIO_Port GPIOG
#define B_2_Pin GPIO_PIN_3
#define B_2_GPIO_Port GPIOG
#define A_3_Pin GPIO_PIN_4
#define A_3_GPIO_Port GPIOG
#define Motor_ch1_Pin GPIO_PIN_5
#define Motor_ch1_GPIO_Port GPIOG
#define Motor_ch2_Pin GPIO_PIN_6
#define Motor_ch2_GPIO_Port GPIOG
#define Motor_ch3_Pin GPIO_PIN_7
#define Motor_ch3_GPIO_Port GPIOG
#define Motor_ch4_Pin GPIO_PIN_8
#define Motor_ch4_GPIO_Port GPIOG
#define Motor_1_Pin GPIO_PIN_6
#define Motor_1_GPIO_Port GPIOC
#define Motor_2_Pin GPIO_PIN_7
#define Motor_2_GPIO_Port GPIOC
#define Motor_3_Pin GPIO_PIN_8
#define Motor_3_GPIO_Port GPIOC
#define Motor_4_Pin GPIO_PIN_9
#define Motor_4_GPIO_Port GPIOC
#define A_4_Pin GPIO_PIN_8
#define A_4_GPIO_Port GPIOA
#define A_1_Pin GPIO_PIN_11
#define A_1_GPIO_Port GPIOA
#define A_2_Pin GPIO_PIN_12
#define A_2_GPIO_Port GPIOA
#define Encode_1A_Pin GPIO_PIN_15
#define Encode_1A_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOD
#define ALL_key_Pin GPIO_PIN_1
#define ALL_key_GPIO_Port GPIOD
#define ALL_key_EXTI_IRQn EXTI1_IRQn
#define Encode_1B_Pin GPIO_PIN_3
#define Encode_1B_GPIO_Port GPIOB
#define Encode_2A_Pin GPIO_PIN_4
#define Encode_2A_GPIO_Port GPIOB
#define Encode_2B_Pin GPIO_PIN_5
#define Encode_2B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
