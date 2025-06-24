/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define ID_Sel1_Dir1_Pin GPIO_PIN_13
#define ID_Sel1_Dir1_GPIO_Port GPIOC
#define ID_Sel0_Dir0_Pin GPIO_PIN_14
#define ID_Sel0_Dir0_GPIO_Port GPIOC
#define ID_Sel2_Dir2_Pin GPIO_PIN_15
#define ID_Sel2_Dir2_GPIO_Port GPIOC
#define RGB_B_Pin GPIO_PIN_10
#define RGB_B_GPIO_Port GPIOG
#define Low_Sw0_Sense_Pin GPIO_PIN_0
#define Low_Sw0_Sense_GPIO_Port GPIOC
#define Low_Sw1_Sense_Pin GPIO_PIN_1
#define Low_Sw1_Sense_GPIO_Port GPIOC
#define Low_Sw2_Sense_Pin GPIO_PIN_2
#define Low_Sw2_Sense_GPIO_Port GPIOC
#define BiDir1_Sense_Pin GPIO_PIN_3
#define BiDir1_Sense_GPIO_Port GPIOC
#define ENC_A_ExtClk_Pin GPIO_PIN_0
#define ENC_A_ExtClk_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define BiDir0_Sense_Pin GPIO_PIN_2
#define BiDir0_Sense_GPIO_Port GPIOA
#define BiDir2_Sense_Pin GPIO_PIN_3
#define BiDir2_Sense_GPIO_Port GPIOA
#define BiDir_ILimit_Pin GPIO_PIN_4
#define BiDir_ILimit_GPIO_Port GPIOA
#define BiDir3_Dir_Pin GPIO_PIN_5
#define BiDir3_Dir_GPIO_Port GPIOA
#define Low_Sw4_Sense_Pin GPIO_PIN_6
#define Low_Sw4_Sense_GPIO_Port GPIOA
#define Low_Sw5_Sense_Pin GPIO_PIN_7
#define Low_Sw5_Sense_GPIO_Port GPIOA
#define Low_Sw6_Sense_Pin GPIO_PIN_4
#define Low_Sw6_Sense_GPIO_Port GPIOC
#define DigIn0_Pin GPIO_PIN_5
#define DigIn0_GPIO_Port GPIOC
#define BiDir3_Sense_Pin GPIO_PIN_0
#define BiDir3_Sense_GPIO_Port GPIOB
#define NTC_In_Pin GPIO_PIN_1
#define NTC_In_GPIO_Port GPIOB
#define Low_Sw6_PWM_Pin GPIO_PIN_2
#define Low_Sw6_PWM_GPIO_Port GPIOB
#define CAN_Term_Pin GPIO_PIN_12
#define CAN_Term_GPIO_Port GPIOB
#define Cond_In_Pin GPIO_PIN_13
#define Cond_In_GPIO_Port GPIOB
#define Low_Sw5_PWM_Pin GPIO_PIN_14
#define Low_Sw5_PWM_GPIO_Port GPIOB
#define Low_Sw3_Sense_Pin GPIO_PIN_15
#define Low_Sw3_Sense_GPIO_Port GPIOB
#define Low_Sw4_PWM_Pin GPIO_PIN_6
#define Low_Sw4_PWM_GPIO_Port GPIOC
#define Low_Sw3_PWM_Pin GPIO_PIN_7
#define Low_Sw3_PWM_GPIO_Port GPIOC
#define Low_Sw2_PWM_Pin GPIO_PIN_8
#define Low_Sw2_PWM_GPIO_Port GPIOC
#define Low_Sw1_PWM_Pin GPIO_PIN_9
#define Low_Sw1_PWM_GPIO_Port GPIOC
#define BiDir3_PWM_Pin GPIO_PIN_8
#define BiDir3_PWM_GPIO_Port GPIOA
#define BiDir2_PWM_Pin GPIO_PIN_9
#define BiDir2_PWM_GPIO_Port GPIOA
#define BiDir1_PWM_Pin GPIO_PIN_10
#define BiDir1_PWM_GPIO_Port GPIOA
#define DigIn2_Pin GPIO_PIN_10
#define DigIn2_GPIO_Port GPIOC
#define RGB_R_Pin GPIO_PIN_11
#define RGB_R_GPIO_Port GPIOC
#define DigIn1_Pin GPIO_PIN_12
#define DigIn1_GPIO_Port GPIOC
#define Flow_Pin GPIO_PIN_2
#define Flow_GPIO_Port GPIOD
#define BiDir_nFault_Pin GPIO_PIN_3
#define BiDir_nFault_GPIO_Port GPIOB
#define Conductivity_Enable_Pin GPIO_PIN_4
#define Conductivity_Enable_GPIO_Port GPIOB
#define RGB_G_Pin GPIO_PIN_5
#define RGB_G_GPIO_Port GPIOB
#define BiDir0_PWM_Pin GPIO_PIN_6
#define BiDir0_PWM_GPIO_Port GPIOB
#define BiDir_nSleep_BOOT0_Pin GPIO_PIN_8
#define BiDir_nSleep_BOOT0_GPIO_Port GPIOB
#define Low_Sw0_PWM_Pin GPIO_PIN_9
#define Low_Sw0_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
