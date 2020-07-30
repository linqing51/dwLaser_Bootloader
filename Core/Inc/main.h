/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

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
#define DA_SYNC_Pin GPIO_PIN_13
#define DA_SYNC_GPIO_Port GPIOC
#define DA_SCLK_Pin GPIO_PIN_14
#define DA_SCLK_GPIO_Port GPIOC
#define DA_DIN_Pin GPIO_PIN_15
#define DA_DIN_GPIO_Port GPIOC
#define LPC_PWM0_Pin GPIO_PIN_4
#define LPC_PWM0_GPIO_Port GPIOA
#define LPB_PWM1_Pin GPIO_PIN_5
#define LPB_PWM1_GPIO_Port GPIOA
#define LPB_PWM0_Pin GPIO_PIN_6
#define LPB_PWM0_GPIO_Port GPIOA
#define LPA_PWM1_Pin GPIO_PIN_7
#define LPA_PWM1_GPIO_Port GPIOA
#define LPA_PWM0_Pin GPIO_PIN_4
#define LPA_PWM0_GPIO_Port GPIOC
#define I2C_SCL_Pin GPIO_PIN_5
#define I2C_SCL_GPIO_Port GPIOC
#define I2C_SDA_Pin GPIO_PIN_0
#define I2C_SDA_GPIO_Port GPIOB
#define PM_ALARM_Pin GPIO_PIN_1
#define PM_ALARM_GPIO_Port GPIOB
#define LCD_OUT_Pin GPIO_PIN_2
#define LCD_OUT_GPIO_Port GPIOB
#define FAN24V_OUT_Pin GPIO_PIN_14
#define FAN24V_OUT_GPIO_Port GPIOB
#define TEC_OUT_Pin GPIO_PIN_15
#define TEC_OUT_GPIO_Port GPIOB
#define PWR_KEY_Pin GPIO_PIN_8
#define PWR_KEY_GPIO_Port GPIOC
#define OTG_FS_PSOC_Pin GPIO_PIN_9
#define OTG_FS_PSOC_GPIO_Port GPIOC
#define OTG_FS_PSON_Pin GPIO_PIN_8
#define OTG_FS_PSON_GPIO_Port GPIOA
#define FSWITCH_NC_Pin GPIO_PIN_15
#define FSWITCH_NC_GPIO_Port GPIOA
#define FSWITCH_NO_Pin GPIO_PIN_10
#define FSWITCH_NO_GPIO_Port GPIOC
#define ESTOP_IN_Pin GPIO_PIN_11
#define ESTOP_IN_GPIO_Port GPIOC
#define INTLOCK_IN_Pin GPIO_PIN_12
#define INTLOCK_IN_GPIO_Port GPIOC
#define FAN5V_OUT_Pin GPIO_PIN_2
#define FAN5V_OUT_GPIO_Port GPIOD
#define AIM_OUT_Pin GPIO_PIN_4
#define AIM_OUT_GPIO_Port GPIOB
#define GREEN_OUT_Pin GPIO_PIN_5
#define GREEN_OUT_GPIO_Port GPIOB
#define RED_OUT_Pin GPIO_PIN_6
#define RED_OUT_GPIO_Port GPIOB
#define BLUE_OUT_Pin GPIO_PIN_7
#define BLUE_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
