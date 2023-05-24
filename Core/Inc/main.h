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
#define M4_FBS_Pin GPIO_PIN_13
#define M4_FBS_GPIO_Port GPIOC
#define LP_PWM1_Pin GPIO_PIN_15
#define LP_PWM1_GPIO_Port GPIOC
#define LAS_PD_Pin GPIO_PIN_0
#define LAS_PD_GPIO_Port GPIOC
#define LAS_NTC_Pin GPIO_PIN_1
#define LAS_NTC_GPIO_Port GPIOC
#define HT_NTC_Pin GPIO_PIN_2
#define HT_NTC_GPIO_Port GPIOC
#define LP_PWM0_Pin GPIO_PIN_3
#define LP_PWM0_GPIO_Port GPIOC
#define SPK_TIMER_Pin GPIO_PIN_0
#define SPK_TIMER_GPIO_Port GPIOA
#define SPK_CS_Pin GPIO_PIN_1
#define SPK_CS_GPIO_Port GPIOA
#define SPK_SCK_Pin GPIO_PIN_2
#define SPK_SCK_GPIO_Port GPIOA
#define SPK_SI_Pin GPIO_PIN_3
#define SPK_SI_GPIO_Port GPIOA
#define LP_SET1_Pin GPIO_PIN_4
#define LP_SET1_GPIO_Port GPIOA
#define LP_SET0_Pin GPIO_PIN_5
#define LP_SET0_GPIO_Port GPIOA
#define TP_SET_Pin GPIO_PIN_6
#define TP_SET_GPIO_Port GPIOA
#define TP_PWM_Pin GPIO_PIN_7
#define TP_PWM_GPIO_Port GPIOA
#define TPR_NTC_Pin GPIO_PIN_4
#define TPR_NTC_GPIO_Port GPIOC
#define LPR0_NTC_Pin GPIO_PIN_5
#define LPR0_NTC_GPIO_Port GPIOC
#define TPR1_NTC_Pin GPIO_PIN_0
#define TPR1_NTC_GPIO_Port GPIOB
#define EPROM_SCL_Pin GPIO_PIN_10
#define EPROM_SCL_GPIO_Port GPIOB
#define EPROM_SDA_Pin GPIO_PIN_11
#define EPROM_SDA_GPIO_Port GPIOB
#define DBG_LED_Pin GPIO_PIN_12
#define DBG_LED_GPIO_Port GPIOB
#define TP_DIR_Pin GPIO_PIN_13
#define TP_DIR_GPIO_Port GPIOB
#define LAS_FB_Pin GPIO_PIN_14
#define LAS_FB_GPIO_Port GPIOB
#define ESTOP_NC_Pin GPIO_PIN_15
#define ESTOP_NC_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_6
#define FAN_PWM_GPIO_Port GPIOC
#define FAN_FG_Pin GPIO_PIN_7
#define FAN_FG_GPIO_Port GPIOC
#define FAN_ENA_Pin GPIO_PIN_8
#define FAN_ENA_GPIO_Port GPIOC
#define G5_AIM_PWM_Pin GPIO_PIN_9
#define G5_AIM_PWM_GPIO_Port GPIOC
#define USB_OTG_FS_ON_Pin GPIO_PIN_8
#define USB_OTG_FS_ON_GPIO_Port GPIOA
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define USB_OTG_FS_DM_Pin GPIO_PIN_11
#define USB_OTG_FS_DM_GPIO_Port GPIOA
#define USB_OTG_FS_DP_Pin GPIO_PIN_12
#define USB_OTG_FS_DP_GPIO_Port GPIOA
#define INTERLOCK_NC_Pin GPIO_PIN_15
#define INTERLOCK_NC_GPIO_Port GPIOA
#define FS_NO_Pin GPIO_PIN_4
#define FS_NO_GPIO_Port GPIOB
#define FS_NC_Pin GPIO_PIN_5
#define FS_NC_GPIO_Port GPIOB
#define RED_LED_PWM_Pin GPIO_PIN_6
#define RED_LED_PWM_GPIO_Port GPIOB
#define GREEN_LED_PWM_Pin GPIO_PIN_7
#define GREEN_LED_PWM_GPIO_Port GPIOB
#define BLUE_LED_PWM_Pin GPIO_PIN_8
#define BLUE_LED_PWM_GPIO_Port GPIOB
#define M4_PIC_Pin GPIO_PIN_9
#define M4_PIC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
