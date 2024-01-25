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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

//Task periods in system ticks (each tick is 1ms)
#define APPS_PERIOD 20U
#define WATCHDOG_PERIOD 2U
#define CTRL_PERIOD 10U
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define ADC3_AUX4_Pin GPIO_PIN_4
#define ADC3_AUX4_GPIO_Port GPIOF
#define ADC3_AUX4F5_Pin GPIO_PIN_5
#define ADC3_AUX4F5_GPIO_Port GPIOF
#define ADC3_AUX5_Pin GPIO_PIN_6
#define ADC3_AUX5_GPIO_Port GPIOF
#define ADC3_AUX6_Pin GPIO_PIN_7
#define ADC3_AUX6_GPIO_Port GPIOF
#define ADC3_AUX7_Pin GPIO_PIN_8
#define ADC3_AUX7_GPIO_Port GPIOF
#define ADC3_AUX8_Pin GPIO_PIN_9
#define ADC3_AUX8_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define ADC2_DMPR2_Pin GPIO_PIN_1
#define ADC2_DMPR2_GPIO_Port GPIOC
#define ADC1_APPS2_Pin GPIO_PIN_0
#define ADC1_APPS2_GPIO_Port GPIOA
#define ADC1_APPS1_Pin GPIO_PIN_1
#define ADC1_APPS1_GPIO_Port GPIOA
#define ADC3_AUX2_Pin GPIO_PIN_3
#define ADC3_AUX2_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define GPIO_D1_Pin GPIO_PIN_7
#define GPIO_D1_GPIO_Port GPIOE
#define GPIO_D2_Pin GPIO_PIN_8
#define GPIO_D2_GPIO_Port GPIOE
#define GPIO_D3_Pin GPIO_PIN_9
#define GPIO_D3_GPIO_Port GPIOE
#define GPIO_D4_Pin GPIO_PIN_10
#define GPIO_D4_GPIO_Port GPIOE
#define GPIO_D9_Pin GPIO_PIN_11
#define GPIO_D9_GPIO_Port GPIOE
#define GPIO_D7_Pin GPIO_PIN_12
#define GPIO_D7_GPIO_Port GPIOE
#define GPIO_D10_Pin GPIO_PIN_13
#define GPIO_D10_GPIO_Port GPIOE
#define GPIO_D8_Pin GPIO_PIN_14
#define GPIO_D8_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define GPIO_D5_Pin GPIO_PIN_12
#define GPIO_D5_GPIO_Port GPIOD
#define GPIO_D6_Pin GPIO_PIN_13
#define GPIO_D6_GPIO_Port GPIOD
#define GPIO_START_BTN_Pin GPIO_PIN_4
#define GPIO_START_BTN_GPIO_Port GPIOG
#define GPIO_RTD_BUZZER_Pin GPIO_PIN_5
#define GPIO_RTD_BUZZER_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define GPIO_BRAKE_SW_Pin GPIO_PIN_8
#define GPIO_BRAKE_SW_GPIO_Port GPIOG
#define SDMMC1_DET_Pin GPIO_PIN_6
#define SDMMC1_DET_GPIO_Port GPIOC
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define GPIO_PUMP_Pin GPIO_PIN_10
#define GPIO_PUMP_GPIO_Port GPIOG
#define GPIO_ACC_FAN_Pin GPIO_PIN_11
#define GPIO_ACC_FAN_GPIO_Port GPIOG
#define GPIO_RAD_FAN_Pin GPIO_PIN_13
#define GPIO_RAD_FAN_GPIO_Port GPIOG
#define GPIO_BRAKE_LIGHT_Pin GPIO_PIN_14
#define GPIO_BRAKE_LIGHT_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
