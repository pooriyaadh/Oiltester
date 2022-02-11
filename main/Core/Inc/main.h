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
#include "stm32f0xx_hal.h"

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
#define SENOLIODVALUE_Pin GPIO_PIN_14
#define SENOLIODVALUE_GPIO_Port GPIOC
#define PUMP_RELAY_Pin GPIO_PIN_15
#define PUMP_RELAY_GPIO_Port GPIOC
#define PT100_OIL_Pin GPIO_PIN_0
#define PT100_OIL_GPIO_Port GPIOA
#define PT100_WATER_Pin GPIO_PIN_1
#define PT100_WATER_GPIO_Port GPIOA
#define PT100_STEAM_Pin GPIO_PIN_2
#define PT100_STEAM_GPIO_Port GPIOA
#define OP_HEATER_OIL_Pin GPIO_PIN_3
#define OP_HEATER_OIL_GPIO_Port GPIOA
#define RELAY_HEATER_OIL_Pin GPIO_PIN_4
#define RELAY_HEATER_OIL_GPIO_Port GPIOA
#define OP_HEATER_WATER_Pin GPIO_PIN_5
#define OP_HEATER_WATER_GPIO_Port GPIOA
#define jackDrive_Pin GPIO_PIN_6
#define jackDrive_GPIO_Port GPIOA
#define RELAY_HEATER_WATER_Pin GPIO_PIN_0
#define RELAY_HEATER_WATER_GPIO_Port GPIOB
#define vacumm_relay_Pin GPIO_PIN_1
#define vacumm_relay_GPIO_Port GPIOB
#define RS1_EN_Pin GPIO_PIN_15
#define RS1_EN_GPIO_Port GPIOB
#define RS2_EN_Pin GPIO_PIN_8
#define RS2_EN_GPIO_Port GPIOA
#define USup_Pin GPIO_PIN_6
#define USup_GPIO_Port GPIOF
#define USdown_Pin GPIO_PIN_7
#define USdown_GPIO_Port GPIOF
#define SPI_CS_PIN_Pin GPIO_PIN_15
#define SPI_CS_PIN_GPIO_Port GPIOA
#define jack_relay1_Pin GPIO_PIN_8
#define jack_relay1_GPIO_Port GPIOB
#define jack_relay2_Pin GPIO_PIN_9
#define jack_relay2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef enum
{
	device_on = 0x01,
	device_off = 0x02,
	device_reply = 0x03,
	device_set_point = 0x04,
}Device_State;
typedef struct
{
	Device_State  state;
	GPIO_TypeDef* port;
	uint16_t      pin;
}relay_def;
typedef struct{
	Device_State status;
}system_def;

extern	system_def system;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
