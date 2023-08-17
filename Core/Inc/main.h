/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TEST_FUNCTION 		RTX_FUNCTION
#define TX_FUNCTION 		(1u)
#define RX_FUNCTION			(2u)
#define EMS_COM         	(3u)
#define RTX_FUNCTION		(4u)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define EMS_CS_Pin GPIO_PIN_9
#define EMS_CS_GPIO_Port GPIOA
#define B1_Pin GPIO_PIN_4
#define B1_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOB
#define Button_2_Pin GPIO_PIN_0
#define Button_2_GPIO_Port GPIOD
#define Button_3_Pin GPIO_PIN_1
#define Button_3_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Macros for LED manipulation */
#define LED_ON GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET
#define FLASH_FREQUENCY (500u)

#define RED_LED_ON() (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, LED_ON))
#define RED_LED_OFF() (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, LED_OFF))
#define BLUE_LED_ON() (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_BLUE_Pin, LED_ON))
#define BLUE_LED_OFF() (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_BLUE_Pin, LED_OFF))
#define GREEN_LED_ON() (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_GREEN_Pin, LED_ON))
#define GREEN_LED_OFF() (HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_GREEN_Pin, LED_OFF))

/*Macros for SPI operation*/
/* SPI related variables */
extern SPI_HandleTypeDef        hspi1;
#define SPI_CAN                 &hspi1
#define MCP_CS_OFF()          (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET))
#define MCP_CS_ON()          (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET))
 #define SPI_TIMEOUT         (100u)
// extern SPI_HandleTypeDef        hspi2;
// #define SPI_CAN                 &hspi2
// #define MCP_CS_OFF()          (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET))
// #define MCP_CS_ON()          (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
