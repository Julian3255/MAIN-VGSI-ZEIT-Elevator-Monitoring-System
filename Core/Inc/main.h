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
#define EMS_TYPE          MASTER_EMS
#define MASTER_EMS        (0u)
#define SLAVE_EMS         (1u)

#define SPI_CHANNEL_1       (1u)
#define SPI_CHANNEL_2       (2u)

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
/* 
Define data type for the time os EMS, which includes:
  - second
  - minute
  - hour
  - day
  - weekday
  -year
*/
typedef struct 
{
    int second;
    int minute;
    int hour; 
    int day;
    int weekday;
    int year;  
} time;

/* 
Define data type for each EMS, which includes:
  - time
  - chosen floor
  - current floor
  - door status (opened = 0, closed = 1)
  - open door button status (unpressed = 0, pressed = 1)
  - close door button status (unpressed = 0, pressed = 1)
*/
typedef struct 
{
    time EMS_time;
    int chosen_floor;
    int curren_floor;
    int door_status;
    int open_door_button_stat;
    int close_door_button_stat;
    int up_button_status;
    int down_button_status;
} EMS_data; 

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
#define DOOR_OPENED   (0)
#define DOOR_CLOSED   (1)
#define BUTTON_OFF    (0)
#define BUTTON_ON    (1)

/*Macros for SPI operation*/
/* SPI related variables for handling MCP2515 CAN-SPI module */
extern SPI_HandleTypeDef        hspi1;
#define SPI_CAN                 &hspi1
#define SPI1_CS_OFF()          (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET))
#define SPI1_CS_ON()          (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET))

extern SPI_HandleTypeDef        hspi2;
#define SPI_EMS                &hspi2
#define SPI2_CS_OFF()          (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET))
#define SPI2_CS_ON()          (HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET))

 #define SPI_TIMEOUT         (100u)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
