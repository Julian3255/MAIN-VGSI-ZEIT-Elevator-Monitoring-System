/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wbxx_hal_gpio.h"
#include "stm32wbxx_hal_spi.h"
#include "stm32wbxx_it.h"

#include "mcp2515.h"
#include "CAN_TxRx.h"
// Yang Junyoung
#include "stm32_seq.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_IPCC_Init(void);
static void MX_RTC_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Variables for SysTick  */
int buttonState2, buttonState3;             // the current reading from the input pin
int lastButtonState2 = 1, lastButtonState3 = 1;   // the previous reading from the input pin
unsigned long lastDebounceTime2 = 0, lastDebounceTime3 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
int now = 0, then = 0;
int second = 0;


volatile int sw2_count = 0, sw3_count = 0;
volatile int tx_close_door = 0, tx_open_door = 0;
volatile int send = 0, send_open = 0, send_close = 0;
volatile int start_tx = 0;

/* Variables for CAN transmission*/
volatile uint16 rx_done = 0;
volatile uint16 tx_done = 0;
volatile uint16 send_rx = 0;
volatile int rx_flag = 0;
uint8 data_count = 0;
volatile uint8 rxLength;
volatile uint8 rxLength2;
uint8 TxDataAdrr[8] =
{
    MCP_TXB0_DATA0,
    MCP_TXB0_DATA1,
    MCP_TXB0_DATA2,
    MCP_TXB0_DATA3,
    MCP_TXB0_DATA4,
    MCP_TXB0_DATA5,
    MCP_TXB0_DATA6,
    MCP_TXB0_DATA7
};

uint8 rx_sidh = 0;
uint8 rx_sidl = 0;
uint8 rx_sidh2 = 0;
uint8 rx_sidl2 = 0;
uint8 rx_sidh_status1 = 0;
uint8 rx_sidh_status2 = 0;
uint8 rx_sidhl_status = 0;
uint16 rx_id = 0;
uint16 rx_id2 = 0;
uint8 func_code = 0;
uint8 base_adr = 0;


uint8 TxBufferData_SPI1[8];
uint8 TxBufferData_SPI2[8];


EMS_data    Master_EMS;
EMS_data    Slave_EMS_1;
extern EMS_Buffer Master;
extern EMS_Buffer Slave_1;


uint8 count_master = 1;
uint8 temp1;
volatile int data_flag = 0;
volatile int CAN_rx = 0;

uint8 tec = 0;
uint16 rec = 0;
uint8 eflg = 0;
uint8 can_inte = 0;
uint8 can_intf = 0;
uint8 can_intf2 = 0;
uint8 can_intf_clr = 0;
uint8 can_ctrl = 0;
uint8 rxb0ctrl = 0;
uint8 rxb1ctrl = 0;
uint8 canstat = 0;
uint8 status = 0;
uint16 rx_status = 0;


uint8 cnf1, cnf2, cnf3;
uint8 tx0_ctrl_b4 = 0;
uint8 tx0_ctrl = 0;
uint8 tx0_sidh = 0, tx0_sidl = 0;
uint8 tx0_dlc = 0;
uint8 tx0_data[8];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */


  count_master = 1;
  MCP2515_SPI1_CanInit();
  MCP2515_SPI2_CanInit();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //EMS_data Master_EMS;

  while (1)
  {
    // Yang Junyoung
    UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);
    count_master = 2;


  /* Read value from SW2 and SW3 to get open and close door signal */
  SW2_CloseButton_Scan();
  SW3_OpenButton_Scan();

  #if(TEST_FUNCTION == RTX_FUNCTION)
    /*  READ  */
    MCP2515_SPI1_ReadReg(MCP_CANINTF, &can_intf, 1);
    MCP2515_SPI2_ReadReg(MCP_CANINTF, &can_intf2, 1);

   if(!data_flag) {
    // If the interrupt for RX0B buffer is full -> extract the data
      if ((can_intf & (0x01))) {
          base_adr = 0x66;
          rx_done++;
      // Read the high-level and low-level address extracted from the CAN ID
        Read_CAN_ID();

        Read_RXdata(&rx_id, &base_adr);
        #if(EMS_TYPE == SLAVE_EMS) 
          Send_RXdata(&rx_id);
          Read_TXdata(SPI_CHANNEL_2);
          CAN_rx++;
		#endif

      // Clear the interrupt flags
        MCP2515_SPI1_RegModify(MCP_CANINTF, 0xFF, 0x00);
      }
	  #if(EMS_TYPE == MASTER_EMS)
		  else if ((can_intf2 & (0x01))) {
			  Read_CAN2_ID();
			  Read_Slave1_RXdata(&rx_id2, &base_adr);
			  MCP2515_SPI2_RegModify(MCP_CANINTF, 0xFF, 0x00);
		  }
	  #endif
      data_flag = 1;
   }

  now = HAL_GetTick();
  if(now >= 10) {
    data_flag = 0;
  }

    /*  SEND  */
//    if((send) & (start_tx)) {
//    /* Set TXREQ to initate message transmission and clear error bits in TXB0CTRL */
//    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, MCP_TXB_ABTF_M | MCP_TXB_MLOA_M \
//        | MCP_TXB_TXERR_M | MCP_TXB_TXREQ_M, 0x00);
//    MCP2515_SPI2_RegModify(MCP_TXB0CTRL, 0x03, 0xFF);
//
//    /* Load the High and Low address and DLC byte length */
//    MCP2515_SPI2_WriteReg(MCP_TXB0SIDH, 0x40, 1);  // 0x200 = 0010 0000 0000 -> 0100 0000    = 0x40
//    MCP2515_SPI2_WriteReg(MCP_TXB0SIDL, 0x00, 1);   // 000 = 0x00
//    MCP2515_SPI2_WriteReg(MCP_TXB0DLC, 0x04, 1); // change to corespoind byte length
//
//
//    /* Load the buffer data bytes */
//    if(tx_open_door) {
//      Send_OpenDoor();
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//      send_open++;
//    }
//
//    if(tx_close_door) {
//        Send_CloseDoor();
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//        send_close++;
//    }
//
//      /* If the TXREQ bit in TX0CTRL is set -> Transmission requested
//        Increment the tx_done counter to evaluate number of transmission
//      */
//      MCP2515_SPI2_WriteReg(MCP_TXB0CTRL, 0x08, 0x08);
//      // MCP2515_SPI2_ReadStatus(&status);
//      // MCP2515_SPI2_ReadReg(MCP_TXB0CTRL, &tx0_ctrl, 1);
//      // Read the data bytes in the frame
//      Read_TXdata(SPI_CHANNEL_1);
//      if(status & (0x04)) {
//        tx_done++;
//        status = 0;
//      }
//
//    // Reset send request to prevent unwanted transmission
//    send = 0;
//    }
  #endif
  }
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */


  /* USER CODE END Smps */
}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */


  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */


  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */


  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */


  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */


  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */


  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */


  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */


  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */


  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */


  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */


  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */


  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */


  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */


  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */


  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */


  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */


  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */


  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|EMS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|GPIO_PIN_12|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EMS_CS_Pin */
  GPIO_InitStruct.Pin = EMS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EMS_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_2_Pin Button_3_Pin */
  GPIO_InitStruct.Pin = Button_2_Pin|Button_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int SW2_CloseButton_Scan(void) {
    /* Button SW2 debounce*/
      int reading = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0);


      // check to see if you just pressed the button
      // (i.e. the input went from LOW to HIGH), and you've waited long enough
      // since the last press to ignore any noise:


      // If the switch changed, due to noise or pressing:
      if (reading != lastButtonState2) {
        // reset the debouncing timer
        lastDebounceTime2 = HAL_GetTick();
      }


      if ((HAL_GetTick() - lastDebounceTime2) > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:


        // if the button state has changed:
        if (reading != buttonState2) {
          buttonState2 = reading;


          // only toggle the LED if the new button state is HIGH
        if (buttonState2 == 1) {
          sw2_count++;
          tx_close_door = 1;
          tx_open_door = 0;
          send = 1;
          if (sw2_count == 2) {
            start_tx = 1;
            }
          }
        }
      }
      // save the reading. Next time through the loop, it'll be the lastButtonState:
      lastButtonState2 = reading;
    return tx_close_door;
}


int SW3_OpenButton_Scan(void) {
    /* Button SW3 debounce*/
      int reading = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);


      // check to see if you just pressed the button
      // (i.e. the input went from LOW to HIGH), and you've waited long enough
      // since the last press to ignore any noise:


      // If the switch changed, due to noise or pressing:
      if (reading != lastButtonState3) {
        // reset the debouncing timer
        lastDebounceTime3 = HAL_GetTick();
      }


      if ((HAL_GetTick() - lastDebounceTime3) > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:


        // if the button state has changed:
        if (reading != buttonState3) {
          buttonState3 = reading;


          // only toggle the LED if the new button state is HIGH
        if (buttonState3 == 1) {
          sw3_count++;
          tx_close_door = 0;
          tx_open_door = 1;
          send = 1;
          if (sw3_count == 2) {
              start_tx = 1;
          }
        }
      }
    }
      // save the reading. Next time through the loop, it'll be the lastButtonState:
      lastButtonState3 = reading;
    return tx_open_door;
}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(hspi);
//  if(hspi->Instance == SPI2) {
//    //HAL_SPI_Receive_IT(&hspi, &spi2_data, 1);
//    spi2_count++;
//  }
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_SPI_RxCpltCallback should be implemented in the user file
//   */
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
