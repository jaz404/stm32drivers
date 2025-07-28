/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: Jaspreet Chhabra
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_MAX 4095
#define REVERSE_PIN GPIO_PIN_5
#define REVERSE_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_HSE(uint8_t clock_freq);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void CAN1_Tx();
void CAN1_Rx(void);
void CAN_Filter_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t motor_speed = 0;
CAN_RxHeaderTypeDef RxHeader;
volatile uint8_t crawl_mode = 0;  // 0 = off, 1 = on

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config_HSE(50);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();
  // Reverse PIN
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Forward by default
  CAN1_Tx();
  // TEST DAC
//  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//
//   uint16_t dac_values[] = {
//     0,                       // 0%
//     4095 * 0.10,             // 10%
//     4095 * 0.30,             // 30%
//     4095 * 0.50,             // 50%
//     4095 * 0.70,             // 70%
//     4095                    // 100%
//   };
//
//   for (int i = 0; i < sizeof(dac_values)/sizeof(dac_values[0]); i++) {
//     HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_values[i]);
//     HAL_Delay(3000); // wait 3 seconds
//   }

  	if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK)
		{
		Error_Handler();
		}

	if( HAL_CAN_Start(&hcan1) != HAL_OK)
		{
		Error_Handler();
		}
	// Start the TIMER interrupt
	HAL_TIM_Base_Start_IT(&htim6);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  if (crawl_mode)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Forward
		  HAL_Delay(3000);

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Reverse
		  HAL_Delay(3000);
	  }

	  // Always check CAN messages
//	  HAL_CAN_Polling_IRQHandler(&hcan1); // or HAL_CAN_IRQHandler if using interrupts
//	  HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config_HSE(uint8_t clock_freq)
{
  RCC_OscInitTypeDef Osc_Init;
  RCC_ClkInitTypeDef Clock_Init;
  uint8_t flash_latency=0;

  Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE ;
  Osc_Init.HSEState = RCC_HSE_ON;
  Osc_Init.PLL.PLLState = RCC_PLL_ON;
  Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  switch(clock_freq) {
  case SYS_CLOCK_FREQ_50_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 50;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
    flash_latency = 1;
    break;

  case SYS_CLOCK_FREQ_84_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 84;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;
    flash_latency = 2;
    break;

  case SYS_CLOCK_FREQ_120_MHZ:
    Osc_Init.PLL.PLLM = 4;
    Osc_Init.PLL.PLLN = 120;
    Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
    Osc_Init.PLL.PLLQ = 2;
    Osc_Init.PLL.PLLR = 2;
    Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
    Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
    Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;
    flash_latency = 3;
    break;

  default:
    return ;
  }

  if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_RCC_ClockConfig(&Clock_Init, flash_latency) != HAL_OK)
  {
    Error_Handler();
  }

  /*Configure the systick timer interrupt frequency (for every 1 ms) */
  uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
  HAL_SYSTICK_Config(hclk_freq/1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
//  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
//  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
//  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
//  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

  hcan1.Init.Prescaler = 10;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
//  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim6.Init.Period = 65535;
  htim6.Init.Prescaler = 49999;  // Divides 50MHz to 1kHz
  htim6.Init.Period = 4999;      // 1kHz → 5s interrupt

  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Set default state: HIGH = Forward direction */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Transmit a message via CAN1.
  * @retval None
  */
void CAN1_Tx()
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t message[2];  // Two-byte message

  TxHeader.DLC = 2;  // Data length = 2 bytes
  TxHeader.StdId = 0x399;
  TxHeader.IDE   = CAN_ID_STD;
  TxHeader.RTR   = CAN_RTR_DATA;

  message[0] = crawl_mode ? 1 : 0;
  message[1] = (motor_speed == -1) ? 255 : (uint8_t)motor_speed;

  char dbg[64];
  sprintf(dbg, "Tx: crawl=%d, speed=%d\r\n", message[0], message[1]);
  HAL_UART_Transmit(&huart2, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message, &TxMailbox) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  Configures the CAN filter.
  * @retval None
  */
void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef can1_filter_init;

  can1_filter_init.FilterActivation = ENABLE;
  can1_filter_init.FilterBank  = 0;
  can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
  // CANid total bits 11
  // xxx xxxx xxxx
  // 100 xxxx xxxx
  // Accept only 4XX
  // id 1000 = 0x8
  // mask 1110 = 0xE
  can1_filter_init.FilterIdHigh = 0x0000;
  can1_filter_init.FilterIdLow = 0x0000;
  can1_filter_init.FilterMaskIdHigh = 0X0000;
  can1_filter_init.FilterMaskIdLow = 0x0000;
  can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
  can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

  if( HAL_CAN_ConfigFilter(&hcan1,&can1_filter_init) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
  char msg[50];
  sprintf(msg,"Message Transmitted:M0\r\n");
  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
  char msg[50];
  sprintf(msg,"Message Transmitted:M1\r\n");
  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

/**
  * @brief  Transmission Mailbox 2 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
  char msg[50];
  sprintf(msg,"Message Transmitted:M2\r\n");
  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t rcvd_msg[8];
	char dbg_rx[64];
	sprintf(dbg_rx, "Rx: ID=0x%03lX, Data[0]=%d\r\n", RxHeader.StdId, rcvd_msg[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*)dbg_rx, strlen(dbg_rx), HAL_MAX_DELAY);

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rcvd_msg) != HAL_OK)
    {
        Error_Handler();
    }

    if (RxHeader.StdId >= 0x301 && RxHeader.StdId <= 0x306)
    {
        crawl_mode = 0;
    }


    if (RxHeader.StdId == 0x301 && RxHeader.RTR == 0)
    {   // STOP
        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_SET); // ensure forward
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
        motor_speed = 0;
    }
    else if (RxHeader.StdId == 0x302 && RxHeader.RTR == 0)
    {   // 10%
        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_SET); // ensure forward
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.10 * DAC_MAX);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
        motor_speed = 10;
    }
    else if (RxHeader.StdId == 0x303 && RxHeader.RTR == 0)
    {   // 30%
        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_SET); // ensure forward
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.30 * DAC_MAX);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
        motor_speed = 30;
    }
    else if (RxHeader.StdId == 0x304 && RxHeader.RTR == 0)
    {   // 50%
        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_SET); // ensure forward
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.50 * DAC_MAX);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
        motor_speed = 50;
    }
    else if (RxHeader.StdId == 0x305 && RxHeader.RTR == 0)
    {   // 70%
        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_SET); // ensure forward
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.70 * DAC_MAX);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
        motor_speed = 70;
    }
    else if (RxHeader.StdId == 0x306 && RxHeader.RTR == 0)
    {   // 100%
        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_SET); // ensure forward
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_MAX);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
        motor_speed = 100;
    }
    else if (RxHeader.StdId == 0x3FF && RxHeader.RTR == 0)
    {   // Reverse ON
        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_RESET); // LOW = reverse
        motor_speed = -1;
    }
//		Redundant
//    else if (RxHeader.StdId == 0x321 && RxHeader.RTR == 0)
//    {   // Reverse OFF
//        HAL_GPIO_WritePin(REVERSE_PORT, REVERSE_PIN, GPIO_PIN_SET); // HIGH = forward
//    }
    else if (RxHeader.StdId == 0x310 && RxHeader.RTR == 0)
    {   // Enable crawl
        crawl_mode = 1;
        motor_speed = 20;  // example crawl speed
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0.20 * DAC_MAX);
        HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    }
//    else if (RxHeader.StdId >= 0x301 && RxHeader.StdId <= 0x306)
//    {
//        crawl_mode = 0;  // exit crawl if another speed is commanded
//    }
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * This is called whenever the timer interrupt is raised
  * We will send the heartbeat signal from this function
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ // broadcasts heartbeat signal
  CAN1_Tx();
}
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
#ifdef USE_FULL_ASSERT
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
