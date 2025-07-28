/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <string.h>
#include <stdio.h>
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
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config_HSE(uint8_t clock_freq);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
void CAN1_Tx();
void CAN1_Rx(void);
void CAN_Filter_Config(void);
/* USER CODE BEGIN PFP */
uint16_t XY(uint8_t x, uint8_t y, uint8_t width, uint8_t serpentine);
void Set_LEDs_ByState(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_LED 64
#define USE_BRIGHTNESS 0

uint8_t fsm_state = 0; 	// initialize fault state
CAN_RxHeaderTypeDef RxHeader;

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

volatile uint8_t datasentflag   = 1;
volatile uint8_t state_changed = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	    {
	        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_2);
	        datasentflag = 1;              // allow next frame
	    }
}

void Set_LED(int LEDnum, int Red, int Green, int Blue)
{
    float brightness_scale = 0.4f;  // 40% brightness

    LED_Data[LEDnum][0] = LEDnum;
    LED_Data[LEDnum][1] = (int)(Green * brightness_scale);
    LED_Data[LEDnum][2] = (int)(Red * brightness_scale);
    LED_Data[LEDnum][3] = (int)(Blue * brightness_scale);
}


#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	if (!datasentflag) return;
	    datasentflag = 0;
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
//			if (color&(1<<i))
//			{
//				pwmData[indx] = 60;  // 2/3 of 90
//			}
//
//			else pwmData[indx] = 30;  // 1/3 of 90

			if (color & (1 << i))
			    pwmData[indx] = 42;  // 2/3 of 63
			else
			    pwmData[indx] = 21;  // 1/3 of 63


			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)pwmData, indx);
//	while (!datasentflag){};
//	datasentflag = 0;
}

void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}


// ported from the arduino code for 8 LEDs located at ->>>>  https://adrianotiger.github.io/Neopixel-Effect-Generator/

uint16_t effStep = 0;


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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();
  CAN1_Tx();

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

	HAL_Delay(1000);

	Reset_LED();          // give LED_Data something valid
	Set_LEDs_ByState();   // first frame (state 0, red comet)

//  uint8_t fsm_state = 0;        // Start from state 0
//   uint16_t MAX_LED = 64;     // Total number of LEDs
//   uint32_t last_change_time = 0;
//   uint32_t state_duration_ms = 5000;  // 5 seconds per state
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  Set_LEDs_ByState();
//	  	        effStep++;

	  	        // Change state every 5 seconds
	  //	        if (HAL_GetTick() - last_change_time >= state_duration_ms) {
	  //	            fsm_state = (fsm_state + 1) % 7;  // Cycle 0→1→...→6→0
	  //	            last_change_time = HAL_GetTick();
	  //	        }

//	  HAL_Delay(30);

	  effStep++;                               // animation cursor

//	     if (state_changed)                       // repaint only when needed
//	     {
//	         state_changed = 0;
//	                          // builds colour table + WS2812_Send()
//	     }
	     Set_LEDs_ByState();

	     HAL_Delay(30);
    /* USER CODE END WHILE */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // configure GPIOs
    GPIO_InitTypeDef ledgpio;
    ledgpio.Pin = GPIO_PIN_5;
    ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
    ledgpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA,&ledgpio);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* === LED pattern generator  (internal WS2812_Send() removed) ============ */
void Set_LEDs_ByState(void)
{
    uint8_t  trail_len = MAX_LED / 2;
    uint16_t head      = effStep % MAX_LED;

    for (uint16_t i = 0; i < MAX_LED; ++i) {
        uint16_t distance = (i + MAX_LED - head) % MAX_LED;

        switch (fsm_state) {
        case 0:  Set_LED(i, distance < trail_len ? 255 : 0, 0, 0);                break;
        case 1:  Set_LED(i, 0, distance < trail_len ? 255 : 0, 0);                break;
        case 2:  Set_LED(i, 0, 0, distance < trail_len ? 255 : 0);                break;
        case 3:  Set_LED(i, 0, 0, 0);                                             break;

        case 4: {                              /* moving split blue / yellow   */
            uint16_t half  = MAX_LED / 2;
            uint16_t start = effStep % MAX_LED;
            uint16_t blue  = (start            + i) % MAX_LED;
            uint16_t yel   = (start + half     + i) % MAX_LED;
            Set_LED(blue, 0, 0, 255);
            Set_LED(yel , 255, 255, 0);
        } break;

        case 5:                                /* blinking blue / yellow comet */
            if ((effStep / 20) & 1) { Set_LED(i,0,0,0); break; }
            if (distance < trail_len)
                Set_LED(i, (i < MAX_LED/2) ? 0 : 255,
                           (i < MAX_LED/2) ? 0 : 255,
                           (i < MAX_LED/2) ? 255 : 0);
            else Set_LED(i,0,0,0);
            break;

        case 6:                                /* blinking yellow comet        */
            if ((effStep / 20) & 1 || distance >= trail_len)
                Set_LED(i,0,0,0);
            else
                Set_LED(i,255,255,0);
            break;
        }
    }
    WS2812_Send();      /* single call for all states */
}

/**
  * @brief  Transmit a message via CAN1.
  * @retval None
  */
void CAN1_Tx()
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;
  uint8_t message;

  TxHeader.DLC = 1;
  TxHeader.StdId = 0x499;
  TxHeader.IDE   = CAN_ID_STD;

  TxHeader.RTR = CAN_RTR_DATA;

  message = fsm_state;

  // board led blinks on tx
  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
  // send the message

  if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&message,&TxMailbox) != HAL_OK)
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
//  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
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
//  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
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
//  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
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
//	HAL_UART_Transmit(&huart2, (uint8_t*)dbg_rx, strlen(dbg_rx), HAL_MAX_DELAY);


  char msg[50];

  if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,rcvd_msg) != HAL_OK)
  {
    Error_Handler();
  }

  switch (RxHeader.StdId)
      {
          case 0x401:  fsm_state = 0; break;   // FAULT
          case 0x402:  fsm_state = 1; break;   // SAFE‑TO‑APPROACH
          case 0x403:  fsm_state = 2; break;   // READY‑TO‑LAUNCH
          case 0x404:  fsm_state = 3; break;   // LAUNCH (unused)
          case 0x405:  fsm_state = 4; break;   // CRAWLING
          case 0x406:  fsm_state = 5; break;   // BRAKING
          case 0x407:  fsm_state = 6; break;   // DEBUG
          default: return;                     // ignore everything else
      }
      state_changed = 1;                       // tell main loop to repaint
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
