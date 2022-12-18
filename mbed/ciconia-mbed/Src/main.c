/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SBUS_BUF_LEN  25
#define ROS_BUF_LEN  21
#define SBUS_CHE_LEN  25
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

// SBUS DEFINITIONS
uint16_t   channel[SBUS_CHE_LEN];
uint8_t     buffer[SBUS_BUF_LEN];

uint8_t    sbus_buffer;
uint8_t    sbus_footer1 = 0x00;
uint8_t    sbus_footer2 = 0x04;
uint8_t    sbus_header = 0x0f;
uint8_t    sbus_state = 0;
uint8_t    sbus_i = 0;
uint8_t    end_line = 0x0d;

uint8_t    lostframe;
uint8_t    failsafe;

//ROS MESSAGE DEFINITIONS
uint8_t    ros_buffer[ROS_BUF_LEN];
uint8_t    ros_temp_buffer;
uint8_t    ros_header1 = 0xff;
uint8_t    ros_header2 = 0xff;

uint8_t    ros_footer1 = 0xfc;
uint8_t    ros_footer2 = 0xfc;

uint8_t    ros_uart_state = 0;
uint8_t    ros_uart_i = 0;
uint8_t    checksum = 0;


uint16_t   PWM1;
uint16_t   PWM2;
uint16_t   PWM3;
uint16_t   PWM4;
uint16_t   PWM5;

uint16_t   DAIL;
uint16_t   DELE;
uint16_t   DRUD;


uint16_t ICValue1_Rising;
uint16_t ICValue1_Falling;
uint8_t  is_rising1_captured = 0;
uint16_t PWM1;

uint16_t ICValue2_Rising;
uint16_t ICValue2_Falling;
uint8_t  is_rising2_captured = 0;
uint16_t PWM2;

uint16_t ICValue3_Rising;
uint16_t ICValue3_Falling;
uint8_t  is_rising3_captured = 0;
uint16_t PWM3;

uint8_t RUDDER_R_ID = 3;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

  //HAL_UART_Receive_DMA(&huart3, (uint8_t *)buffer, 25);
  HAL_UART_Receive_DMA(&huart3, &sbus_buffer, 1);
  HAL_UART_Receive_DMA(&huart1, &ros_temp_buffer, 1);

  //HAL_Delay(100);
  //Dynamixel_Set_Servo_Angle(40, RUDDER_R_ID, &huart6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 100;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 57600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3){
		HAL_UART_Receive_DMA(&huart3, &sbus_buffer, 1);
		if ( sbus_state == 0 && sbus_buffer == sbus_header ){
			buffer[sbus_i] = sbus_buffer;
			sbus_state = 1;
			sbus_i ++;
		}
		else if( sbus_state == 1 && sbus_i <= SBUS_BUF_LEN){
			buffer[sbus_i] = sbus_buffer;
			sbus_i ++;
			if ( sbus_i == SBUS_BUF_LEN ){
				sbus_state = 2;
			}
		}

		if ( sbus_state == 2 ){
			sbus_state = 0;
			sbus_i = 0;

			if (sbus_buffer == sbus_footer1 || ((sbus_buffer & sbus_header) == sbus_footer1)){

				channel[0] = (((uint16_t)buffer[1])
						| ((uint16_t)buffer[2] << 8)) & 0x07FF;
				channel[1] = (((uint16_t)buffer[2] >> 3)
						| ((uint16_t)buffer[3] << 5)) & 0x07FF;
				channel[2] =
						(((uint16_t)buffer[3] >> 6) | ((uint16_t)buffer[4] << 2)
								| ((uint16_t)buffer[5] << 10)) & 0x07FF;
				channel[3] = (((uint16_t)buffer[5] >> 1)
						| ((uint16_t)buffer[6] << 7)) & 0x07FF;
				channel[4] = (((uint16_t)buffer[6] >> 4)
						| ((uint16_t)buffer[7] << 4)) & 0x07FF;
				channel[5] = (((uint16_t)buffer[7] >> 7)
						| ((uint16_t)buffer[8] << 1) | ((uint16_t)buffer[9] << 9))
						& 0x07FF;
				channel[6] = (((uint16_t)buffer[9] >> 2)
						| ((uint16_t)buffer[10] << 6)) & 0x07FF;
				channel[7] = (((uint16_t)buffer[10] >> 5)
						| ((uint16_t)buffer[11] << 3)) & 0x07FF;
				channel[8] = (((uint16_t)buffer[12])
						| ((uint16_t)buffer[13] << 8)) & 0x07FF;
				channel[9] = (((uint16_t)buffer[13] >> 3)
						| ((uint16_t)buffer[14] << 5)) & 0x07FF;
				channel[10] = (((uint16_t)buffer[14] >> 6)
						| ((uint16_t)buffer[15] << 2)
						| ((uint16_t)buffer[16] << 10)) & 0x07FF;
				channel[11] = (((uint16_t)buffer[16] >> 1)
						| ((uint16_t)buffer[17] << 7)) & 0x07FF;
				channel[12] = (((uint16_t)buffer[17] >> 4)
						| ((uint16_t)buffer[18] << 4)) & 0x07FF;
				channel[13] = (((uint16_t)buffer[18] >> 7)
						| ((uint16_t)buffer[19] << 1)
						| ((uint16_t)buffer[20] << 9)) & 0x07FF;
				channel[14] = (((uint16_t)buffer[20] >> 2)
						| ((uint16_t)buffer[21] << 6)) & 0x07FF;
				channel[15] = (((uint16_t)buffer[21] >> 5)
						| ((uint16_t)buffer[22] << 3)) & 0x07FF;
				channel[17] = (uint16_t)(buffer[23] & 0x01);
				channel[18] = (uint16_t)(buffer[23] & 0x02);

				lostframe  = buffer[23] & 0x04;
				failsafe    = buffer[23] & 0x08;

				HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, SBUS_BUF_LEN);

				HAL_UART_Transmit_IT(&huart1, &end_line, 1);

			}
		}
	}
	if(huart == &huart1){

		HAL_UART_Receive_DMA(&huart1, &ros_temp_buffer, 1);

		if ( ros_uart_state == 0 && ros_temp_buffer == ros_header1 ){
			ros_buffer[ros_uart_i] = ros_temp_buffer;
			ros_uart_state = 1;
			ros_uart_i ++;
		}
		else if ( ros_uart_state == 1 && ros_temp_buffer == ros_header2 ){
			ros_buffer[ros_uart_i] = ros_temp_buffer;
			ros_uart_state = 2;
			ros_uart_i ++;
		}
		else if( ros_uart_state == 2 && ros_uart_i < ROS_BUF_LEN + 1){
			ros_buffer[ros_uart_i] = ros_temp_buffer;
			ros_uart_i ++;
			if ( ros_uart_i >= 2 && ros_uart_i <= 18 ){
				checksum += ros_temp_buffer;
			}
			if ( ros_uart_i == ROS_BUF_LEN ){
				ros_uart_state = 3;
			}
		}
		else{
			ros_uart_i = 0;
			ros_uart_state = 0;
		}

		if ( ros_uart_state == 3 ){
			ros_uart_state = 0;
			ros_uart_i = 0;

			if (ros_buffer[ROS_BUF_LEN-2] == ros_footer1 && ros_buffer[ROS_BUF_LEN-1] == ros_footer2 && ros_buffer[ROS_BUF_LEN-3] == checksum){

				PWM1 = (((uint16_t)ros_buffer[2]) | ((uint16_t)ros_buffer[3] << 8)) & 0xFFFF;
				PWM2 = (((uint16_t)ros_buffer[4]) | ((uint16_t)ros_buffer[5] << 8)) & 0xFFFF;
				PWM3 = (((uint16_t)ros_buffer[6]) | ((uint16_t)ros_buffer[7] << 8)) & 0xFFFF;
				PWM4 = (((uint16_t)ros_buffer[8]) | ((uint16_t)ros_buffer[9] << 8)) & 0xFFFF;
				PWM5 = (((uint16_t)ros_buffer[10]) | ((uint16_t)ros_buffer[11] << 8)) & 0xFFFF;

				DAIL = (((uint16_t)ros_buffer[12]) | ((uint16_t)ros_buffer[13] << 8)) & 0xFFFF;
				DELE = (((uint16_t)ros_buffer[14]) | ((uint16_t)ros_buffer[15] << 8)) & 0xFFFF;
				DRUD = (((uint16_t)ros_buffer[16]) | ((uint16_t)ros_buffer[17] << 8)) & 0xFFFF;

			}
			checksum = 0;
		}
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim4){
	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	  {
		if ( is_rising1_captured == 0 ){
			ICValue1_Rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			is_rising1_captured = 1;
		}
		else{
			ICValue1_Falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			is_rising1_captured = 0;

			if (ICValue1_Falling > ICValue1_Rising)
			{
				PWM1 = ICValue1_Falling-ICValue1_Rising;
			}
			else if (ICValue1_Falling < ICValue1_Rising && ICValue1_Rising + ICValue1_Falling < (htim4.Init.Period + 1) / 2 ){
				is_rising1_captured = 1;
				PWM1 = ICValue1_Rising - ICValue1_Falling;
			}
			else{
				PWM1 = (htim4.Init.Period + 1 - ICValue1_Rising) + ICValue1_Falling;
			}
		}
	  }

	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	  {
		if ( is_rising2_captured == 0 ){
			ICValue2_Rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			is_rising2_captured = 1;
		}
		else{
			ICValue2_Falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

			is_rising2_captured = 0;

			if (ICValue2_Falling > ICValue2_Rising)
			{
				PWM2 = ICValue2_Falling-ICValue2_Rising;
			}
			else if (ICValue2_Falling < ICValue2_Rising && ICValue2_Rising + ICValue2_Falling < (htim4.Init.Period + 1) / 2 ){
				is_rising2_captured = 1;
				PWM2 = ICValue2_Rising - ICValue2_Falling;
			}
			else
			{
				PWM2 = (htim4.Init.Period + 1 - ICValue2_Rising) + ICValue2_Falling;
			}
		}
	  }

	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	  {
		if ( is_rising3_captured == 0 ){
			ICValue3_Rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			is_rising3_captured = 1;
		}
		else{

			ICValue3_Falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			is_rising3_captured = 0;

			if (ICValue3_Falling > ICValue3_Rising)
			{
				PWM3 = ICValue3_Falling-ICValue3_Rising;
			}
			else if (ICValue3_Falling < ICValue3_Rising && ICValue3_Rising + ICValue3_Falling < (htim4.Init.Period + 1) / 2 ){
				is_rising3_captured = 1;
				PWM3 = ICValue3_Rising - ICValue3_Falling;
			}
			else{
				PWM3 = (htim4.Init.Period + 1 - ICValue3_Rising) + ICValue3_Falling;
			}
		}
	  }
  }
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

