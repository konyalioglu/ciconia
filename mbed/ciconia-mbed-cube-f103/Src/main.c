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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
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

uint16_t ICValue4_Rising;
uint16_t ICValue4_Falling;
uint8_t  is_rising4_captured = 0;
uint16_t PWM4;


uint16_t ANGLE_POS_0;
uint16_t ANGLE_POS_1;
uint16_t ANGLE_POS_2;
uint16_t ANGLE_POS_3;
uint16_t ANGLE_POS_4;

uint8_t rxbuffer[100];
uint8_t rxtempbuffer;
uint8_t rxindex;


uint8_t id_array[5] = {0,1,2,3,4};
uint16_t min_angle_data[5] = {ID0_MIN_POS, ID1_MIN_POS,ID2_MIN_POS,ID3_MIN_POS,ID4_MIN_POS};
uint16_t max_angle_data[5] = {ID0_MAX_POS, ID1_MAX_POS, ID2_MAX_POS, ID3_MAX_POS, ID4_MAX_POS};
uint16_t data_array[5] = {ID0_INIT_POS, ID1_INIT_POS, ID2_INIT_POS, ID3_INIT_POS, ID4_INIT_POS};

uint8_t drop1_counter = 0;
uint8_t drop2_counter = 0;
uint8_t drop1_flag = 0;
uint8_t drop2_flag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(10000);


  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  HAL_HalfDuplex_EnableReceiver(&huart3);



  Dynamixel_LED_ON(10, &huart3);
  Dynamixel_LED_ON(11, &huart3);

  Dynamixel_Set_MIN_Angle (ID10_MIN_POS, 10, &huart3);
  Dynamixel_Set_MAX_Angle (ID10_MAX_POS, 10, &huart3);

  Dynamixel_Set_Group_MIN_Servo_Angle(&huart3, min_angle_data, id_array, 5);
  Dynamixel_Set_Group_MAX_Servo_Angle(&huart3, max_angle_data, id_array, 5);
  Dynamixel_Set_Group_LED_ON(&huart3, id_array, 5);

  Dynamixel_Set_Group_Torque_Enable(&huart3, id_array, 5);
  Dynamixel_Set_Group_Servo_Angle(&huart3, data_array, id_array, 5);

  HAL_Delay(500);

  HAL_TIM_Base_Start_IT(&htim3);

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim4){

	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	  {
		  ICValue1_Rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	  }
	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	  {
		  ICValue1_Falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		  if (ICValue1_Falling > ICValue1_Rising)
		  {
			  PWM1 = ICValue1_Falling - ICValue1_Rising;
		  }
		  else if (ICValue1_Falling < ICValue1_Rising && ICValue1_Rising + ICValue1_Falling < (htim4.Init.Period + 1) / 2 ){

			  PWM1 = ICValue1_Rising - ICValue1_Falling;
		  }
		  else
		  {
			  PWM1 = (htim4.Init.Period + 1 - ICValue1_Rising) + ICValue1_Falling;
		  }

		  if (PWM1 < 1000 || PWM1 > 2000){
			  PWM1 = 1500;
		  }

	  }


	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	  {
		  ICValue2_Rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);

	  }
	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	  {
		  ICValue2_Falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		  if (ICValue2_Falling > ICValue2_Rising)
		  {
			  PWM2 = ICValue2_Falling-ICValue2_Rising;
		  }
		  else if (ICValue2_Falling < ICValue2_Rising && ICValue2_Rising + ICValue2_Falling < (htim4.Init.Period + 1) / 2 ){

			  PWM2 = ICValue2_Rising - ICValue2_Falling;
		  }
		  else
		  {
			  PWM2 = (htim4.Init.Period + 1 - ICValue2_Rising) + ICValue2_Falling;
		  }

		  if (PWM2 < 1000 || PWM2 > 2000){
			  PWM2 = 1500;
		  }
	  }
  }

  if (htim == &htim2){

	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	  {
		  ICValue3_Rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	  }
	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	  {
		  ICValue3_Falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		  if (ICValue3_Falling > ICValue3_Rising)
		  {
			  PWM3 = ICValue3_Falling-ICValue3_Rising;
		  }
		  else if (ICValue3_Falling < ICValue3_Rising && ICValue3_Rising + ICValue3_Falling < (htim2.Init.Period + 1) / 2 ){
			  PWM3 = ICValue3_Rising - ICValue3_Falling;
		  }
		  else
		  {
			  PWM3 = (htim4.Init.Period + 1 - ICValue3_Rising) + ICValue3_Falling;
		  }

		  if (PWM3 < 1000 || PWM3 > 2000){
			  PWM3 = 1500;
		  }

	  }


	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	  {
		  ICValue4_Rising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
	  }
	  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	  {
		  ICValue4_Falling = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		  if (ICValue4_Falling > ICValue4_Rising)
		  {
			  PWM4 = ICValue4_Falling - ICValue4_Rising;
		  }
		  else if (ICValue4_Falling < ICValue4_Rising && ICValue4_Rising + ICValue4_Falling < (htim4.Init.Period + 1) / 2 ){
			  PWM4 = ICValue4_Rising - ICValue4_Falling;
		  }
		  else
		  {
			  PWM4 = (htim4.Init.Period + 1 - ICValue4_Rising) + ICValue4_Falling;
		  }

		  if (PWM4 < 500 || PWM4 > 2500){
			  PWM4 = 1500;
		  }
	  }
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	  if (htim == &htim3){
		  data_array[0] = (PWM1 - PWM_NOM) + ID0_INIT_POS;
		  data_array[1] = (PWM1 - PWM_NOM) + ID1_INIT_POS;
		  data_array[2] = (PWM2 - PWM_NOM) + ID2_INIT_POS;
		  data_array[3] = (PWM3 - PWM_NOM) + ID3_INIT_POS;
		  data_array[4] = -(PWM3 - PWM_NOM) + ID4_INIT_POS;
		  Dynamixel_Set_Group_Servo_Angle(&huart3, data_array, id_array, 5);


		  if (drop1_flag == 1) {
			  drop1_counter++;
		  }

		  if (PWM4 > 1600 && drop1_flag == 0){

			  drop1_flag = 1;
			  Dynamixel_Torque_Enable (10, &huart3);

		  }

		  if (drop1_counter == 10){
			  Dynamixel_Set_Servo_Angle (ID10_MAX_POS, 10, &huart3);
		  }

		  if (drop1_counter == 50){
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		  }


		  if (drop1_counter == 100){
			  Dynamixel_Set_Servo_Angle(ID10_FIN_POS, 10, &huart3);
		  }

		  if (drop1_counter == 200){
			  Dynamixel_Set_Servo_Angle(ID10_INIT_POS, 10, &huart3);
		  }

		  if (drop1_counter > 250) {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			  drop1_flag = 0;
			  drop1_counter = 0;
			  Dynamixel_Torque_Disable(10, &huart3);
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
