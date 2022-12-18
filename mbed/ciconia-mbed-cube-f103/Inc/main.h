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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dynamixel.h"
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
/* USER CODE BEGIN Private defines */
//
//DYNAMIXEL SERVO PARAMETERS
//
//AILERON  PWM->SERVO OUPUT #1 -> TIM4_CH1 and TIM4_CH2 (PB6 - PB7)
//ELEVATOR PWM->SERVO OUPUT #2 -> TIM4_CH3 and TIM4_CH4 (PB8 - PB9)
//RUDDER  PWM->SERVO OUPUT #4 -> TIM2_CH1 and TIM4_CH2 (PA0 - PA1)
//DROP MECH.  PWM->SERVO OUPUT AUX #1 -> TIM2_CH3 and TIM4_CH4 (PA2 - PA3)
//
//
//ID 2: ELEVATOR MAX ANGLE:312 (3550 - 0xDDE) MIN ANGLE:205(2342 - 0x9926)
//ID 3: RUDDER RIGHT MAX ANGLE: 276 (3140 - 0x0C44) MIN ANGLE: 173 (1970 - 0x7B2)
//ID 4: RUDDER LEFT MAX ANGLE:360(4095 - 0x0FFF)  MIN ANGLE:270(3080 - 0xC08)
// Half-Duplex UART Comm (Dynamixel Servos) -> PB10

#define ID0_INIT_POS		1688
#define ID0_MIN_POS			ID0_INIT_POS - 400
#define ID0_MAX_POS		    ID0_INIT_POS + 400

#define ID1_INIT_POS        2018
#define ID1_MIN_POS		    ID1_INIT_POS - 400
#define ID1_MAX_POS		    ID1_INIT_POS + 400

#define ID2_INIT_POS        2950
#define ID2_MIN_POS		    ID2_INIT_POS - 400
#define ID2_MAX_POS		    ID2_INIT_POS + 400

#define ID3_INIT_POS		2538
#define ID3_MIN_POS		    ID3_INIT_POS - 400
#define ID3_MAX_POS		    ID3_INIT_POS + 400

#define ID4_INIT_POS		3600
#define ID4_MIN_POS			ID4_INIT_POS - 400
#define ID4_MAX_POS		    ID4_INIT_POS + 400

#define ID10_INIT_POS		650
#define ID10_MIN_POS		442
#define ID10_MAX_POS		680
#define ID10_FIN_POS		442

#define PWM_INPUT_MAX       1900
#define PWM_INPUT_MIN 		1100
#define PWM_NOM 			1500
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
