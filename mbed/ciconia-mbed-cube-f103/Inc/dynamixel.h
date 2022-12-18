/*
 * dynamixel.h
 *
 *  Created on: Oct 17, 2022
 *      Author: turan
 */

#ifndef INC_DYNAMIXEL_H_
#define INC_DYNAMIXEL_H_



#endif /* INC_DYNAMIXEL_H_ */


/*
 * dynamixel.h
 *
 *  Created on: Oct 17, 2022
 *      Author: turan
 */

#ifndef INC_DYNAMIXEL_H_
#define INC_DYNAMIXEL_H_



#endif /* INC_DYNAMIXEL_H_ */


#include "main.h"


#define SHUTDOWN               18
#define TORQUE_ENABLE          24
#define PRESENT_SPEED          38
#define GOAL_POSITION          30
#define PRESENT_POSITION       36
#define PRESENT_TEMPRETURE 	   43
#define LED                    25
#define MAX_ANGLE_ADDRESS       8
#define MIN_ANGLE_ADDRESS 		6

#define GOAL_POSITION_SIZE     2
#define MAX_POSITION_SIZE      2
#define MIN_POSITION_SIZE      2
#define LED_SIZE               1
#define TORQUE_ENABLE_SIZE     1

#define H1 0xff
#define H2 0xff

#define SERVO_ID_POS 2
#define SERVO_LEN_POS 3
#define SERVO_ERROR_POS 4
#define SERVO_PARAM_POS 5

#define  PING              1
#define  READ              2
#define  WRITE             3
#define  SYNC_WRITE     0x83
#define  BROADCAST_ID   0xFE

extern UART_HandleTypeDef HAL;


void Dynamixel_LED_ON (const int ID, UART_HandleTypeDef *huart);


void Dynamixel_LED_OFF (const int ID, UART_HandleTypeDef *huart);


void Dynamixel_Torque_Enable (const int ID, UART_HandleTypeDef *huart);


void Dynamixel_Torque_Disable (const int ID, UART_HandleTypeDef *huart);


void Dynamixel_Set_Servo_Angle (const int angle, uint8_t ID, UART_HandleTypeDef *huart);


void Dynamixel_Set_MAX_Angle (const int angle, uint8_t ID, UART_HandleTypeDef *huart);


void Dynamixel_Set_MIN_Angle (const int angle, uint8_t ID, UART_HandleTypeDef *huart);


void Dynamixel_Send_Write_Command (const uint8_t commandByte,
					   UART_HandleTypeDef *huart,
					   const uint8_t ID,
		               const uint8_t N,
		               const uint8_t *params);


void Dynamixel_Send_Write_Command (const uint8_t commandByte,
					   UART_HandleTypeDef *huart,
					   const uint8_t ID,
		               const uint8_t N,
		               const uint8_t *params);



void Dynamixel_Set_Group_LED_ON(UART_HandleTypeDef *huart,
										 uint8_t id_array[],
										 uint8_t servo_number);



void Dynamixel_Set_Group_LED_OFF(UART_HandleTypeDef *huart,
										 uint8_t *id_array,
										 uint8_t servo_number);



void Dynamixel_Set_Group_Torque_Enable(UART_HandleTypeDef *huart,
										uint8_t *id_array,
										uint8_t servo_number);



void Dynamixel_Set_Group_Torque_Disable(UART_HandleTypeDef *huart,
										uint8_t *id_array,
										uint8_t servo_number);



void Dynamixel_Set_Group_Servo_Angle(UART_HandleTypeDef *huart,
									uint16_t *data,
									uint8_t *id_array,
									uint8_t servo_number);



void Dynamixel_Set_Group_MAX_Servo_Angle(UART_HandleTypeDef *huart,
									uint16_t *data,
									uint8_t *id_array,
									uint8_t servo_number);



void Dynamixel_Set_Group_MIN_Servo_Angle(UART_HandleTypeDef *huart,
									uint16_t *data,
									uint8_t *id_array,
									uint8_t servo_number);



void Dynamixel_Send_Sync_Write_Command (UART_HandleTypeDef *huart,
							   	   	   const uint8_t L,
									   const uint8_t N,
									   const uint8_t *params);


void send_byte(UART_HandleTypeDef *huart, uint8_t byte);
