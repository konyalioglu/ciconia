/*
 * dynamixel.c
 *
 *  Created on: Oct 17, 2022
 *      Author: turan
 */



#include "dynamixel.h"



void Dynamixel_Torque_Enable (const int ID, UART_HandleTypeDef *huart)
{


    const uint8_t params[2] = {TORQUE_ENABLE,
                               1};

    Dynamixel_Send_Write_Command(WRITE, huart, ID, TORQUE_ENABLE_SIZE, params);


}


void Dynamixel_Torque_Disable (const int ID, UART_HandleTypeDef *huart)
{


    const uint8_t params[2] = {TORQUE_ENABLE,
                               0};

    Dynamixel_Send_Write_Command(WRITE, huart, ID, TORQUE_ENABLE_SIZE, params);


}



void Dynamixel_LED_ON (const int ID, UART_HandleTypeDef *huart)
{


    const uint8_t params[2] = {LED,
                               1};

    Dynamixel_Send_Write_Command(WRITE, huart, ID, LED_SIZE, params);


}



void Dynamixel_LED_OFF (const int ID, UART_HandleTypeDef *huart)
{


    const uint8_t params[2] = {LED,
                               0};

    Dynamixel_Send_Write_Command(WRITE, huart, ID, LED_SIZE, params);


}



void Dynamixel_Set_Servo_Angle (const int angle, uint8_t ID, UART_HandleTypeDef *huart)
{

    if (angle > 0 || angle < 0xfff){



    const uint8_t highByte = (uint8_t)((angle >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angle & 0xff);



    const uint8_t params[3] = {GOAL_POSITION,
                               lowByte,
                               highByte};


    Dynamixel_Send_Write_Command(WRITE, huart, ID, GOAL_POSITION_SIZE, params);


    }
}



void Dynamixel_Set_MIN_Angle (const int angle, uint8_t ID, UART_HandleTypeDef *huart)
{

    if (angle > 0 || angle < 0xfff){



    const uint8_t highByte = (uint8_t)((angle >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angle & 0xff);



    const uint8_t params[3] = {MIN_ANGLE_ADDRESS,
                               lowByte,
                               highByte};


    Dynamixel_Send_Write_Command(WRITE, huart, ID, GOAL_POSITION_SIZE, params);


    }
}



void Dynamixel_Set_MAX_Angle (const int angle, uint8_t ID, UART_HandleTypeDef *huart)
{

    if (angle > 0 || angle < 0xfff){



    const uint8_t highByte = (uint8_t)((angle >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angle & 0xff);



    const uint8_t params[3] = {MAX_ANGLE_ADDRESS,
                               lowByte,
                               highByte};


    Dynamixel_Send_Write_Command(WRITE, huart, ID, GOAL_POSITION_SIZE, params);


    }
}



void Dynamixel_Send_Write_Command (const uint8_t commandByte,
					   	   	   UART_HandleTypeDef *huart,
							   const uint8_t ID,
							   const uint8_t N,
							   const uint8_t *params){

	HAL_HalfDuplex_EnableTransmitter(huart);

	uint8_t checksum = 0;


    send_byte(huart, H1);
    send_byte(huart, H2);
    send_byte(huart, ID);
    send_byte(huart, N + 3);
    send_byte(huart, commandByte);


    for (uint8_t i = 0; i < N+1; i++)
    {
    	send_byte(huart, params[i]);
        checksum += params[i];
    }


    checksum += ID + N + 3 + commandByte;


    send_byte(huart, ~checksum);


    HAL_HalfDuplex_EnableReceiver(huart);

}



void Dynamixel_Set_Group_LED_ON(UART_HandleTypeDef *huart,
										 uint8_t *id_array,
										 uint8_t servo_number){



	uint8_t params[((LED_SIZE + 1) * servo_number) + 2];


	params[0] = LED;
	params[1] = LED_SIZE;


	for ( uint8_t i = 0; i < servo_number; i++ )
	{

		params[i * (LED_SIZE + 1) + 2] = *(id_array + i);
		params[i * (LED_SIZE + 1) + 3] = 1;


	}

	Dynamixel_Send_Sync_Write_Command(huart, LED_SIZE, servo_number, params);


}



void Dynamixel_Set_Group_LED_OFF(UART_HandleTypeDef *huart,
										 uint8_t *id_array,
										 uint8_t servo_number){


	    uint8_t params[((LED_SIZE + 1) * servo_number) + 2];


	    params[0] = LED;
	    params[1] = LED_SIZE;


		for ( uint8_t i = 0; i < servo_number; i++ )
		{

			params[i * (LED_SIZE + 1) + 2] = *(id_array+i);
			params[i * (LED_SIZE + 1) + 3] = 0;


		}

		Dynamixel_Send_Sync_Write_Command(huart, LED_SIZE, servo_number, params);


}



void Dynamixel_Set_Group_Torque_Enable(UART_HandleTypeDef *huart,
										uint8_t *id_array,
										uint8_t servo_number){



	uint8_t params[((TORQUE_ENABLE_SIZE + 1) * servo_number) + 2];


	params[0] = TORQUE_ENABLE;
	params[1] = TORQUE_ENABLE_SIZE;


	for ( uint8_t i = 0; i < servo_number; i++ )
	{

		params[i * (TORQUE_ENABLE_SIZE + 1) + 2] = *(id_array+i);
		params[i * (TORQUE_ENABLE_SIZE + 1) + 3] = 1;


	}

	Dynamixel_Send_Sync_Write_Command(huart, TORQUE_ENABLE_SIZE, servo_number, params);


}




void Dynamixel_Set_Group_Torque_Disable(UART_HandleTypeDef *huart,
										uint8_t *id_array,
										uint8_t servo_number){



	uint8_t params[((TORQUE_ENABLE_SIZE + 1) * servo_number) + 2];


	params[0] = TORQUE_ENABLE;
	params[1] = TORQUE_ENABLE_SIZE;


	for ( uint8_t i = 0; i < servo_number; i++ )
	{

		params[i * (TORQUE_ENABLE_SIZE + 1) + 2] = *(id_array+i);
		params[i * (TORQUE_ENABLE_SIZE + 1) + 3] = 0;


	}

	Dynamixel_Send_Sync_Write_Command(huart, TORQUE_ENABLE_SIZE, servo_number, params);


}



void Dynamixel_Set_Group_Servo_Angle(UART_HandleTypeDef *huart,
									uint16_t *data,
									uint8_t *id_array,
									uint8_t servo_number){


	uint8_t params[((GOAL_POSITION_SIZE + 1) * servo_number) + 2];


	params[0] = GOAL_POSITION;
	params[1] = GOAL_POSITION_SIZE;


	for ( uint8_t i = 0; i < servo_number; i++ )
	{

		params[i * (GOAL_POSITION_SIZE + 1) + 2] = *(id_array + i);
		params[i * (GOAL_POSITION_SIZE + 1) + 3] = (uint8_t)(*(data + i) & 0xff);
		params[i * (GOAL_POSITION_SIZE + 1) + 4] = (uint8_t)((*(data + i) >> 8) & 0xff);

	}


	Dynamixel_Send_Sync_Write_Command(huart, GOAL_POSITION_SIZE, servo_number, params);



}



void Dynamixel_Set_Group_MAX_Servo_Angle(UART_HandleTypeDef *huart,
									uint16_t *data,
									uint8_t *id_array,
									uint8_t servo_number){


	uint8_t params[((MAX_POSITION_SIZE + 1) * servo_number) + 2];


	params[0] = MAX_ANGLE_ADDRESS;
	params[1] = MAX_POSITION_SIZE;


	for ( uint8_t i = 0; i < servo_number; i++ )
	{

		params[i * (MAX_POSITION_SIZE + 1) + 2] = *(id_array + i);
		params[i * (MAX_POSITION_SIZE + 1) + 3] = (uint8_t)(*(data + i) & 0xff);
		params[i * (MAX_POSITION_SIZE + 1) + 4] = (uint8_t)((*(data + i) >> 8) & 0xff);

	}


	Dynamixel_Send_Sync_Write_Command(huart, MAX_POSITION_SIZE, servo_number, params);



}



void Dynamixel_Set_Group_MIN_Servo_Angle(UART_HandleTypeDef *huart,
									uint16_t *data,
									uint8_t *id_array,
									uint8_t servo_number){


	uint8_t params[((MIN_POSITION_SIZE + 1) * servo_number) + 2];


	params[0] = MIN_ANGLE_ADDRESS;
	params[1] = MIN_POSITION_SIZE;


	for ( uint8_t i = 0; i < servo_number; i++ )
	{

		params[i * (MIN_POSITION_SIZE + 1) + 2] = *(id_array + i);
		params[i * (MIN_POSITION_SIZE + 1) + 3] = (uint8_t)(*(data + i) & 0xff);
		params[i * (MIN_POSITION_SIZE + 1) + 4] = (uint8_t)((*(data + i) >> 8) & 0xff);

	}


	Dynamixel_Send_Sync_Write_Command(huart, MIN_POSITION_SIZE, servo_number, params);



}




void Dynamixel_Send_Sync_Write_Command (UART_HandleTypeDef *huart,
							   	   	   const uint8_t L,
									   const uint8_t N,
									   const uint8_t *params){


	HAL_HalfDuplex_EnableTransmitter(huart);

	uint8_t checksum = 0;

	uint8_t LENGTH   = ( L + 1 ) * N + 4;



	send_byte(huart, H1);
	send_byte(huart, H2);
	send_byte(huart, BROADCAST_ID);
	send_byte(huart, LENGTH);
	send_byte(huart, SYNC_WRITE);


	for (uint8_t i = 0; i < ( L + 1 ) * N + 2; i++)
	{
		send_byte(huart, *(params + i));
		checksum += *(params + i);
	}


	checksum += BROADCAST_ID + LENGTH + SYNC_WRITE;


	send_byte(huart, ~checksum);



    HAL_HalfDuplex_EnableReceiver(huart);

}



void send_byte(UART_HandleTypeDef *huart, uint8_t byte){

	HAL_UART_Transmit(huart, &byte, 1, 1);

}
