#ifndef API_CAN_H
#define API_CAN_H

#include "CAN.h"
#include "stm32f10x.h"       /* STM32F10x Definitions    */


/*----------------------------------------------------------------------------------------------------------------------------------------------
//Definition of the datas exchanged in the CAN communication
//We use union types to convert short in char[8] to permit the integration of the onformation in
//a frame
------------------------------------------------------------------------------------------------------------------------------------------------*/

//Definiton of the data from the sensors
typedef struct data_sensors 	
	{		
		union {
			short num_left_odometer;
			unsigned char  bytes_left_odometer[2];
		}left_odometer;
		
		union {
			short num_right_odometer;
			unsigned char  bytes_right_odometer[2];
		}right_odometer;
		
		union {
			short num_potentiometer;
			unsigned char  bytes_potentiometer[2];
		}potentiometer;
		
	}data_sensors;

//Defintion of the command data
typedef struct data_commands 	
	{
		unsigned char  bytes_motor_command;
		
		union {
			short num_steering_wheel_command;
			unsigned char  bytes_steering_wheel_command[2];
		}steering_wheel_command;
		
	}data_commands;

	void can_init (int id);
	void periodic_CAN_send(char* can_trame , int id, int period);
	void non_periodic_CAN_send(CAN_msg* msg_can, char* can_trame , int id);
	void merge_data_STM(data_sensors data, char* can_trame);
	void merge_data_Raspberry(data_commands data, char* can_trame);
	data_commands CAN_receive_STM(void);
	data_sensors CAN_receive_Raspberry(void);
		
	void SysTick_Handler(void);
	void Delay (uint32_t dlyTicks);
	void display_commands (unsigned char com_motor, unsigned char* com_steer);
	void display_sensors (unsigned char* sens_odo_left, unsigned char* sens_odo_right, unsigned char* sens_potar );
	void display_trame (char* trame_can);
	void display_val (short val_display);

	


#endif
	