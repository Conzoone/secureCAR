/* Fichier de test pour l'API API_CAN.c
-----------------------------------------------------------------------------*/



#include "API_CAN.h"
#include <stdio.h>
#include "stm32f10x.h"                            /* STM32F10x Definitions    */
#include "LCD.h"                                  /* LCD function prototypes  */




int main(void){
	
	data_commands command_test_received;	

	can_init(20);	
		
	#ifdef __USE_LCD
  lcd_init  ();                                   /* initialise LCD           */
  lcd_clear ();
	#endif	
	
	while(1){
		command_test_received = CAN_receive_STM();
		display_commands(command_test_received.bytes_motor_command, command_test_received.steering_wheel_command.bytes_steering_wheel_command);
		Delay(10);
		lcd_clear();
	}
	
}
