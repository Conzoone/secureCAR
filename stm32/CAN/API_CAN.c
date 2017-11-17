/*----------------------------------------------------------------------------
 * Name:    API_CAN.c
 * Purpose: API for CAN functions
 * Note(s): 
 *----------------------------------------------------------------------------
*/

#include "API_CAN.h"
#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"                            /* STM32F10x Definitions    */
#include "LCD.h"                                  /* LCD function prototypes  */



volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
char text[17];


void can_init(int id){
	
	CAN_setup ();                                   /* setup CAN Controller     */
  CAN_wrFilter (id, STANDARD_FORMAT);             /* Enable reception of msgs */

  /* ! COMMENT LINE BELOW TO ENABLE DEVICE TO PARTICIPATE IN CAN NETWORK !    */
  // CAN_testmode(CAN_BTR_SILM | CAN_BTR_LBKM);      // Loopback, Silent Mode (self-test)

  CAN_start ();                                   /* start CAN Controller   */
  CAN_waitReady ();                               /* wait til tx mbx is empty */
	
	SysTick_Config(SystemCoreClock/1000);
	
	
}



void merge_data_STM(data_sensors data,char* can_trame){
	
	//La taille d'une trame CAN est de 8 octets
		
	can_trame[0] = data.left_odometer.bytes_left_odometer[0] ;
	can_trame[1] = data.left_odometer.bytes_left_odometer[1] ;
	can_trame[2] = data.right_odometer.bytes_right_odometer[0] ;
	can_trame[3] = data.right_odometer.bytes_right_odometer[1] ;
	can_trame[4] = data.potentiometer.bytes_potentiometer[0] ;
	can_trame[5] = data.potentiometer.bytes_potentiometer[1] ;
//	can_trame[6] = '\0';
//	can_trame[7] = '\0';
	
}


void merge_data_Raspberry(data_commands data, char* can_trame){
	
//	int i;
	can_trame[0] = data.bytes_motor_command;
	can_trame[1] = data.steering_wheel_command.bytes_steering_wheel_command[0];
	can_trame[2] = data.steering_wheel_command.bytes_steering_wheel_command[1];
	
//	for (i =3; i<8;i++){
//		can_trame[i]='\0';
//	}
	
}

data_commands CAN_receive_STM(void){
	
	data_commands data_received;
	data_received.steering_wheel_command.num_steering_wheel_command = -1 ;
	data_received.bytes_motor_command = '\0' ;	
	if (CAN_RxRdy){
		CAN_RxRdy = 0;
		data_received.bytes_motor_command = CAN_RxMsg.data[0] ;
		data_received.steering_wheel_command.bytes_steering_wheel_command[0] = CAN_RxMsg.data[1] ;
		data_received.steering_wheel_command.bytes_steering_wheel_command[1] = CAN_RxMsg.data[2] ;
		}
	return data_received;
}

data_sensors CAN_receive_Raspberry(void){
	
	data_sensors data_received;
	
	data_received.left_odometer.num_left_odometer = -1 ;
	data_received.right_odometer.num_right_odometer = -1 ;
	data_received.potentiometer.num_potentiometer = -1 ;
	
	if (CAN_RxRdy){
		
		CAN_RxRdy = 0;
		data_received.left_odometer.bytes_left_odometer[0] = CAN_RxMsg.data[0] ;
		data_received.left_odometer.bytes_left_odometer[1] = CAN_RxMsg.data[1] ;
		data_received.right_odometer.bytes_right_odometer[0] = CAN_RxMsg.data[2] ;
		data_received.right_odometer.bytes_right_odometer[1] = CAN_RxMsg.data[3] ;
		data_received.potentiometer.bytes_potentiometer[0] = CAN_RxMsg.data[4] ;
		data_received.potentiometer.bytes_potentiometer[1] = CAN_RxMsg.data[5] ;
	}
	return data_received;
}

void non_periodic_CAN_send(CAN_msg* msg_can ,char* can_trame , int id){
	
	int i = 0,i2 = 0, size_limit = 0 ;
	
	//On initalise les données de la trame
	for (i =0 ;i<8;i++){
			msg_can->data[i] = 0 ;
	}
	
	//Dans tous les cas, nos messages font moins d'un octet	
	msg_can->len = 1;
	
	//Si ID est à 10, alors on sait qu'il s'agit du message des capteurs
	if (id == 10 ){
		msg_can->id = 10;
		msg_can->format = STANDARD_FORMAT;
		msg_can->type = DATA_FRAME;
		size_limit = 6 ; //Pour rentrer le bon nombre de donnees dans la trame can
	}
	
	//Si ID est à 20, alors on sait qu'il s'agit du message de commande
	else if (id == 20){
		msg_can->id = 20;
		msg_can->len = 8;
		msg_can->format = STANDARD_FORMAT;
		msg_can->type = DATA_FRAME;
		size_limit = 3; //Pour rentrer le bon nombre de donnees dans la trame can
	}
	
	if (CAN_TxRdy) {                              /* tx msg on CAN Ctrl       */
      CAN_TxRdy = 0;
			for(i2=0;i2<size_limit;i2++){
				msg_can->data[i2] = can_trame[i2];                 /* data[0] = ADC value      */
			}
      
      CAN_wrMsg (msg_can);                     /* transmit message         */
    }
	
}
	
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}

void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}


void display_commands (unsigned char com_motor, unsigned char* com_steer) {

#ifdef __USE_LCD
	sprintf(text, "Mo:%c, Wh:%c%c", com_motor, com_steer[0],com_steer[1]);

  //set_cursor (0, 1);
  lcd_print  (text);                              /* print string to LCD      */
#endif
  Delay (10);                                     /* dlay for 10ms            */
}

void display_sensors (unsigned char* sens_odo_left, unsigned char* sens_odo_right, unsigned char* sens_potar ) {

#ifdef __USE_LCD
	sprintf(text, "L:%c%c, R:%c%c, A:%c%c", sens_odo_left[0], sens_odo_left[1], sens_odo_right[0], sens_odo_right[1],sens_potar[0], sens_potar[1]);
	set_cursor (0, 1);
  lcd_print  (text);                              /* print string to LCD      */
#endif
  Delay (10);                                     /* dlay for 10ms            */
}

void display_trame (char* trame_can) {

#ifdef __USE_LCD
	sprintf(text, "%c%c%c%c%c%c%c%c",trame_can[0],trame_can[1],trame_can[2],trame_can[3],trame_can[4],trame_can[5],trame_can[6],trame_can[7]);
	//set_cursor (0, 1);
  lcd_print  (text);                              /* print string to LCD      */
#endif
  Delay (10);                                     /* dlay for 10ms            */
}

void display_val (short val_display) {

#ifdef __USE_LCD
	sprintf(text,"Rx: %d", val_display);
	//set_cursor (0, 1);
  lcd_print  (text);                              /* print string to LCD      */
#endif
  Delay (10);                                     /* dlay for 10ms            */
}
	

