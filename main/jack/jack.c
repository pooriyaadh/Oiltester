#include "jack.h"
#include "serial.h"
  Jack_Def  jack;
void Jack_Init(void)
{
	// USUP PINS
	jack.US_UP.port = USup_GPIO_Port;
	jack.US_UP.pin = USup_Pin;
	//USDOWN PINS
	jack.US_DOWN.port = USdown_GPIO_Port;
	jack.US_DOWN.pin = USdown_Pin;
	//RELAY UP
	jack.Motor.relay_up.port =  jack_relay1_GPIO_Port;
	jack.Motor.relay_up.pin  =   jack_relay1_Pin;
	//RELAY DOWN
	jack.Motor.relay_down.port = jack_relay2_GPIO_Port;
	jack.Motor.relay_down.pin =  jack_relay2_Pin;
	
	jack.Motor.state = Motor_STOP; // For init function first  we should STOP the MOTOR
  jack.US_DOWN.state = US_close; // For init function first  we should CLOSE the US_DOWN
  jack.US_UP.state   = US_close; // For init function first  we should CLOSE the US_OPEN
	
	jack.Motor.timer = &htim3;
}
void jack_handler(void)
{
	switch(jack.Motor.state)
	 {
		case Motor_STOP:
			
			HAL_GPIO_WritePin(jack.Motor.relay_up.port , jack.Motor.relay_up.pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(jack.Motor.relay_down.port , jack.Motor.relay_down.pin , GPIO_PIN_RESET);		
				break;
		 case Motor_up :
			      if(HAL_GPIO_ReadPin(jack.US_UP.port , jack.US_UP.pin ) == 0)
	             {
		             HAL_GPIO_WritePin(jack.Motor.relay_up.port , jack.Motor.relay_up.pin , GPIO_PIN_SET);
	             }
					  else 
					  {
						HAL_GPIO_WritePin(jack.Motor.relay_down.port , 	jack.Motor.relay_down.pin , GPIO_PIN_RESET);		 
				    } 
			 break;
		 case Motor_down:
			     if(HAL_GPIO_ReadPin(USdown_GPIO_Port , USdown_Pin) == 0)
	            {
		         HAL_GPIO_WritePin(jack.Motor.relay_down.port , jack.Motor.relay_down.pin , GPIO_PIN_SET);
	           }
							else 
						{
						HAL_GPIO_WritePin(jack.Motor.relay_down.port , jack.Motor.relay_down.pin , GPIO_PIN_RESET);	
						}
			HAL_GPIO_WritePin(jack.Motor.relay_up.port , jack.Motor.relay_up.pin  , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(jack.Motor.relay_down.port , jack.Motor.relay_down.pin , GPIO_PIN_SET); 
			 break;
	 }	
	
	jack.Motor.timer->Instance->CCR1 = jack.Motor.timer->Init.Period * jack.Motor.speed /100;
  
   hmidata.tx.frame[jack_cmnd].packet.sof[0]  =  0XC1;
   hmidata.tx.frame[jack_cmnd].packet.sof[1]  =  0XB7;
   hmidata.tx.frame[jack_cmnd].packet.address =  hmi;
	 hmidata.tx.frame[jack_cmnd].packet.data.byte[3] = jack.Motor.state;
   hmidata.tx.frame[jack_cmnd].packet.command =  jack_cmnd;	 
}



