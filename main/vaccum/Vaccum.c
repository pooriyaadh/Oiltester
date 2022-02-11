#include "Vaccum.h"
vaccum_def   vaccum_Struct;

void Vaccum_Relay_Init(void)
{
	vaccum_Struct.Relay.port = vacumm_relay_GPIO_Port;
	vaccum_Struct.Relay.pin  =  vacumm_relay_Pin;
	vaccum_Struct.state = device_off;
}
void Vaccum_Handler(void)
{
	if(vaccum_Struct.state == device_on) // If get the turn on Command 
	{
	HAL_GPIO_WritePin(vaccum_Struct.Relay.port ,vaccum_Struct.Relay.pin , GPIO_PIN_SET); 
	}
	else if(vaccum_Struct.state == device_off)
	{
	 HAL_GPIO_WritePin(vaccum_Struct.Relay.port ,vaccum_Struct.Relay.pin , GPIO_PIN_RESET);
	}
	
	//****************************************************
	// To Transmit All the data's for Component
  hmidata.tx.frame[vaccum_cmnd].packet.sof[0]  =  0XC1;
  hmidata.tx.frame[vaccum_cmnd].packet.sof[1]  =  0XB7;
  hmidata.tx.frame[vaccum_cmnd].packet.address =  hmi;
	hmidata.tx.frame[vaccum_cmnd].packet.data.byte[3] = vaccum_Struct.state; 
  hmidata.tx.frame[vaccum_cmnd].packet.command =  vaccum_cmnd;
  //****************************************************
}
 



