#include "pump.h"
#include "serial.h"
pump_def    pump_struct;
void Pump_Relay_Init(void)
{
  pump_struct.relay.port = PUMP_RELAY_GPIO_Port;
	pump_struct.relay.pin = PUMP_RELAY_Pin;
	pump_struct.state = device_off;
}
void Pump_Handler(void)
{
	if(pump_struct.state == device_on)
	{
	HAL_GPIO_WritePin(pump_struct.relay.port ,pump_struct.relay.pin , GPIO_PIN_SET);
	}
	else if(pump_struct.state == device_off)
	{
		HAL_GPIO_WritePin(pump_struct.relay.port ,pump_struct.relay.pin , GPIO_PIN_RESET);
	}	

	hmidata.tx.frame[pump_cmnd].packet.sof[0]  =  0XC1;
  hmidata.tx.frame[pump_cmnd].packet.sof[1]  =  0XB7;
  hmidata.tx.frame[pump_cmnd].packet.address =  hmi;
	hmidata.tx.frame[pump_cmnd].packet.data.byte[3] = pump_struct.state; 
  hmidata.tx.frame[pump_cmnd].packet.command =  pump_cmnd;

}









