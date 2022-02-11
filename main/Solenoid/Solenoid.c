#include "Solenoid.h"
Solenoid solenoid_Struct;

void Solenoid_init(void)
{
	solenoid_Struct.relay.port = SENOLIODVALUE_GPIO_Port;
	solenoid_Struct.relay.pin = SENOLIODVALUE_Pin;
  solenoid_Struct.relay.state = device_off;

}
void SonolOid_Handler(void)//edit
{
  static uint32_t turnOnPeriod  = minOnTime;
	static uint32_t turnOffPeriod;
	
	if(solenoid_Struct.relay.state == device_off)
	{
		if(solenoid_Struct.type == normal_close)//edit
		{
			HAL_GPIO_WritePin(solenoid_Struct.relay.port ,solenoid_Struct.relay.pin , GPIO_PIN_SET);
		}
		else if(solenoid_Struct.type == normal_open)
		{
			HAL_GPIO_WritePin(solenoid_Struct.relay.port ,solenoid_Struct.relay.pin , GPIO_PIN_RESET);
		}
	}
	else if(solenoid_Struct.relay.state == device_on)
	{
		if(solenoid_Struct.type == normal_close)//edit
		{
			HAL_GPIO_WritePin(solenoid_Struct.relay.port ,solenoid_Struct.relay.pin , GPIO_PIN_RESET);
		}
		else if(solenoid_Struct.type == normal_open)
		{
			HAL_GPIO_WritePin(solenoid_Struct.relay.port ,solenoid_Struct.relay.pin , GPIO_PIN_SET);
		}
	}
	
 
   if(reader.sensor.pressure.value < solenoid_Struct.Pressure_Set_Point+telorance)// ghasem edit
	 {
		 solenoid_Struct.state = device_off; 
		 solenoid_Struct.relay.state = device_off;
	 }
	 else if(reader.sensor.pressure.value > solenoid_Struct.Pressure_Set_Point)
	 {
		 solenoid_Struct.state = device_on;
     
  	if(reader.sensor.pressure.value - solenoid_Struct.Pressure_Set_Point >  maxDP)
		{
			solenoid_Struct.relay.state = device_on;
			solenoid_Struct.periodIsStarted = false;
		}
		else
		{
			if(!solenoid_Struct.periodIsStarted)
			{
				solenoid_Struct.periodIsStarted = true;
				solenoid_Struct.relay.state = device_on;
				solenoid_Struct.tickstart = HAL_GetTick();
				turnOffPeriod = maxOffTime - (reader.sensor.pressure.value - 
												solenoid_Struct.Pressure_Set_Point) / maxDP * maxOffTime + minOnTime;  
			}
			if( HAL_GetTick() - solenoid_Struct.tickstart > turnOnPeriod)
			{
				solenoid_Struct.relay.state = device_off;
			}
		  if((HAL_GetTick() - solenoid_Struct.tickstart > turnOffPeriod)||
				 (HAL_GetTick() - solenoid_Struct.tickstart > maxOffTime))
			{
					solenoid_Struct.periodIsStarted = false;
			}
		}//ghasem edit
	}
	
  hmidata.tx.frame[solenoid_cmnd].packet.sof[0]  =  0XC1;
  hmidata.tx.frame[solenoid_cmnd].packet.sof[1]  =  0XB7;
  hmidata.tx.frame[solenoid_cmnd].packet.address =  hmi;
  hmidata.tx.frame[solenoid_cmnd].packet.data.byte[3] = solenoid_Struct.state;
  hmidata.tx.frame[solenoid_cmnd].packet.command =  solenoid_cmnd;
 
}













