#include "tempratureController.h"
#include "serial.h"
Seystem_Temprature_Controller systemTempratureController;

void systemTempratureControllerInit(void)
{
	/// initilizing systemTempratureController initializer for oil part
	systemTempratureController.oil.relay.state = device_off;
	systemTempratureController.oil.relay.port  = RELAY_HEATER_OIL_GPIO_Port;
	systemTempratureController.oil.relay.pin   = RELAY_HEATER_OIL_Pin;
	systemTempratureController.oil.triac.state = device_off;
	//systemTempratureController.oil.triac.triggRetartVal = 50000;
	systemTempratureController.oil.timer = &htim6;
	
	//configuaration for triac OIL
	systemTempratureController.oil.triac.port = OP_HEATER_OIL_GPIO_Port;
  systemTempratureController.oil.triac.pin  = OP_HEATER_OIL_Pin;	
	/// initilizing systemTempratureController initializer for water part
	
	systemTempratureController.water.relay.state = device_off;
	systemTempratureController.water.relay.port = RELAY_HEATER_WATER_GPIO_Port;
	systemTempratureController.water.relay.pin = RELAY_HEATER_WATER_Pin;
	systemTempratureController.water.triac.state = device_off;
 // systemTempratureController.water.triac.triggRetartVal = 50000;
	systemTempratureController.water.timer = &htim14;
	
	//configuaration for triac WATER 
	systemTempratureController.water.triac.port = OP_HEATER_WATER_GPIO_Port;
	systemTempratureController.water.triac.pin = OP_HEATER_WATER_Pin; 
}
void Temprature_handler(void)
{
	// Check the states of the device for water 
	if(systemTempratureController.water.relay.state == device_on)
	{
		HAL_GPIO_WritePin(systemTempratureController.water.relay.port , 
											systemTempratureController.water.relay.pin , GPIO_PIN_SET);
	}
	else if(systemTempratureController.water.relay.state == device_off)
	{
    HAL_GPIO_WritePin(systemTempratureController.water.relay.port , 
											systemTempratureController.water.relay.pin , GPIO_PIN_RESET);
	}
	
	// Check the states of the device for oil
	if(systemTempratureController.oil.relay.state == device_on)
	{
		HAL_GPIO_WritePin(systemTempratureController.oil.relay.port , 
											systemTempratureController.oil.relay.pin , GPIO_PIN_SET);
	}
	else if(systemTempratureController.oil.relay.state == device_off)
	{
		HAL_GPIO_WritePin(systemTempratureController.oil.relay.port , 
											systemTempratureController.oil.relay.pin , GPIO_PIN_RESET);
	}
	// turn on zero detection for triac operation 
	if(systemTempratureController.oil.triac.state == device_on)
	{
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	}
	else if(systemTempratureController.water.triac.state == device_on)
	{
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	}
	else
	{
	  HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
	}
	// turn on zero detection for triac operation 

	
	//************************ Controll the Heater relay with set_point ********************************
	//***************************************calcuate The temprature******************************************
  systemTempratureController.oil.temprature.masurement   =  (reader.sensor.PT100_oil.Resistance - 100) / 0.4;			
	systemTempratureController.water.temprature.masurement =  (reader.sensor.PT100_water.Resistance - 100) / 0.4; 
  reader.sensor.pressure.measurement =  pow(10 , reader.sensor.pressure.value - 5.5);
  systemTempratureController.steam.temprature.masurement = (reader.sensor.PT100_steam.Resistance - 100) / 0.4;
 //***************************************calcuate The temprature******************************************

	if(systemTempratureController.oil.temprature.masurement <= systemTempratureController.oil.temprature.setpoint)
	{
		systemTempratureController.oil.relay.state = device_on;
		systemTempratureController.oil.triac.state = device_on;
		systemTempratureController.oil.triac.triggRetartVal =  
		 TempratureControllerCalc(systemTempratureController.oil.temprature.setpoint ,
 		                          systemTempratureController.oil.temprature.masurement);		
	}
	else
	{
		systemTempratureController.oil.relay.state = device_off;
		systemTempratureController.oil.triac.state = device_off;
		systemTempratureController.oil.triac.triggRetartVal = 
     TempratureControllerCalc(systemTempratureController.oil.temprature.setpoint ,
 		                          systemTempratureController.oil.temprature.masurement);				
	}
	
  if(systemTempratureController.water.temprature.masurement  <= systemTempratureController.water.temprature.setpoint)
	{
  	systemTempratureController.water.relay.state = device_on;
		systemTempratureController.water.triac.state = device_on;
		systemTempratureController.water.triac.triggRetartVal = 
		 TempratureControllerCalc(systemTempratureController.water.temprature.setpoint ,
														  systemTempratureController.water.temprature.masurement);
	}
	else
	{
		systemTempratureController.water.relay.state = device_off;
		systemTempratureController.water.triac.state = device_off;
		systemTempratureController.water.triac.triggRetartVal = 
		 TempratureControllerCalc(systemTempratureController.water.temprature.setpoint ,
														  systemTempratureController.water.temprature.masurement);
	}

	

	
	 hmidata.tx.frame[oil_cmnd].packet.sof[0]  = 0XC1;
   hmidata.tx.frame[oil_cmnd].packet.sof[1]  = 0XB7;
   hmidata.tx.frame[oil_cmnd].packet.address =  hmi;
	 hmidata.tx.frame[oil_cmnd].packet.data.floatval = systemTempratureController.oil.temprature.masurement;
   hmidata.tx.frame[oil_cmnd].packet.command =  oil_cmnd;	// complete these packets
	
	
   hmidata.tx.frame[water_cmnd].packet.sof[0]  = 0XC1;
   hmidata.tx.frame[water_cmnd].packet.sof[1]  = 0XB7;
   hmidata.tx.frame[water_cmnd].packet.address =  hmi;
	 hmidata.tx.frame[water_cmnd].packet.data.floatval = systemTempratureController.water.temprature.masurement;
   hmidata.tx.frame[water_cmnd].packet.command =  water_cmnd;
	
	 hmidata.tx.frame[steam_cmnd].packet.sof[0] = 0XC1;
	 hmidata.tx.frame[steam_cmnd].packet.sof[1] = 0XB7;
	 hmidata.tx.frame[steam_cmnd].packet.address =  hmi;
	 hmidata.tx.frame[steam_cmnd].packet.data.floatval = systemTempratureController.steam.temprature.masurement;
	 hmidata.tx.frame[steam_cmnd].packet.command =  steam_cmnd;
	
	
	
	 //******************************* Set_Point_Oil Command************************************************\\
	
	 hmidata.tx.frame[Set_Point_oil].packet.sof[0] = 0XC1;
	 hmidata.tx.frame[Set_Point_oil].packet.sof[1] = 0XB7;
	 hmidata.tx.frame[Set_Point_oil].packet.address =  hmi;
	 hmidata.tx.frame[Set_Point_oil].packet.data.Set_point = systemTempratureController.oil.temprature.setpoint; 
	 hmidata.tx.frame[Set_Point_oil].packet.command =  Set_Point_oil;
   //******************************* Set_Point_Oil Command************************************************\\

   //******************************* Set_Point_Water Command************************************************\\
	 hmidata.tx.frame[Set_Point_Water].packet.sof[0] = 0XC1;
	 hmidata.tx.frame[Set_Point_Water].packet.sof[1] = 0XB7;
	 hmidata.tx.frame[Set_Point_Water].packet.address =  hmi;
	 hmidata.tx.frame[Set_Point_Water].packet.data.Set_point = systemTempratureController.water.temprature.setpoint; 
	 hmidata.tx.frame[Set_Point_Water].packet.command =  Set_Point_oil;
	 //******************************* Set_Point_Water Command************************************************
}
uint16_t TempratureControllerCalc (float setPoint,float measurement)
{
	uint16_t value;
	uint16_t maxTim = 10000;  // this parameter means milli Secounds
	uint16_t minTim = 10;   // maximum watt
	if(setPoint < measurement)
	{
		return maxTim;
	}
	else if(setPoint - measurement > 10)
	{
	   return minTim;
	}
	else
	{
	    value = maxTim-(setPoint-measurement)*maxTim/10+minTim;
	    return value;
	}

}
void TimerHeaterInit(void) //when the trigger is called Timer6 interuppt is start
{
	//check the timer for TIMER6 for oil
	if(systemTempratureController.oil.triac.state == device_on)
	{
   systemTempratureController.oil.timer->Instance->ARR  = systemTempratureController.oil.triac.triggRetartVal;  
	 HAL_TIM_Base_Start_IT(systemTempratureController.oil.timer);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(systemTempratureController.oil.timer);
	}
	
	// check the timer for TIMER14 for WATER
	if(systemTempratureController.water.triac.state == device_on)
	{
		systemTempratureController.water.timer->Instance->ARR = systemTempratureController.water.triac.triggRetartVal;
		HAL_TIM_Base_Start_IT(systemTempratureController.water.timer);
	}
	else 
	{
		HAL_TIM_Base_Stop_IT(systemTempratureController.water.timer);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//TIMER 6 FOR OIL
	if(htim->Instance == systemTempratureController.oil.timer->Instance)
	{
		if(systemTempratureController.oil.triac.state == device_on)
		{
			HAL_GPIO_WritePin(systemTempratureController.oil.triac.port , 
												systemTempratureController.oil.triac.pin , GPIO_PIN_SET);
		}
		else if(systemTempratureController.oil.triac.state == device_off)
		{
			HAL_GPIO_WritePin(systemTempratureController.oil.triac.port ,
												systemTempratureController.oil.triac.pin , GPIO_PIN_RESET);	
		}
  	HAL_TIM_Base_Stop_IT(systemTempratureController.oil.timer);
	}
	
	// TIMER14 FOR WATER
	if(htim->Instance == systemTempratureController.water.timer->Instance)
	{
		if(systemTempratureController.water.triac.state == device_on)
		{
			HAL_GPIO_WritePin(systemTempratureController.water.triac.port , 
												systemTempratureController.water.triac.pin , GPIO_PIN_SET);
		}
		else if(systemTempratureController.water.triac.state == device_off)
		{
		  HAL_GPIO_WritePin(systemTempratureController.water.triac.port ,
												systemTempratureController.water.triac.pin , GPIO_PIN_RESET);
		}
		HAL_TIM_Base_Stop_IT(systemTempratureController.water.timer);
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t  GPIO_Pin) // WHEN THE TRIGGER PIN IS SELECT THEN TIMER IS START
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		HAL_GPIO_WritePin(systemTempratureController.oil.triac.port , systemTempratureController.oil.triac.pin ,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(systemTempratureController.water.triac.port,systemTempratureController.water.triac.pin ,GPIO_PIN_RESET);
		TimerHeaterInit();	
	}
}
