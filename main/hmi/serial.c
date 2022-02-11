#include "serial.h"
uint8_t Rxdata[1];

hmi_data_Def hmidata;
uint16_t Checksum(uint8_t *array, int size)
{
	uint16_t out = 0; 
	for(int i = 0; i < size ; i++)
	{
		out += array[i];
	}
	out = 0x00;
	return (out);
}

void serial_init(void)
{
	 hmidata.uart = &huart1;
	 hmidata.Rs1.port = RS1_EN_GPIO_Port;
	 hmidata.Rs1.pin =  RS1_EN_Pin;
	 hmidata.Rs2.port = RS2_EN_GPIO_Port;
	 hmidata.Rs2.pin =  RS2_EN_Pin;
	//***************************************************************
	 HAL_GPIO_WritePin(hmidata.Rs1.port , hmidata.Rs1.pin , GPIO_PIN_RESET);
   HAL_GPIO_WritePin(hmidata.Rs2.port , hmidata.Rs2.pin , GPIO_PIN_RESET);
	//***************************************************************
	HAL_UART_Receive_IT(&huart1 , (uint8_t *)Rxdata , sizeof(Rxdata)/sizeof(Rxdata[0]));
	
	memset(hmidata.rx.frame.byte,0,sizeof(hmidata.rx.frame.byte)/sizeof(hmidata.rx.frame.byte[0]));
	hmidata.tx.En = false;
	hmidata.rx.counter = 0;
}
void serial_handler(void)
{
	// Transmit Part
	if(hmidata.tx.En)
 {
	 HAL_GPIO_WritePin (hmidata.Rs1.port , hmidata.Rs1.pin , GPIO_PIN_SET);
   HAL_GPIO_WritePin (hmidata.Rs2.port , hmidata.Rs2.pin , GPIO_PIN_SET);
	 Devices_def index = hmidata.tx.selectedDevice;
	 hmidata.tx.frame[index].packet.checksum =  Checksum(hmidata.tx.frame[index].byte,sizeof(hmidata.tx.frame[index].byte));  
   HAL_UART_Transmit (hmidata.uart , hmidata.tx.frame[hmidata.tx.selectedDevice].byte ,sizeof(hmidata.tx.frame[0].byte) , 5000);
   HAL_GPIO_WritePin (hmidata.Rs1.port , hmidata.Rs1.pin , GPIO_PIN_RESET);
   HAL_GPIO_WritePin (hmidata.Rs2.port , hmidata.Rs2.pin , GPIO_PIN_RESET);
	 hmidata.tx.En = false;
 } 
	
 // receive part 
	if(hmidata.rx.counter > 2)
	{
		 if((hmidata.rx.frame.packet.sof[0] == sof1) && (hmidata.rx.frame.packet.sof[1] == sof2)
 			&& (hmidata.rx.frame.packet.address == mainboard))
		  {
			if(hmidata.rx.counter >= sizeof(hmidata.rx.frame.byte))
			{
				 uint16_t checksum = Checksum(hmidata.rx.frame.byte,hmidata.rx.counter-2);
				 Device_State tmp; 
				if(checksum == hmidata.rx.frame.packet.checksum)  
				{
					switch(hmidata.rx.frame.packet.command)
					{
						case pump_cmnd:
						{
							tmp = hmidata.rx.frame.packet.Sub_Command;
							if(tmp == device_reply) 
							{
								hmidata.tx.selectedDevice = pump_cmnd;	
								hmidata.tx.En = true;
							}
							else if((tmp == device_off)||(tmp == device_on))
							{
								pump_struct.state = tmp;
							}
						}
						break;
						
						case vaccum_cmnd:
						{
							tmp = hmidata.rx.frame.packet.Sub_Command;
	          	if(tmp == device_reply)
							{
							 	hmidata.tx.selectedDevice = vaccum_cmnd;	
								hmidata.tx.En = true;	
							}
							else if((tmp == device_off)||(tmp == device_on))
							{
								vaccum_Struct.state = tmp;
							}
						}
						break;
							
						case jack_cmnd:
						{
						  tmp = hmidata.rx.frame.packet.Sub_Command;
						  if(tmp == device_reply)
							{
							 	hmidata.tx.selectedDevice = jack_cmnd;	
								hmidata.tx.En = true;	
							}
						   else if((tmp == Motor_up) || tmp == Motor_down || tmp == Motor_STOP)
							 {
							 	jack.Motor.state = (Motor_state)tmp;	 
							 }	
						}							 
						break;
						
						case oil_cmnd:
						{
							tmp = hmidata.rx.frame.packet.Sub_Command;
						 if(tmp == device_reply)
						 {
							 	hmidata.tx.selectedDevice = oil_cmnd;	
								hmidata.tx.En = true;
						 }
					  }
						 break;
						 
						case water_cmnd:
						{
							 tmp = hmidata.rx.frame.packet.Sub_Command;
						 if(tmp == device_reply)
						 {
							 	hmidata.tx.selectedDevice = water_cmnd;	
								hmidata.tx.En = true;
						 }
					  }
						 break;
						 
						 case steam_cmnd:
						 {
							 tmp = hmidata.rx.frame.packet.Sub_Command;
						  if(tmp == device_reply)
						    {
							 	hmidata.tx.selectedDevice = steam_cmnd;	
								hmidata.tx.En = true;
						    }
						}
						   break;
						 
					  case  pressure_cmnd:
						{
		           tmp = hmidata.rx.frame.packet.Sub_Command;
						     if(tmp == device_reply)
						     {
							 	 hmidata.tx.selectedDevice = pressure_cmnd;	
								 hmidata.tx.En = true;
						     }
						}								 
             break;
						case  Set_Point_oil:
						{
						tmp = hmidata.rx.frame.packet.Sub_Command;
						if(tmp == device_reply)
						 {
							hmidata.tx.selectedDevice = Set_Point_oil;	
							hmidata.tx.En = true;
						 }
						 else if(tmp == device_set_point)
						 {
							systemTempratureController.oil.temprature.setpoint = 
							    hmidata.rx.frame.packet.data.Set_point;
						 }
					  }						 
              break;
						 case  Set_Point_Water:
						 {
								tmp = hmidata.rx.frame.packet.Sub_Command;
								if(tmp == device_reply)
								{
								 hmidata.tx.selectedDevice = Set_Point_Water;	
								 hmidata.tx.En = true;	
								}
							 else if(tmp == device_set_point)
							 {
								systemTempratureController.water.temprature.setpoint = 
								 hmidata.rx.frame.packet.data.Set_point;	 
							 }	
							}										 
							 break;
          	 case Set_Point_Pressure:
							 	tmp = hmidata.rx.frame.packet.Sub_Command;
								if(tmp == device_reply)
								{
								 hmidata.tx.selectedDevice = Set_Point_Pressure;	
								 hmidata.tx.En = true;	
								}
								else if(tmp == device_set_point)
							 {
								solenoid_Struct.Pressure_Set_Point = hmidata.rx.frame.packet.data.Set_point;	 
							 } 
						 break;
							 
						 case turn_on_cmnd:
						 {
								system.status = hmidata.rx.frame.packet.Sub_Command;
						 }
                break;										 
                      										 
					}
					memset(hmidata.rx.frame.byte,0,sizeof(hmidata.rx.frame.byte)/sizeof(hmidata.rx.frame.byte[0]));
					hmidata.rx.counter = 0;
				}
				else
				{
					memset(hmidata.rx.frame.byte,0,sizeof(hmidata.rx.frame.byte)/sizeof(hmidata.rx.frame.byte[0]));
					hmidata.rx.counter = 0;
				}
				
			}
		}
		else
		{
			memset(hmidata.rx.frame.byte,0,sizeof(hmidata.rx.frame.byte)/sizeof(hmidata.rx.frame.byte[0]));
			hmidata.rx.counter = 0;
		}
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //  When Recieving data from uart
{
	if(huart->Instance ==  hmidata.uart->Instance)
	{
	 if(hmidata.rx.counter < sizeof(hmidata.rx.frame.byte)/sizeof(hmidata.rx.frame.byte[0]))
	 {
		hmidata.rx.frame.byte[hmidata.rx.counter++] = Rxdata[0];  
	 }
	}
	HAL_UART_Receive_IT(&huart1 , (uint8_t *)Rxdata , sizeof(Rxdata)/sizeof(Rxdata[0]));
}



	



