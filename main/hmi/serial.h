#ifndef  __UART__H
#define  __UART__H
#include "usart.h"
#include "string.h"
#include "pump.h"
#include "Vaccum.h"
#include "serial.h"
#include "Solenoid.h"
#include "jack.h"
#include "tempratureController.h"
#include "stdbool.h"



typedef union
{
	uint8_t  byte[4];
	uint16_t intVal[2];
	uint32_t longval;
	float    floatval;
	float    Set_point;
}data_def;

typedef enum
{
		pump_cmnd = 0x01 ,
	  vaccum_cmnd = 0x02 ,
	  jack_cmnd = 0x03,
	  oil_cmnd = 0x04,
	  water_cmnd = 0x05,  
	  solenoid_cmnd = 0x06,              	
	  pressure_cmnd = 0x08,
	  turn_on_cmnd = 0x09,
	  steam_cmnd = 0x0a,
	  Set_Point_oil = 0x0b,
	  Set_Point_Water = 0x0c,
	  Set_Point_Pressure = 0x0d,
	  cmnd_max ,
}Devices_def;
#pragma pack(1)
typedef struct{
	uint8_t   sof[2];
	uint8_t   address;
	Devices_def   command;
	uint8_t   Sub_Command;
	data_def  data;
	uint16_t  checksum;
}packet_def;

typedef union{
	uint8_t byte[11];
  packet_def packet;
}frame_def;
#pragma pack()

typedef struct
{
	GPIO_TypeDef* port;
uint16_t      pin;
}RS1;
typedef struct
{
	GPIO_TypeDef* port;
uint16_t      pin;
}RS2;

typedef struct
{
 bool   En;
 frame_def frame[cmnd_max]; 
 Devices_def selectedDevice;
}transmit_def;
typedef struct
{
  bool   En;
  int       counter;
	Devices_def devices;
	frame_def frame;
}receive_def;





typedef struct{
	receive_def  rx;
	transmit_def tx;
	UART_HandleTypeDef* uart;
	RS1 Rs1;
	RS2 Rs2;
}hmi_data_Def;

typedef enum 
{ 
 	 hmi  = 0x00, 
   mainboard = 0x01,	
}Address;
typedef enum
{
	 sof1 = 0XC1,
	 sof2 = 0XB7
}SOF;
extern hmi_data_Def hmidata;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); //  When Recieving data from uart


void serial_init(void);

void serial_handler(void);
uint16_t Checksum(uint8_t *array, int size);

#endif

