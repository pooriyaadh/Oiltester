#ifndef  __SENOLOID__H
#define  __SENOLOID__H
#include "gpio.h"
#include "serial.h"
#include "AD7190.h"

#define maxDP           15
#define telorance       1

#define minOnTime       2000   //vs milisecound
#define maxOffTime      10000  //vs milisecound



typedef enum
{
	normal_open,
	normal_close
}soneloidType_def;
typedef struct
{
relay_def     		relay; 
uint32_t   				tickstart;
Device_State 			state;
bool              periodIsStarted;
soneloidType_def  type; 
float  						Pressure_Set_Point;
}Solenoid;	

extern  Solenoid  solenoid_Struct;
void Solenoid_init(void);
void SonolOid_Handler(void);
#endif








