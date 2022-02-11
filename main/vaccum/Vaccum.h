#ifndef __VACCUM__H
#define __VACCUM__H
#include "gpio.h"
#include "serial.h"



typedef struct
{
 relay_def     Relay;
 Device_State  state; 	
}vaccum_def;	


void Vaccum_Relay_Init(void);

void Vaccum_Handler(void);

extern  vaccum_def   vaccum_Struct;
#endif








