#ifndef __PUMP__H
#define __PUMP__H
#include "gpio.h"

typedef struct
{
 relay_def    relay;	
 Device_State state;
}pump_def;	

extern  pump_def pump_struct;

void Pump_Handler(void);

void Pump_Relay_Init(void);

#endif


