#ifndef __MAINBOARDTRAYAK__H
#define __MAINBOARDTRAYAK__H
#include "gpio.h"
#include "tim.h"
#include "AD7190.h"

typedef enum
{
	 PID,
   HARD_LIMIT,
}Controller_Method;
typedef struct
{
	Device_State state;
	uint32_t triggRetartVal;
	GPIO_TypeDef* port;
	uint16_t      pin;
} Triac_State;

typedef struct
{
	float P;
	float I;
	float D;
}PID_parameter;
typedef struct
{
	float setpoint;
	float masurement;
	float setpointMUX;
  float setpointMIN;	
}Temprature_DEF;

typedef struct
{
	relay_def   relay;
	Triac_State triac;
	Temprature_DEF temprature;
	TIM_HandleTypeDef* timer;
}TempratureController;
typedef struct
{
	TempratureController oil;
	TempratureController water;
	TempratureController steam;
}Seystem_Temprature_Controller;

extern Seystem_Temprature_Controller systemTempratureController;
void  TimerHeaterInit (void);
void  Temprature_handler (void);
void  TimerHeaterTriacInit (void);

void systemTempratureControllerInit(void);
void Temprature_handler(void);

uint16_t TempratureControllerCalc (float setPoint,float measurement);
#endif




