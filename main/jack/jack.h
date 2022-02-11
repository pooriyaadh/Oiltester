#ifndef __JACK__H
#define __JACK__H
#include "gpio.h"
#include "tim.h"
void Jack_Init (void);
void jack_handler (void);
typedef enum
{
	 US_open,
	 US_close,
}US_State;

typedef struct
{
	US_State state;
	GPIO_TypeDef* port;
	uint16_t      pin;	
} US_Def;
typedef enum
{
	Motor_up = 0x02,
	Motor_down = 0x1,
	Motor_STOP = 0x03,
}Motor_state;
typedef struct
{
	float Trigg;
	float Duration;	
}Timer_Def;
typedef struct
{
	Timer_Def Time;
	Motor_state state;
	TIM_HandleTypeDef* timer;
  relay_def relay_up;
  relay_def	relay_down;
 float  speed;
}Motor_def;
typedef struct
{
	Motor_def Motor;
	US_Def  US_DOWN;
	US_Def  US_UP; 
}Jack_Def;

extern Jack_Def  jack; 
#endif







