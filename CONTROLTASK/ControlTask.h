#ifndef __ControlTask_H
#define __ControlTask_H	

#include "main.h"


void WholeInitTask(void);
void Control_Task(void);
extern RemoteState_e remoteState_pre;
extern uint32_t time_tick_1ms;
extern uint16_t small_power_delay;
#endif
