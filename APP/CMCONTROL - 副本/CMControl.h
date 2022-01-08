#ifndef _CMCONTROL_H_
#define _CMCONTROL_H_

#include "main.h"

void CMControlInit(void);
void CMPID_Init(void);
void CMControlOut(int16_t speedA , int16_t speedB ,int16_t speedC ,int16_t speedD );
float CMSpeedLegalize(float MotorCurrent , float limit);
void move(int16_t speedX, int16_t speedY, int16_t rad);
void CMControlLoop(void);
void CMStop(void);
int16_t followValCal(float Setposition);
void keyboardmove(uint16_t keyboardvalue);
void SwingFollowValCal(void);

extern float speed_limite;
extern u8 quick_spin_flag;

#endif
