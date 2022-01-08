#ifndef _CMCONTROL_H_
#define _CMCONTROL_H_

#include "main.h"

void CMControlInit(void);
void CMPID_Init(void);
void CMControlOut(int16_t speedA , int16_t speedB ,int16_t speedC ,int16_t speedD );
float CMSpeedLegalize(float MotorCurrent , float limit);
void move(int16_t speedX, int16_t speedY, int16_t rad);
void key_move(int16_t speedX, int16_t speedY, int16_t rad);
void CMControlLoop(void);
void CMStop(void);
int16_t followValCal(float Setposition);
void keyboardmove(uint16_t keyboardvalue,uint16_t xlimit,uint16_t ylimit);
//void SwingFollowValCal(void);
void CM_Switch_Moni(void);
void CM_Normal_PID(void);
void CM_Climb_PID(void);
float caculate_balance(float Setposition);
void move_balance(int16_t speedY, int16_t rad,int16_t balance);

extern float speed_limite;
extern u8 quick_spin_flag;
extern u8 climb_mode_flag;
extern u8 super_cap_flag;
extern float output_current_sum;
extern float lec_numA;
extern float lec_numB;
extern float lec_numC;
extern float lec_numD;
extern int16_t speedA_final,speedB_final,speedC_final,speedD_final;

#endif
