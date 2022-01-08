#ifndef _SHOOTCONTROL_H_
#define _SHOOTCONTROL_H_

#include "main.h"


typedef struct {
    uint8_t             changed:1;
    uint8_t             stopping:4;
    volatile float      speed[3];
    uint16_t            target;
    volatile int16_t    output[3];
    PID_Type            pid;
    volatile uint32_t   counters[3];
    uint32_t            counter;
//    uint32_t        counter;            // for test
} PWMFriction_Type;



void Friction_Looper(uint32_t target) ;
void Friction_Init(void);


void ShootControlLoop(void);
void FrictionControl(void);
void FrictionJudge(void);
void ShootStop(void);
void GunControl(void);								//拨弹电机操作
void GunShootAction(void);
void GunShootAction_small_power_mode(void);
void ShootControlMove(float SetPosition);      //电机驱动
void ShootControlMove_small_power_mode(float SetPosition);
float ShootSpeedControl(float SetPosition,float NowPosition,PID_Struct* pid);
void ShootControlInit(void);
void shoot_cal(float shoot_speed_set );

extern float PIDOut_Speed_Shoot,PIDOut_Whole_Shoot;					//Pit,Yaw PID总输出
extern float shoot_speed_set ;
extern int robotLevel;
extern uint32_t shootTime_tick_1ms;
extern int gun_count ;
#endif
