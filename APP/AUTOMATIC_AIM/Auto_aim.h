#ifndef __AutoAim
#define __AutoAim	
#include "main.h"
bool glide_compare(float value_buf[],float ADNum, float limit_num);
void Auto_aim(frame fram,int len);
void auto_aim_init(void);
void auto_bigbuff_init(void);

extern double view_ch2;
extern double view_ch3;
extern uint32_t last_time_tick_1ms,last_last_time_tick_1ms;
extern int  FirstFlag;
//extern int  if_newframe;
extern int  if_rotate_ok;
extern int 	if_if_newframe;

extern double  virual_ch2;
extern double  virual_ch3;

extern int timeout_count;

extern int	firstIN ;
extern int	shoot_seq;
extern int  big_moveOver;
extern int  big_moveBegin;
extern int	direction_wait;

extern float auto_aim_yaw_temp, auto_aim_pit_temp;
extern frame fram;
extern int why_not_shoot;
extern int if_newframe;
extern int timeout_count;
extern int bigbuff_change;

extern int reachPitch_Max;
extern int reachPitch_Min;

extern int auto_aim_count_;
extern int upack_count;

extern float yaw_bias;
extern float pitch_bias;


#endif
