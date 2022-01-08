#ifndef __MONITORCONTROL_H
#define __MONITORCONTROL_H

#include "main.h"

extern float max_output_current;
extern float dynamic_zero_float_offset;
extern u8 clearing_flag;
extern u8 raging_mode;
extern float yaw_position_0;
extern int hand_cleaning_sent;
extern double hand_zero_clear;
void can2_monitor(void);
void monitorControlLoop(void);		
void Power_Moni(void);
void clear_zero_float(void);
void Reset_monitor(void);
void Rescue_Moni(void);
void press_send_rectangle(void);
#endif
