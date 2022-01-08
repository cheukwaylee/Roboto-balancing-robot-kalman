#include "define.h"
#ifndef __FRAME
#define __FRAME	 

extern u8 sendbuffer[100];

//modified by sui qi
//modified by twb 2017
#pragma  pack(1) 
typedef struct _frame				
{
	u8 head;		//0xf1 0xf1
	u16 timestamp;
	float yaw;	
	float pitch;
	u8 extra[2]; //additional imformation	
	u8 crc8check;
	u8 end;
}frame;			
#pragma  pack() 

#pragma  pack(1) 
typedef struct _frame_power				
{
	u8 head[2];
	int16_t speed[4];
	int16_t T_current[4];
	float   limite[4];
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	u8 tail[2];
}power_frame;			
#pragma  pack() 

#pragma  pack(1) 
typedef struct _gyro_data				
{
	u8 head[2];
	int16_t adi_raw_acce[3];
	int16_t adi_raw_gyro[3];
	float mpu_raw_acce[3];
	float mpu_raw_gyro[3];
	float real_mpu_acce[3];
	float real_mpu_gyro[3];
	float real_adi_acce[3];
	float real_adi_gyro[3];
	float yaw;
	float pitch;
	float roll;
	float yaw_target[2];
	float pitch_target[2];
	u8 tail[2];
}gyro_data;			
#pragma  pack() 


/*
#pragma  pack(1) 
typedef struct _frame_send				
{
	u8 head[2];	
	u32 timestamp;

	s16 v_x;
	s16 v_y;
	s16 v_w;											  									
	s16 target_x;//undermined
	s16 target_y;//undermined
	s16 target_w;//undermined		
	float platform_delta_yaw;
	float platform_delta_pitch;
	u8 cmd0;
	u8 cmd1;
	u8 cmd2;
	u8 cmd3;	
	u8 crc8check;							
}frame_send;											
#pragma  pack() 
*/
u8 sumCheck(u8 *buff,u16 len);
u8 crc8Check(u8 *buff,u16 len);
u16 crc16Check(u8 *buff,u32 len);
/*
*return 0: success
*return 1: no frame in buff
*return 2: crc8 error
*/
int unpackFrame(u8 *buff,u16 len,frame *fram);
void packFrame(u8 *buff,frame *fram);
void sendtoComputer(int timestamp_doing, int auto_aim, int big_buff);
void sendtoComputerInit(void);
void ChangeToAuto(void);
void ChangeToManual(void);
void ChangeToPower(void);
void ChangeToBigPower(void);
void PowersendtoComputer(void);
void GyrosendtoComputer(void);

#endif
