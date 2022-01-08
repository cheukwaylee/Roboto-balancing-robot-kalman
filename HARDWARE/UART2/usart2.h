#ifndef __USART2_H__
#define __USART2_H__

#include "stm32f4xx.h"

#define AUTO_FRAME_HEAD 0xf1
#define AUTO_FRAME_LENGTH 15

//typedef __packed struct _frame			//视觉发来的数据框架	
//{
//	u16 head;		//帧头，为0xf1f1
//	u16 timestamp;//包序号
//	float yaw;	  //yaw误差值
//	float pitch;  //pitch误差值
//	u8 extra[2]; //additional imformation	
//	u8 crc8check;	//crc8校验位	
//	u8 update_flag;
//}frame;

//typedef struct 			//视觉发来的数据框架	
//{
//	float yaw;	  //yaw误差值
//	float pitch;  //pitch误差值
//	float yaw_current;
//	float pitch_current;
//	u8 update_flag;
//	u8 shoot_flag;
//	
//}frame_update;.

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


void sendtoComputer(int timestamp_doing, int auto_aim, int big_buff,int entering_auto_aim);


void USART2_DMA_Init(void);
u8 crc8Check(u8 *buff,u16 len);
extern u8 update_flag;
extern frame fram;


#endif
