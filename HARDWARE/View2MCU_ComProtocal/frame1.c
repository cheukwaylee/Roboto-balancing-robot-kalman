#include "main.h"
#include <stdio.h>
#include <string.h>
#include "define.h"
#include "frame.h"
#include "crc_table.h"

//created by sui qi
//modified by twb 2017.7.18 add send module

frame sendtoCom_frame;
power_frame power_data_to_com;
gyro_data gyro_data_to_com;
u8 sendbuffer[100];



u8 sumCheck(u8 *buff,u16 len)
{
    u16 i=0;
    u32 sum=0;
    for(i=0;i<len;i++)
        sum+=buff[i];
    return (u8)sum&0x000000ff;
}

u8 crc8Check(u8 *buff,u16 len)
{
	u8 ucIndex,ucCRC8=(u8)CRC8_INIT;
	while (len--)
	{
		ucIndex = ucCRC8^(*buff++);
		ucCRC8 = CRC8_Table[ucIndex];
	}
	return(ucCRC8);
}

u16 crc16Check(u8 *buff,u32 len)
{
	u8 chData;
	u16 wCRC=(u16)CRC16_INIT;
	if(buff == NULL)
	{
		return 0xFFFF;
	}
	while(len--)
	{
		chData = *buff++;
		(wCRC) = ((u16)(wCRC) >> 8) ^ CRC16_Table[((u16)(wCRC) ^ (u16)(chData)) &0x00ff];
	}
	return wCRC;
}


int unpackFrame(u8 *buff,u16 len,frame *fram)
{	
	u16 i=0;
	while(buff[i]!=0xf1)	
	{
		i++;
		if(i>49)
			break;
	}	
	if(i==49)
	{
		if(buff[i]!=0xf1||buff[0]!=0xf1)
			return 1;
	}
	if(i<49&&buff[i+14]==0xf2)
	{
		
	  memcpy(fram,(buff+i),sizeof(frame));
	}
//	else
//	{
//		memcpy(fram,(buff+i),30-i);
//		memcpy(((u8 *)fram)+30-i,buff,sizeof(frame)-30+i);
//	}

	if(fram->crc8check==crc8Check((u8 *)fram,sizeof(frame)-2))
	{

		//memset(buff,0x00,64);
		return 0;
	}
	else
		return 2;
}

void packFrame(u8 *buff,frame *fram)
{	
	memcpy(buff,fram,sizeof(frame));
	buff[sizeof(frame)-2] = crc8Check((u8 *)fram,sizeof(frame)-1);
}

void packFrame1(u8 *buff,frame *fram)
{	
	memcpy(buff+6,((u8*)fram)+6,8);//只需向内存移入底盘偏移值
	buff[sizeof(frame)-1] = crc8Check((u8 *)fram,sizeof(frame)-1);
}
void GyrosendtoComputer(void)
{	
	gyro_data_to_com.head[0]=0xF0;
	gyro_data_to_com.head[1]=0xF1;
	
	memcpy(gyro_data_to_com.adi_raw_acce, (int16_t*)raw_busrt_data.data.accel, sizeof(raw_busrt_data.data.accel));
	memcpy(gyro_data_to_com.adi_raw_gyro, (int16_t*)raw_busrt_data.data.gyro, sizeof(raw_busrt_data.data.gyro));
	
	gyro_data_to_com.mpu_raw_acce[0]=mpu6500_raw_data.Accel_X;
	gyro_data_to_com.mpu_raw_acce[1]=mpu6500_raw_data.Accel_Y;
	gyro_data_to_com.mpu_raw_acce[2]=mpu6500_raw_data.Accel_Z;
	
	gyro_data_to_com.mpu_raw_gyro[0]=mpu6500_raw_data.Gyro_X;
	gyro_data_to_com.mpu_raw_gyro[1]=mpu6500_raw_data.Gyro_Y;
	gyro_data_to_com.mpu_raw_gyro[2]=mpu6500_raw_data.Gyro_Z;
	
	gyro_data_to_com.real_mpu_acce[0]=mpu6500_real_data.Accel_X;
	gyro_data_to_com.real_mpu_acce[1]=mpu6500_real_data.Accel_Y;
	gyro_data_to_com.real_mpu_acce[2]=mpu6500_real_data.Accel_Z;
	
	gyro_data_to_com.real_mpu_gyro[0]=mpu6500_real_data.Gyro_X;
	gyro_data_to_com.real_mpu_gyro[1]=mpu6500_real_data.Gyro_Y;
	gyro_data_to_com.real_mpu_gyro[2]=mpu6500_real_data.Gyro_Z;
	
	gyro_data_to_com.real_adi_acce[0]=adis16470_real_data.acce_x;
	gyro_data_to_com.real_adi_acce[1]=adis16470_real_data.acce_y;
	gyro_data_to_com.real_adi_acce[2]=adis16470_real_data.acce_z;
	
	gyro_data_to_com.real_adi_gyro[0]=adis16470_real_data.gyro_x;
	gyro_data_to_com.real_adi_gyro[1]=adis16470_real_data.gyro_y;
	gyro_data_to_com.real_adi_gyro[2]=adis16470_real_data.gyro_z;
	
	gyro_data_to_com.yaw=Yaw;
	gyro_data_to_com.pitch=Pitch;
	gyro_data_to_com.roll=Roll;
	
	gyro_data_to_com.pitch_target[0]=PitchTarget.Gyroscope;
	gyro_data_to_com.pitch_target[1]=PitchTarget.Mechanical;
	
	gyro_data_to_com.yaw_target[0]=YawTarget.Gyroscope;
	gyro_data_to_com.yaw_target[0]=YawTarget.Mechanical;
	
	
	
	
	
	gyro_data_to_com.tail[0]=0xF6;
	gyro_data_to_com.tail[1]=0xF8;
	USART2_Send((uint8_t*)&gyro_data_to_com, sizeof(gyro_data_to_com));
	
}
void PowersendtoComputer(void)
{	
	power_data_to_com.head[0]=0xFF;
	power_data_to_com.head[1]=0xF1;
	power_data_to_com.speed[0]=current_cm_201;
	power_data_to_com.speed[1]=current_cm_202;
	power_data_to_com.speed[2]=current_cm_203;
	power_data_to_com.speed[3]=current_cm_204;
	power_data_to_com.limite[0]=lec_numA;
	power_data_to_com.limite[1]=lec_numB;
	power_data_to_com.limite[2]=lec_numC;
	power_data_to_com.limite[3]=lec_numD;
	power_data_to_com.T_current[0]=t_i_1;
	power_data_to_com.T_current[1]=t_i_2;
	power_data_to_com.T_current[2]=t_i_3;
	power_data_to_com.T_current[3]=t_i_4;
	power_data_to_com.chassis_volt=ext_power_heat_data.chassis_volt;
	power_data_to_com.chassis_power_buffer=ext_power_heat_data.chassis_power_buffer;
	power_data_to_com.chassis_power=ext_power_heat_data.chassis_power;
	power_data_to_com.chassis_current=ext_power_heat_data.chassis_current;
	power_data_to_com.tail[0]=0xF2;
	power_data_to_com.tail[1]=0xF3;
	
	USART2_Send((uint8_t*)&power_data_to_com, sizeof(power_data_to_com));
}
void sendtoComputer(int timestamp_doing, int auto_aim, int big_buff)
{
//	sendtoCom_frame.yaw = (current_position_205-MIDDLE_YAW)*0.0439;
//	sendtoCom_frame.pitch =(current_position_206-MIDDLE_PITCH)*0.0439;
	sendtoCom_frame.head=0xf1;
	sendtoCom_frame.end=0xf2;
	
	sendtoCom_frame.timestamp = timestamp_doing;
	sendtoCom_frame.yaw = (current_position_205-MIDDLE_YAW);
	sendtoCom_frame.pitch = (current_position_206-MIDDLE_PITCH);
	if(auto_aim == 1)
		sendtoCom_frame.extra[0] = 0;
	if(big_buff == 1)
		sendtoCom_frame.extra[0] = 1;
	if(big_buff == 2)
		sendtoCom_frame.extra[0] = 2;
	
	packFrame(sendbuffer,&sendtoCom_frame);//减少每次搬运内存时间
	#ifdef AutoAim_USB
		USB_SendData(sendbuffer, strlen(sendbuffer));
	#else
  DMA_Cmd(DMA1_Stream6,ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
	#endif
}

void sendtoComputerInit(void)
{
  sendtoCom_frame.head=0xf1;
	sendtoCom_frame.end=0xf2;
	
	sendtoCom_frame.yaw = (current_position_205-MIDDLE_YAW)*0.0439;
	sendtoCom_frame.pitch = (current_position_206-MIDDLE_PITCH)*0.0439;
	
	packFrame(sendbuffer,&sendtoCom_frame);

}
void ChangeToAuto(void)//选择打开自瞄模式
{
	
	#ifdef little_board
	
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x212;	           // 标准标识符为0x200
		TxMessage.ExtId=0x00;	             // 设置扩展标示符（29位）
		TxMessage.IDE=CAN_ID_STD;					 // 使用标准帧模式
		TxMessage.RTR=CAN_RTR_DATA;				 // 发送的是数据
		TxMessage.DLC=8;							     // 数据长度为2字节
		TxMessage.Data[0]= 0x01;
		TxMessage.Data[1]= 0x01;	
		TxMessage.Data[2]= 0x00;
		TxMessage.Data[3]= 0x00;	
		TxMessage.Data[4]= 0x00;
		TxMessage.Data[5]= 0x00;	
		TxMessage.Data[6]= 0x00;
		TxMessage.Data[7]= 0x00;	
		
		CAN_Transmit(CAN2, &TxMessage);
	
	#else
	
		u32 AutoFlag[2]={1,1};
		memcpy(sendbuffer+14,&AutoFlag,8);
		
	#endif
}
void ChangeToManual(void)//选择打开手动模式
{

	#ifdef little_board
	
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x212;	           // 标准标识符为0x200
		TxMessage.ExtId=0x00;	             // 设置扩展标示符（29位）
		TxMessage.IDE=CAN_ID_STD;					 // 使用标准帧模式
		TxMessage.RTR=CAN_RTR_DATA;				 // 发送的是数据
		TxMessage.DLC=8;							     // 数据长度为2字节
		TxMessage.Data[0]= 0x00;
		TxMessage.Data[1]= 0x00;	
		TxMessage.Data[2]= 0x00;
		TxMessage.Data[3]= 0x00;	
		TxMessage.Data[4]= 0x00;
		TxMessage.Data[5]= 0x00;	
		TxMessage.Data[6]= 0x00;
		TxMessage.Data[7]= 0x00;	
		
		CAN_Transmit(CAN2, &TxMessage);
	
	#else
	
		u32 AutoFlag=0;
		memcpy(sendbuffer+14,&AutoFlag,4);
		
	#endif
	
}
void ChangeToPower(void)//选择打开能量机关模式
{

	#ifdef little_board
	
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x212;	           // 标准标识符为0x200
		TxMessage.ExtId=0x00;	             // 设置扩展标示符（29位）
		TxMessage.IDE=CAN_ID_STD;					 // 使用标准帧模式
		TxMessage.RTR=CAN_RTR_DATA;				 // 发送的是数据
		TxMessage.DLC=8;							     // 数据长度为2字节
		TxMessage.Data[0]= 0x01;
		TxMessage.Data[1]= 0x03;	
		TxMessage.Data[2]= 0x00;
		TxMessage.Data[3]= 0x00;	
		TxMessage.Data[4]= 0x00;
		TxMessage.Data[5]= 0x00;	
		TxMessage.Data[6]= 0x00;
		TxMessage.Data[7]= 0x00;	
		
		CAN_Transmit(CAN2, &TxMessage);
	
	#else
	
		u32 AutoFlag[2]={1,3};
		memcpy(sendbuffer+14,&AutoFlag,8);
		
	#endif
}
void ChangeToBigPower(void)//选择打开能量机关模式
{

	#ifdef little_board
	
		CanTxMsg TxMessage;
		
		TxMessage.StdId=0x212;	           // 标准标识符为0x200
		TxMessage.ExtId=0x00;	             // 设置扩展标示符（29位）
		TxMessage.IDE=CAN_ID_STD;					 // 使用标准帧模式
		TxMessage.RTR=CAN_RTR_DATA;				 // 发送的是数据
		TxMessage.DLC=8;							     // 数据长度为2字节
		TxMessage.Data[0]= 0x01;
		TxMessage.Data[1]= 0x04;	
		TxMessage.Data[2]= 0x00;
		TxMessage.Data[3]= 0x00;	
		TxMessage.Data[4]= 0x00;
		TxMessage.Data[5]= 0x00;	
		TxMessage.Data[6]= 0x00;
		TxMessage.Data[7]= 0x00;	
		
		CAN_Transmit(CAN2, &TxMessage);
	
	#else
	
		u32 AutoFlag[2]={1,4};
		memcpy(sendbuffer+14,&AutoFlag,8);
		
	#endif
}






