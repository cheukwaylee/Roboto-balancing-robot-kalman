#include "main.h"

JYDATA_STRUCT gyro_speed_degreepers, gyro_angle_degree;
float finall_yaw_degree=0;
float pre_yaw_degree=0;
int N2pi=0;

//u8 sumCheck(u8 *buff,u16 len)
//{
//    u16 i=0;
//    u32 sum=0;
//    for(i=0;i<len;i++)
//        sum+=buff[i];
//    return (u8)sum&0x000000ff;
//}


void uart3_update_from_dma(void)
{
	u8 USART_RX_BUF3_temp1[11];
	u8 USART_RX_BUF3_temp2[11];
	for(int i=0;i<MAXLBUF_3;i++)
	{
		if(i<11)
			USART_RX_BUF3_temp1[i]=USART_RX_BUF3[i];
		else
			USART_RX_BUF3_temp2[i-11]=USART_RX_BUF3[i];
	}
	u16 check_re1;
  u16	check_re2;
	if((short)USART_RX_BUF3_temp1[0]==0x55)
		check_re1=sumCheck(USART_RX_BUF3_temp1,10);
	if((short)USART_RX_BUF3_temp2[0]==0x55)
		check_re2=sumCheck(USART_RX_BUF3_temp2,10);
	
	if(check_re1==USART_RX_BUF3_temp1[10]&&(short)USART_RX_BUF3_temp1[1]==0x52)
	{
		gyro_speed_degreepers.x_value=(short)(USART_RX_BUF3_temp1[3]<<8|USART_RX_BUF3_temp1[2])/32768.0*2000;//单位为°每秒
		gyro_speed_degreepers.y_value=(short)(USART_RX_BUF3_temp1[5]<<8|USART_RX_BUF3_temp1[4])/32768.0*2000;//单位为°每秒
		gyro_speed_degreepers.z_value=(short)(USART_RX_BUF3_temp1[7]<<8|USART_RX_BUF3_temp1[6])/32768.0*2000;//单位为°每秒
		gyro_speed_degreepers.T=(short)(USART_RX_BUF3_temp1[9]<<8|USART_RX_BUF3_temp1[8])/100.0;//单位为°C			
	}
	
	if(check_re2==USART_RX_BUF3_temp2[10]&&(short)USART_RX_BUF3_temp2[1]==0x53)
	{
		gyro_angle_degree.x_value=(short)(USART_RX_BUF3_temp2[3]<<8|USART_RX_BUF3_temp2[2])/32768.0*180.0;//单位为°
		gyro_angle_degree.y_value=(short)(USART_RX_BUF3_temp2[5]<<8|USART_RX_BUF3_temp2[4])/32768.0*180.0;//单位为°
		gyro_angle_degree.z_value=(short)(USART_RX_BUF3_temp2[7]<<8|USART_RX_BUF3_temp2[6])/32768.0*180.0;//单位为°
		gyro_angle_degree.T=(short)(USART_RX_BUF3_temp2[9]<<8|USART_RX_BUF3_temp2[8])/100.0;//单位为°C
	}
	
	if(gyro_angle_degree.z_value>0&&pre_yaw_degree<0)
	{
		N2pi-=1;
	}
	else if(gyro_angle_degree.z_value<0&&pre_yaw_degree>0)
	{
		N2pi+=1;
	}
	finall_yaw_degree=gyro_angle_degree.z_value+N2pi*359;
	pre_yaw_degree=gyro_angle_degree.z_value;
		
}






