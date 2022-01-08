#ifndef __jy901_usart_h
#define __jy901_usart_h

typedef struct JYDATA
{
         float x_value,y_value,z_value;
         short T;
}JYDATA_STRUCT;

extern JYDATA_STRUCT gyro_speed_degreepers, gyro_angle_degree;


void uart3_update_from_dma(void);
extern float finall_yaw_degree;

#endif



