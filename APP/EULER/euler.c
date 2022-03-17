/************************************************************
 *File		:	euler.c
 *Author	:  @YangTianhao ,490999282@qq.com,@TangJiaxin ,tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.03.02
 *Description: 	euler caluation functions
 ************************************************************/

#include "euler.h"
#include "math.h"
#include "main.h"
#define euler_dt 0.001f
float Yaw_gyro,Roll_gyro,Pitch_gyro;
float Yaw_mag,Roll_accel,Pitch_accel;
float Yaw,Roll,Pitch,Yaw_Offset,Pitch_Offset,Roll_Offset;
float Pitch_estim, Pitch_gyro_estim; 		//estimations done by the Kalman filter (by means of the predictions)
int count = 0;
float delay_speed=0;

int euler_count=0;
double yaw_zero_float=0;

float invSqrt(float num);

void init_euler(void){
	
	unsigned char i;
	float ax,ay,az,a;
	ax=ay=az=0.0f;
	for(i=0;i<5;i++){
		real_data_update();
		ax+=adis16470_real_data.acce_x;
		ay+=adis16470_real_data.acce_y;
		az+=adis16470_real_data.acce_z;
		
		delay_ms(5);
	}
	if(ax==0&&ay==0&&az==0)
	{
			for(i=0;i<5;i++){
			MPU6500_Read();
			ax+=mpu6500_real_data.Accel_X;
			ay+=mpu6500_real_data.Accel_Y;
			az+=mpu6500_real_data.Accel_Z;
			
			delay_ms(5);
		}
	}
	a=1.0f/sqrt(ax*ax+ay*ay+az*az);
	ax*=a;
	ay*=a;
	az*=a;
	Roll=atan2(-ay,az);
	Pitch=asin(ax);
	Yaw=0;
	Pitch_gyro=0;
	
	Pitch_estim = 0;						//initial value of the angular position estimated by the Kalman filter
	Pitch_gyro_estim = 0;				//initial value of the angular velocity estimated by the Kalman filter
}
/*


*/

void update_euler(void){
	if(euler_count<=1500)
	{
		euler_count++;
	}
	if(euler_count==1000)
	{
		yaw_zero_float=Yaw/2000.0f;
	}
	
	real_data_update();
	MPU6500_Read();
	
	float Roll_sin,Roll_cos,Pitch_sin,Pitch_cos;
	float a,gx,gy,gz,dx,dy,dz;
	static const float K=1.0f;  //?????????? ?? ????????
	static const float KP=0.5f; //?????P????? ?????????
	Roll_sin=sin(Roll);
	Roll_cos=cos(Roll);
	Pitch_sin=sin(Pitch);
	Pitch_cos=cos(Pitch);
	delay_speed=adis16470_real_data.gyro_z*57.3f;


	a=invSqrt(adis16470_real_data.acce_x*adis16470_real_data.acce_x
		+adis16470_real_data.acce_y*adis16470_real_data.acce_y
	+adis16470_real_data.acce_z*adis16470_real_data.acce_z);
	adis16470_real_data.acce_x*=a;
	adis16470_real_data.acce_y*=a;
	adis16470_real_data.acce_z*=a;

	dx=adis16470_real_data.acce_y*Pitch_cos*Roll_cos-adis16470_real_data.acce_z*Pitch_cos*Roll_sin;
	dy=adis16470_real_data.acce_z*(-Pitch_sin)-adis16470_real_data.acce_x*Pitch_cos*Roll_cos;
	dz=adis16470_real_data.acce_x*Pitch_cos*Roll_sin+adis16470_real_data.acce_y*Pitch_sin;

	gx=adis16470_real_data.gyro_x+KP*dx;
	gy=adis16470_real_data.gyro_y+KP*dy;
	gz=adis16470_real_data.gyro_z+KP*dz;

	
	Roll=Roll+
	(Pitch_cos*gx+Pitch_sin*Roll_sin*gy+Pitch_sin*Roll_cos*gz)/Pitch_cos*euler_dt;
	
	Pitch_gyro = (Roll_cos*gy-Roll_sin*gz)*euler_dt;
	Pitch=Pitch+Pitch_gyro;
	
	
	Yaw_gyro=Yaw+
	(Roll_sin*gy+Roll_cos*gz)/Pitch_cos*euler_dt;
	

	if(Roll>PI){	
		Roll-=2.0f*PI;
	}else if(Roll<-PI){
		Roll+=2.0f*PI;
	}
	
	if(Pitch>PI/2.0f){	
		Pitch-=PI;
	}else if(Pitch<-PI/2.0f){
		Pitch+=PI;
	}
	
	if(euler_count<500)
		Yaw=0;
	else
		Yaw=K*Yaw_gyro+(1.0f-K)*Yaw_mag-dynamic_zero_float_offset;
	//Yaw=K*Yaw_gyro+(1.0f-K)*Yaw_mag-yaw_zero_float-dynamic_zero_float_offset;
	
	if(count <= 10)    //??offset?
	{	
		Yaw_Offset = Yaw ;
		Pitch_Offset = 0;
		Roll_Offset = 0;
		
		count++;
	}
	
}
float Yaw_temp;
void update_euler_mpu(void){
	
	float Roll_sin,Roll_cos,Pitch_sin,Pitch_cos;
	float a,gx,gy,gz,dx,dy,dz;
	static const float K=1.0f;  //?????????? ?? ????????
	static const float KP=0.5f; //?????P????? ?????????
	Roll_sin=sin(Roll);
	Roll_cos=cos(Roll);
	Pitch_sin=sin(Pitch);
	Pitch_cos=cos(Pitch);
	
	MPU6500_Read();
	delay_speed=mpu6500_real_data.Gyro_Z * 57.3f;
	a=invSqrt(mpu6500_real_data.Accel_X*mpu6500_real_data.Accel_X
		+mpu6500_real_data.Accel_Y*mpu6500_real_data.Accel_Y
	+mpu6500_real_data.Accel_Z*mpu6500_real_data.Accel_Z);
	mpu6500_real_data.Accel_X*=a;
	mpu6500_real_data.Accel_Y*=a;
	mpu6500_real_data.Accel_Z*=a;

	dx=mpu6500_real_data.Accel_Y*Pitch_cos*Roll_cos-mpu6500_real_data.Accel_Z*Pitch_cos*Roll_sin;
	dy=mpu6500_real_data.Accel_Z*(-Pitch_sin)-mpu6500_real_data.Accel_X*Pitch_cos*Roll_cos;
	dz=mpu6500_real_data.Accel_X*Pitch_cos*Roll_sin+mpu6500_real_data.Accel_Y*Pitch_sin;

	gx=mpu6500_real_data.Gyro_X+KP*dx;
	gy=mpu6500_real_data.Gyro_Y+KP*dy;
	gz=mpu6500_real_data.Gyro_Z+KP*dz;
	
	
	Roll=Roll+
	(Pitch_cos*gx+Pitch_sin*Roll_sin*gy+Pitch_sin*Roll_cos*gz)/Pitch_cos*euler_dt;
	Pitch_gyro =(Roll_cos*gy-Roll_sin*gz)*euler_dt;
	Pitch=Pitch+Pitch_gyro;
	
	Yaw_temp = (Roll_sin*gy+Roll_cos*gz)/Pitch_cos*euler_dt;
	Yaw_gyro=Yaw+ Yaw_temp;

	
	
	
	if(Roll>PI){	
		Roll-=2.0f*PI;
	}else if(Roll<-PI){
		Roll+=2.0f*PI;
	}
	
	if(Pitch>PI/2.0f){	
		Pitch-=PI;
	}else if(Pitch<-PI/2.0f){
		Pitch+=PI;
	}
	

	Yaw=K*Yaw_gyro+(1.0f-K)*Yaw_mag-dynamic_zero_float_offset;
	
	if(count <= 10)    //??offset?
	{	
		Yaw_Offset = Yaw ;
		Pitch_Offset = 0;
		Roll_Offset = 0;
		
		count++;
	}
	
}






float invSqrt(float num) {
	float halfnum = 0.5f * num;
	float y = num;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfnum * y * y));
	return y;
}
