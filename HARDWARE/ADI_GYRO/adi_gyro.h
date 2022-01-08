#ifndef __adi_gyro_h__
#define __adi_gyro_h__

void ADIS16470_BurstRead(void);
bool ADIS16470_Init(void);
void real_data_update(void);

typedef struct _ADIS16470_real_data
{
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float acce_x;
	float acce_y;
	float acce_z;
	int16_t checksum;
}ADIS16470_REAL_DATA;

extern ADIS16470_REAL_DATA adis16470_real_data;

extern __packed union __raw_busrt_data {
    uint16_t bytes[10];
    __packed struct {
        uint16_t diag_stat;
        int16_t gyro[3];
        int16_t accel[3];
        int16_t temp;
        uint16_t data_cnt;
        uint16_t checksum;
    } data;
		__packed struct {
				uint8_t bytes[20];
    } data2;
} raw_busrt_data;


#endif



