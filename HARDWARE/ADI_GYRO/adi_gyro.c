/**
  ******************************************************************************
  * @file       adi_gyro.c
  * @author     YTom&REX_YYJ
  * @version    v0.0-alpha
  * @date       2019.Mar.14
  * @brief      Use spi_4 to read ADIS16470 IMU
  @verbatim      

  @endverbatim
  ******************************************************************************
  * @license
  *
  ******************************************************************************
  */

#include "main.h"
#include "adis16470_reg.h"

#define ADIS16470_ID        0x4056


ADIS16470_REAL_DATA adis16470_real_data;

uint16_t cmdA=0x00, cmdB=0x00;
uint16_t prod_id[2];
uint16_t _checksum=0;


#define ADIS_DEBUG_BUFFER
#ifdef ADIS_DEBUG_BUFFER
    uint16_t buffer[6]={0x00};
		u8 state=0;
		bool flag=false;
#endif
		


__packed union __raw_busrt_data raw_busrt_data;





bool ADIS16470_ReadByte(uint8_t addr, uint16_t* output) {
	/* Send out address */
	SPI4_NS= 0;
	SPI4ReadWriteByte((uint16_t)(addr<<8)&0xFF00);
	SPI4_NS= 1;
	delay_us(6);
	/* Read returned value */
	SPI4_NS= 0;
	uint16_t ret = SPI4ReadWriteByte(0x00);
	if(output) output[0] = ret;
	SPI4_NS= 1;
	delay_us(6);

	return true;
}



bool ADIS16470_WriteByte(uint8_t addr, uint16_t data) {
    /* If write 0x0004 to addr:0xDC 0xDD, should send 0xDC04 then 0xDD00 */
    cmdA = (((uint16_t)(addr|0x80)<<8)&0xFF00)|(data&0x00FF);
    cmdB = ((((uint16_t)(addr|0x80)+1)<<8)&0xFF00)|((data>>8)&0x00FF);
    
    SPI4_NS= 0;
    SPI4ReadWriteByte(cmdA);
    SPI4_NS= 1;
    //delay_us(10);
    SPI4_NS= 0;
    SPI4ReadWriteByte(cmdB);
    SPI4_NS= 1;
    //delay_us(10);
    uint16_t regval;
    bool ret = ADIS16470_ReadByte(addr, &regval);
    
    return true;
}



bool ADIS16470_DeviceIDCheck(void) {
    ADIS16470_ReadByte(ADIS_PROD_ID, &prod_id[0]);
    ADIS16470_ReadByte(ADIS_PROD_ID, &prod_id[1]);
    if((prod_id[0] == ADIS16470_ID) && (prod_id[1] == ADIS16470_ID)) {
        return true;
    }
    return false;
}

uint8_t ADIS16470_GetState(void) {
    uint16_t state = 9999;//给一个在状态值外的值防止报错
    ADIS16470_ReadByte(ADIS_DIAG_STAT, &state);
    return state;
}



void ADIS16470_BurstRead(void) {
		SPI4_NS= 0;
		SPI4ReadWriteByte(ADIS_BURST);
    /* Read returned value */
    for(uint8_t i=0; i<10; ++i) {
        raw_busrt_data.bytes[i] = SPI4ReadWriteByte(0x00);
    }
		SPI4_NS= 1;
		delay_us(10);

}

void real_data_update(void)
{
		for(int i=0;i<18;i++)
		{
			_checksum+=raw_busrt_data.data2.bytes[i];
		}
		if(_checksum==raw_busrt_data.data.checksum)
		{
					/*tranfer the row data to real data:gyro in (degree/sec)    accel in (g) */
		adis16470_real_data.gyro_x=raw_busrt_data.data.gyro[1]*0.1/360.0*6.28;
		adis16470_real_data.gyro_y=raw_busrt_data.data.gyro[0]*0.1/360.0*6.28;
		adis16470_real_data.gyro_z=raw_busrt_data.data.gyro[2]*0.1/360.0*6.28;
		adis16470_real_data.acce_x=raw_busrt_data.data.accel[1]*1.25/1000.0;
		adis16470_real_data.acce_y=raw_busrt_data.data.accel[0]*1.25/1000.0;
		adis16470_real_data.acce_z=raw_busrt_data.data.accel[2]*1.25/1000.0;
		}
		adi_checksum=raw_busrt_data.data.checksum;
		_checksum=0;
}







void ADIS16470_Reset(bool hardware) {
    if(hardware) {
        /* Hardware reset */
        GPIO_ResetBits(GPIOF,GPIO_Pin_1);
        delay_ms(50);
        GPIO_SetBits(GPIOF,GPIO_Pin_1);
        delay_ms(200);
    } else {
        ADIS16470_WriteByte(ADIS_GLOB_CMD,0x80);
        delay_ms(200);
    }
}


void GPIOF_Pin1_Init(void)
{
	GPIO_InitTypeDef 					GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//??PORTH??	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;   //GPIOI 2 7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//??100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //??
	GPIO_Init(GPIOF,&GPIO_InitStructure);              //???
	
	GPIO_SetBits(GPIOF,GPIO_Pin_1);
	
}

		
bool ADIS16470_Init(void) {
  
    bool ret = false;
    
    /* Hardware reset IMU */
		GPIOF_Pin1_Init();
    ADIS16470_Reset(true);
    
		flag=ADIS16470_DeviceIDCheck();
    
#ifdef ADIS_DEBUG_BUFFER
    ADIS16470_ReadByte(ADIS_MSC_CTRL, &buffer[0]);
    ADIS16470_ReadByte(ADIS_FILT_CTRL, &buffer[1]);
    ADIS16470_ReadByte(ADIS_DEC_RATE, &buffer[2]);
#endif
    
    /* Config IMU */
    /* [DR=high when data ready] */
//    // [internal clock mode: Fg = 2000Hz, Fa = 4000Hz][MSC_CTRL def=0x00C1]
//    ret = ADIS16470_WriteByte(ADIS_MSC_CTRL, 0x0001);   delay_us(10);
    /* Set internal FIR filter size variable 3 (means number of taps in each stage) */
    ret = ADIS16470_WriteByte(ADIS_FILT_CTRL, 0x0003);      delay_us(10);
    if(!ret) return false;
    /* When working in internal clock mode, nominal output data rate = 2000/(DEC_RATE+1) */
    ret = ADIS16470_WriteByte(ADIS_DEC_RATE, 0x0001);       delay_us(10);       // [1000Hz]
		
    if(!ret) return false;
    
#ifdef ADIS_DEBUG_BUFFER
    ADIS16470_ReadByte(ADIS_MSC_CTRL, &buffer[3]);
    ADIS16470_ReadByte(ADIS_FILT_CTRL, &buffer[4]);
    ADIS16470_ReadByte(ADIS_DEC_RATE, &buffer[5]);
#endif
    
    /* Update IMU parameters */
    state=ADIS16470_GetState();
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		
		GPIO_InitTypeDef   gpio;
		gpio.GPIO_Pin = GPIO_Pin_0;
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz; 
		GPIO_Init(GPIOF, &gpio);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, GPIO_PinSource0); 
    EXTI_InitTypeDef            exit_initer;
    exit_initer.EXTI_Line       = EXTI_Line0;
    exit_initer.EXTI_LineCmd    = ENABLE;
    exit_initer.EXTI_Mode       = EXTI_Mode_Interrupt;
    exit_initer.EXTI_Trigger    = EXTI_Trigger_Rising;
    EXTI_Init(&exit_initer);
    
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel=EXTI0_IRQn; //定时器3中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级2
		NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
		NVIC_Init(&NVIC_InitStructure);

    return true;
}


float yaw_rx=0;
void EXTI0_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line0)) {
		
		ADIS16470_BurstRead();
		
		EXTI_ClearITPendingBit(EXTI_Line0);
	}

}



