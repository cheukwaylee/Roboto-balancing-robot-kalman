#include "main.h"

void SPI5Init(void){
	unsigned short temp=0;
	
	RCC->AHB1ENR|=1<<5;
	RCC->APB2ENR|=1<<20;
		
	GPIO_InitTypeDef   gpio;
	gpio.GPIO_Pin = GPIO_Pin_7 |  GPIO_Pin_8|  GPIO_Pin_9  ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOF, &gpio);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,5); 
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,5);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,5);

	gpio.GPIO_Pin = GPIO_Pin_6  ;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOF, &gpio);
	PFout(6)=1;
	
	RCC->APB2RSTR|=1<<20;	//复位SPI5
	RCC->APB2RSTR&=~(1<<20); //停止复位SPI5
	temp|=0<<10;	//全双工模式	
	temp|=1<<2;	  //SPI主机  
	temp|=1<<8;
	temp|=0<<11;  //8位数据格式	
	temp|=1<<1;   //空闲模式下SCK为1 CPOL=1 
	temp|=1<<0;   //数据采样从第2个时间边沿开始,CPHA=1  
	temp|=1<<9;   //软件nss管理
	temp|=7<<3;   //Fsck=Fpclk/256
	temp|=0<<7;   //MSB First  
	temp|=1<<6;   //SPI启动 
	SPI5->CR1=temp;
	SPI5->I2SCFGR&=~(1<<11);//选择SPI模式
}

unsigned char SPI5ReadWriteByte(unsigned char TxData){
	unsigned short retry=0;
	while((SPI5->SR&1<<1)==0){
		retry++;
		if(retry>600){
			return 0;
		}
	}
	SPI5->DR=TxData;
	retry=0;
	while((SPI5->SR&1<<0)==0){
		retry++;
		if(retry>600){
			return 0;
		}
	}
	return SPI5->DR;
}

void SPI4Init(void){
	SPI_InitTypeDef SPI_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4, ENABLE);
		
	GPIO_InitTypeDef   gpio;
	gpio.GPIO_Pin = GPIO_Pin_12 |  GPIO_Pin_5|  GPIO_Pin_6  ;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOE, &gpio);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_SPI4); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_SPI4);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_SPI4);

	gpio.GPIO_Pin = GPIO_Pin_4  ;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &gpio);
	SPI4_NS = 1;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI4, &SPI_InitStructure);
	/* ?? FLASH_SPI */
	SPI_Cmd(SPI4, ENABLE);



}

unsigned short SPI4ReadWriteByte(unsigned short TxData){
	unsigned short retry=0;
	while((SPI4->SR&SPI_SR_TXE)==0);
	SPI4->DR=TxData;
	retry=0;
	while((SPI4->SR&SPI_SR_RXNE)==0 && retry<10000){
		retry++;
	}
	return SPI4->DR;
}
