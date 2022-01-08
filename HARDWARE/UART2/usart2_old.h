#ifndef __USART2_H__
#define __USART2_H__

#include "main.h"

#define MAXLBUF 64

extern u8 USART_RX_BUF2[MAXLBUF];  

void USART2_DMA_Init(void);
void USART2_Send(u8 *buffer, u8 len);


#endif
