#ifndef _TIMER_H
#define _TIMER_H
#include "stm32f4xx.h"

//////////////////////////////////////////////////////////////////////////////////	 
extern u8 flag_Ready;
extern float adi_checksum;
extern u8 adi_die_flag;


void TIM6_Init(void);
void TIM5_Init(void);
void TIM2_Init(void);
void TIM9_Init(void);
extern u16 TIM6_time_count;
void TIM4_Init(void) ;
extern void NVIC_IRQEnable(uint8_t irq, uint8_t pri, uint8_t subpri);




#endif
