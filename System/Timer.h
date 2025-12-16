#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

extern volatile uint32_t micros_overflow;

void Timer_Init(void);
uint32_t micros(void);

#endif
