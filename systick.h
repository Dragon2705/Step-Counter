#ifndef SYSTICK_H
#define SYSTICK_H

#include "stm32f4xx.h"
#include "stdint.h"

void sysTick_Init(void);
void Delay_ms (uint32_t ms);
void Delay_us (uint32_t us);


#endif