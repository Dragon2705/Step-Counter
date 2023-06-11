#include "systick.h"

uint32_t Tick_ms = 0;
uint32_t Tick_us = 0;

void sysTick_Init(void) {
		SysTick->LOAD |= 84 - 1  ;//84 - 1; //clock 84MHz //chia clock
		SysTick->CTRL |= (1 << 0) | (1 << 1) | (1<<2); //0x00000007
		NVIC_SetPriority(SysTick_IRQn, 1);
		NVIC_EnableIRQ(SysTick_IRQn);
}

void SysTick_Handler(){
	Tick_us++; 
}
void Delay_us (uint32_t us){
	while(Tick_us < us);
	Tick_us = 0;//reset
}

void Delay_ms (uint32_t ms){
	for (uint16_t i=0; i<ms; i++)
	{
		Delay_us (1000); // delay of 1 ms
	}
}




