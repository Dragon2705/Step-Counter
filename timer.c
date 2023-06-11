#include <stm32f4xx.h>
#include "timer.h"
#include <stdio.h>
#include "mpu6050.h"
#include "lcd1602.h"


extern uint8_t flag;
extern uint32_t Steps;
double smoothedVerticalAcc = 0.0;
double Acc_old = 1;
int8_t Acc_check = -1;
int8_t led_green = 0;


double smoothedVerticalAcc1 = 0;
double Acc1 = 0;
void TIM5Config (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable Timer clock
	2. Set the prescalar and the ARR
	3. Enable the Timer, and wait for the update Flag to set
	************************************************/

// 1. Enable Timer clock
	RCC->APB1ENR |= (1<<3);  // Enable the timer5 clock
	
// 2. Set the prescalar and the ARR
	TIM5->PSC = 2800-1;  // 84MHz/84 = 1 MHz ~~ 1 uS delay
	TIM5->ARR = 499;  // MAX ARR value
	
// 3. Enable the Timer, and wait for the update Flag to set
	TIM5->CR1 |= (1<<0); // Enable the Counter
	TIM5->CR1 |= (1<<7); //Auto-reload preload enable
	TIM5->EGR = 1;
	TIM5->DIER = 1;
	NVIC_SetPriority(TIM5_IRQn, 2);
	NVIC_EnableIRQ(TIM5_IRQn);
}
char bufff[17];
void TIM5_IRQHandler(){
	if(TIM5->SR & (1<<0)){
		TIM5->SR &= ~(1<<0); // clear update interrupt flag
			if(flag == 0){
				double Acc = Vertical_Acc();
				sprintf (bufff, "%f", Acc);
				lcd_put_cur(1,0);
				lcd_send_string ("Acc: ");
				lcd_send_string (bufff);
				smoothedVerticalAcc  = Acc1 * 0.0155+ 0.0155 * Acc + smoothedVerticalAcc1 * 0.969;
				if((Acc_check > 0) && (smoothedVerticalAcc < Acc_old) && (smoothedVerticalAcc > 1.2)){
						Steps++;
						lcd_put_cur(0,0);
						char buf[17];
						lcd_send_string ("Step: ");
						sprintf (buf, "%d", Steps);
						lcd_send_string (buf);
				}
				Acc1 = Acc;
				smoothedVerticalAcc1 = smoothedVerticalAcc;
				Acc_check = smoothedVerticalAcc > Acc_old ? 1 : -1;
				Acc_old = smoothedVerticalAcc;
				
				if(led_green == 15){
						GPIOA->ODR ^= (1<<0);
						led_green = 0;
				}else{
						led_green ++;
				}
			}
	}
}
