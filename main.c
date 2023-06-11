#include "stm32f4xx.h"                  // Device header
#include "timer.h"
#include "I2C.h"
#include "mpu6050.h"
#include "lcd1602.h"
#include "stdio.h"
#include "systick.h"
void GPIOConfig(void);

uint8_t flag = 0.;
uint32_t Steps =0;

int main (void)
{
	GPIOConfig ();
	sysTick_Init();
	I2C_Config();
	TIM5Config ();
	lcd_init ();
	MPU6050_Init();
	char buff[17];
	sprintf(buff, "%d", Steps);
	lcd_put_cur(0, 0);
	lcd_send_string("Step: ");
	lcd_send_string(buff);         
	while (1)
	{
	}
	
}

void GPIOConfig (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable GPIOA clock
	2. Set the PIN PA5 as output
	3. Configure the output mode
	************************************************/

	// 1. Enable GPIOA clock
	RCC->AHB1ENR |= (1<<0);  // Enable the GPIOA clock
	RCC->APB2ENR |= (1 << 14); //enable SYSCNFG clock
	
	// 2. Set the PIN PA6 as output led green
	GPIOA->MODER &= ~(3 <<0);  // pin PA6(bits 13:12 = 0:0) as output (01)
	GPIOA->MODER |= (1 << 0);
	GPIOA->OTYPER &= ~(1 << 0);
	
	//Set the PIN PA7 as output led red
	GPIOA->MODER &= ~(3 << 2);  // pin PA7(bits 15:14 = 0:0) as output (01)
	GPIOA->MODER |= (1 << 2);
	GPIOA->OTYPER &= ~(1 << 1);
	
	//Set the PIN PA6-BUTTON_PAUSE as output
	GPIOA->MODER &= ~(3 << 12);  // pin PA6(bits 13:12 = 0:0) as input (00);
	GPIOA->OTYPER &= ~(1 << 6); //PA6 push-pull
	GPIOA->PUPDR |= (1<<12);		//pull up mode
	GPIOA->ODR |= (1 << 6);
	//Interrupt config BUTTON_PAUSE
	SYSCFG->EXTICR[1] &= ~(0xf<<8); //Bits[8,9,10,11]=[0,0,0,0] -> config exti6 line for pa6
	EXTI->IMR |= (1<<6); //DISABLE mask on EXTI6 
	EXTI->FTSR  |= (1<<6); // enable falling edge trigger for PA6
	EXTI->RTSR &= ~(1<<6);// Disable Rising Edge Trigger for PA6
	
	//Set the PIN PA7-BUTTON_RESET as output
	GPIOA->MODER &= ~(3 << 14);  // pin PA7(bits 14:15 = 0:0) as input (00)
	GPIOA->OTYPER &= ~(1 << 7); //PA7 push-pull
	GPIOA->PUPDR |= (1<<14);		//PA7 up mode
	GPIOA->ODR |= (1 << 7);
	//Interrupt config BUTTON_RESET
	SYSCFG->EXTICR[1] &= ~(0xf<<12); //Bits[12,13,14,15]=[0,0,0,0] -> config exti7 line for pa7
	EXTI->IMR |= (1<<7); //DISABLE mask on EXTI7 
	EXTI->FTSR  |= (1<<7); // enable falling edge trigger for PA7
	EXTI->RTSR &= ~(1<<7);// Disable Rising Edge Trigger for PA7
	//ENABLE INTERRUPT
	NVIC_SetPriority (EXTI9_5_IRQn, 3);
	NVIC_EnableIRQ (EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler (void)
{
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************
	
	1. Check the Pin, which trgerred the Interrupt
	2. Clear the Interrupt Pending Bit
	
	********************************************************/
	if (EXTI->PR & (1<<6))    // If the PA1 triggered the interrupt
	{
		if(flag == 0){
			GPIOA->ODR |= (1 << 0);//green
			GPIOA->ODR |= (1 << 1);//red
			flag = 1;
		}
		else{
			GPIOA->ODR &= ~(1 << 1);
			GPIOA->BSRR &= (1<<16);
			flag = 0;
		}
		Delay_ms(200);			
		EXTI->PR |= (1<<6);  // Clear the interrupt flag by writing a 1 
	}else if(EXTI->PR & (1<<7)){	
					Steps = 0;
					lcd_put_cur(0,0);
					lcd_send_string("Step: 0 ");
		Delay_ms(200);			
		EXTI->PR |= (1<<7);  // Clear the interrupt flag by writing a 1 
	}
}
