#include"gpio.h"
#include "stm32f4xx.h"


void gpio_init()
{
		
	RCC->AHB1ENR |=1<<0;
	GPIOA->MODER |=(0x1<<(5*2));

}

void button_init_inteerupt()
{
	
	__disable_irq();
	RCC->AHB1ENR |=1<<2;
	GPIOC->MODER &=~(0x3<<(13*2));
	
	RCC->APB2ENR |=1<<14;
	SYSCFG->EXTICR[3] |=(2<<4);
	EXTI->IMR |=1<<13;
	EXTI->FTSR |=1<<13;
	
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	__enable_irq();
}

void led(bool status)
{
	if(status)
	GPIOA->ODR |=1<<5;
	else
		GPIOA->ODR &=~(1<<5);
}