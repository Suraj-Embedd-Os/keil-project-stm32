#include"gpio.h"
#include "stm32f4xx.h"


void gpio_init()
{
		
	RCC->AHB1ENR |=1<<0;
	GPIOA->MODER |=(0x1<<(5*2));

}

void led(bool status)
{
	if(status)
	GPIOA->ODR |=1<<5;
	else
		GPIOA->ODR &=~(1<<5);
}