
#include"tim.h"
#include "stm32f4xx.h"
#include "stm32f446xx.h"

void tim2_init()
{
	
	//enable pherpheral clock
	RCC->APB1ENR |=1<<0;
	//select presscaler
	TIM2->PSC =16000-1; // 16 000 000/1600 = 10 000
	
	//select auto reload
	TIM2->ARR =1000-1;
	TIM3->CNT=0;

	
}

void tim2_OC_init()
{
	
	//enable pherpheral clock
	RCC->APB1ENR |=1<<0;
	//select presscaler
	TIM2->PSC =16000-1; // 16 000 000/1600 = 10 000
	
	//select auto reload
	TIM2->ARR =1000-1;
	TIM2->CNT=0;
	
	TIM2->CCMR1 &=~(7<<4);
	TIM2->CCMR1 |=(3<<4);
	TIM2->CCER |=1<<0;
	
	
	RCC->AHB1ENR |=1<<0;
	GPIOA->MODER |=(0x2<<(5*2));
	GPIOA->AFR[0] |=(0x1<<(4*5));
	TIM2->CR1 |=1<<0;

	
}



void tim3_IC_init()
{
	
	//enable pherpheral clock
	RCC->APB1ENR |=1<<0;
	//select presscaler
	TIM3->PSC =16000-1; // 16 000 000/16000 = 1000
	
	//select auto reload
	TIM3->ARR =60000-1;
	TIM3->CNT=0;
	
	TIM3->CCMR1 &=~(3<<0);
	TIM3->CCMR1 |=(2<<0);
	TIM3->CCER |=1<<0;
	
	
	RCC->AHB1ENR |=1<<0;
	GPIOA->MODER |=(0x2<<(5*2));
	GPIOA->AFR[0] |=(0x1<<(4*5));
	TIM2->CR1 |=1<<0;

	
}

void tim_delay_ms(int ms)
{
	
	//select auto reloadTIM
	TIM2->ARR =ms-1;
	TIM2->CNT=0;
	//enable timmer
	TIM2->CR1 |=1<<0;
	
	while(!(TIM2->SR & 1<<0)){}
			TIM2->SR &=~(1<<0);
		
		TIM2->CR1 &=~(1<<0);
}

