
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

void tim_delay_ms(int ms)
{
	
	//select auto reload
	TIM2->ARR =ms-1;
	TIM3->CNT=0;
	//enable timmer
	TIM2->CR1 |=1<<0;
	
	while(!(TIM2->SR & 1<<0)){}
			TIM2->SR &=~(1<<0);
		
		TIM2->CR1 &=~(1<<0);
}

