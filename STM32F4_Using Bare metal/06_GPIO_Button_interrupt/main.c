

#include"usart.h"
#include"systick.h"
#include"tim.h"
#include"gpio.h"


int main()
{
	Mx_Usart_Init();	
	systic_init();
	tim2_init();
	gpio_init();
	button_init_inteerupt();
	
	while(1)
	{
		
			
		
	}
	
}



void EXTI15_10_IRQHandler()
{
	
	if(EXTI->PR & 1<<13)
	{
		EXTI->PR |=1<<13;
		GPIOA->ODR ^=(1<<5);
		
	}
	
}










