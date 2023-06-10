

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
	
	while(1)
	{
		led(0);
		printf("main running\n\r");
		tim_delay_ms(1000);
		led(1);
		tim_delay_ms(1000);
		
		
	}
	
}














