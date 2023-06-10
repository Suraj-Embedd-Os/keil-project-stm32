

#include"usart.h"
#include"systick.h"
#include"tim.h"


int main()
{
	Mx_Usart_Init();	
	systic_init();
	tim2_init();
	
	while(1)
	{
		printf("main running\n\r");
		tim_delay_ms(2000);
		
		
	}
	
}














