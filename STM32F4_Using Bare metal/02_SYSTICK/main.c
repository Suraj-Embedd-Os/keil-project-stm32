

#include"usart.h"
#include"systick.h"

void test1(void)
{
	while(1)
	{
		printf("task2 running\n\r");
		//delayMs(1000);
		
	}
	
}
void test2(void)
{
	
	while(1)
	{
		printf("task2 running\n\r");
		//delayMs(1000);
		
	}
	
}

void (*fun)(void);
int main()
{
	Mx_Usart_Init();	
	systic_init();
	fun=test1;
	fun();
	while(1)
	{
		printf("main running\n\r");
		//delayMs(1000);
		
		
	}
	
}














