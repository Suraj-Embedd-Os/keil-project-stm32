

#include"systick.h"
#include "stm32f446xx.h"

#define SYSTEM_CLK	(16000000U)

volatile static unsigned int sys_count=0,sys_count_2=0;

extern void (*fun)(void);

void systic_init()
{
	SysTick->LOAD=SYSTEM_CLK/1000;
	SysTick->VAL=0;
	SysTick->CTRL=0;
	SysTick->CTRL|=0x1|0x4|0x2;

}

void delayMs(int ms)
{
	sys_count=ms;
	while(sys_count!=0){}
}
void SysTick_Handler()
{
	if(sys_count!=0)
	sys_count--;
	
}

