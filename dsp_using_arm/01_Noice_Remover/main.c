
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <stdint.h>

extern  void SystemClock_Config(void);
uint32_t freq;
uint32_t counter;
int main()
{
	HAL_Init();
	SystemClock_Config();
	freq=HAL_RCC_GetHCLKFreq();
	while(1){
		counter++;
		if(counter==100)counter=0;
		for(int i=0;i<3000;++i);
	}
	
	return 0;
}

void SysTick_Handler()
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}