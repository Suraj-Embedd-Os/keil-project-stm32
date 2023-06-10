
#include"usart.h"


void Mx_Usart_Init(void)
{
	//pa2=tx,pa3=rx
	RCC->AHB1ENR |=(1<<0);
	RCC->APB1ENR |=(1<<17);
	
	GPIOA->MODER |=(0x2<<(2*2));
	GPIOA->MODER |=(0x2<<(3*2));
	
	GPIOA->PUPDR |=(0x1<<(3*2));
	GPIOA->PUPDR |=(0x2<<(2*2));
	
	GPIOA->AFR[0] |=(0x7<<(3*4));
	GPIOA->AFR[0] |=(0x7<<(2*4));
	
	
	
	USART2->CR1 |=(1<<13);
	USART2->BRR	=0x8B;
	USART2->CR1 |=(0x3<<2);
	
}

int Usart_write(int ch)
{
	   while(!(USART2->SR & (1<<7))){}
	   USART2->DR =(ch&0xFF);
		 return ch;
}

int Usart_read(void)
{
	  while(!(USART2->SR & (1<<5))){}
		return (USART2->DR & 0xFF);
}



struct __FILE {int handler;};

FILE  __stdin   = {0};
FILE  __stdout  = {1};
FILE  __stderr  = {2};


int fgetc(FILE *f)
{
	int c;
	c=Usart_read();
	if(c=='\r')
	{
		Usart_write(c);
		c='\n';
	}
	//Usart_write(c);
	return c;
}


int fputc(int c ,FILE *f)
{
	return Usart_write(c);
}

