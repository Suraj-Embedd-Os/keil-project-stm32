

#include"usart.h"


int main()
{
	Mx_Usart_Init();
	int n;
	char str[100];
	printf("hello from other side\n\r");
	fprintf(stdout,"test for stdout\n\r");
	fprintf(stderr,"test for stderr\n\r");
	
	while(1)
	{
		printf("how old are u ?");
		scanf("%d",&n);
		printf("your age is : %d \n\r",n);
		printf("enter your name :");
		gets(str);
		printf("i like your stile :");
		puts(str);
		printf("\n\r");
		
	}
	
}














