
#include"stm32f4xx.h"
#include<stdint.h>

int __svc(0x00) sum(int,int);
int __svc(0x01) sub(int,int);
int __svc(0x02) mul(int,int);
int __svc(0x03) div(int,int);

void SVC_Handler_C(uint32_t *svc_arg);

int x,y,z;

int main()
{
	x=1;y=5;
	z=sum(x,y);
	
	x=9;y=5;
	z=sub(x,y);
	
	x=9;y=5;
	z=mul(x,y);
	
	x=9;y=5;
	z=div(x,y);
	
	while(1);
	
	
}


__ASM void SVC_Handler(void)
{
	TST  		LR,#04  //test LR 2 bit is zero
	ITE			EQ
	MRSEQ		R0,MSP
	MRSNE		R0,PSP
	B			__cpp(SVC_Handler_C)
}

void SVC_Handler_C(uint32_t *svc_arg)
{
	uint32_t svc_num;
	svc_num = ((char *)svc_arg[6])[-2];
	
	switch(svc_num)
	{
		case 0:
			svc_arg[0]=svc_arg[0]+svc_arg[1];
		break;
		case 1:
			svc_arg[0]=svc_arg[0]-svc_arg[1];
		break;
		case 2:
			svc_arg[0]=svc_arg[0]*svc_arg[1];
		break;
		case 3:
			svc_arg[0]=svc_arg[0]/svc_arg[1];
		break;
		default:
			break;
		
	}
	
}



