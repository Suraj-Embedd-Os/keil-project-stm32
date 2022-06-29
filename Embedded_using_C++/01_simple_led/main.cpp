

#include "mCal_reg.h"

#define RCC_AHB1ENR (*((volatile uint32_t *)0x40023800))
#define GPIO_MODER 	(*((volatile uint32_t *)0x40020000))
#define GPIOA_ODR		(*((volatile uint32_t *)0x40020014))


class Led
{
	public:
		typedef std::uint32_t port_type;
		typedef std::uint32_t bval_type;
	
	Led(const port_type p, bval_type b):port(p),bval(b){
			*reinterpret_cast<volatile port_type*>(port) &=~(1U<<bval); 
		
		//set pin to output
		const port_type gpio_mode_reg = port -0x14;
		*reinterpret_cast<volatile port_type*>(gpio_mode_reg) |=(1U<<(bval*2));
	}
	
	void toggle()const{
		*reinterpret_cast<volatile port_type*>(port) ^=(1U<<bval);
	}
	void psedu_delay(int n)
	{
		int i;
		for(; n>0;n--){
		for(i=0;i<3200;i++){}
		}
	}
	private:
		const port_type  port;
		const bval_type		bval;
};


int main()
{
	
	RCC_AHB1ENR |=(1U<<0);
	
	Led led5(mcal::reg1::gpioa,mcal::reg1::gpioa_pin5);
	
	while(1)
	{
		
		led5.toggle();
		for(int i=0;i<10000000;i++);
	}
}

