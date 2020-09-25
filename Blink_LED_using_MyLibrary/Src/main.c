#include "stm32f103_myLib.h"

#define TIME 400000
int main()
{
	RCC->APB2ENR |= RCC_APB2ENR_APB2EN(4); // set clock
	GPIOC->CRH |= GPIO_CRL_CRL(20) | GPIO_CRL_CRL(21); // set mode :output
	GPIOC->CRH &= ~(GPIO_CRL_CRL(22) | GPIO_CRL_CRL(23));
	while(1)
	{
		GPIOC->BSRR |= GPIO_BSRR_BSR(13); //led on
		for(int i =0;i <= TIME;i++);
		GPIOC->BSRR |= GPIO_BSRR_BSR(29); // led off
		for(int i =0;i <= TIME;i++);
	}
}
