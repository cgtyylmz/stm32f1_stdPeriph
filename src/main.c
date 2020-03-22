#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void delay(int millis) {
    while (millis-- > 0) {
        volatile int x = 5971;
        while (x-- > 0) {
            __asm("nop");
        }
    }
}


int main()
{
	GPIO_InitTypeDef gpioInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	gpioInitStructure.GPIO_Pin = GPIO_Pin_13;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpioInitStructure);

	while(1)
	{
		GPIOC -> BSRR = GPIO_Pin_13;
		delay(100);
		GPIOC -> BRR = GPIO_Pin_13;
		delay(300);

	}
	return 0;
}

