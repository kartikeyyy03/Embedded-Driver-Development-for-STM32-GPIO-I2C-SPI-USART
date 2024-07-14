/*
 * 002led_button.c
*/



#include "stm32f407xx.h"
#define HIGH 				1
#define LOW 				0

#define BUTTON_PRESSED		LOW

void delay(void){

	for(uint32_t i=0;i<500000/2;i++);


}

int main(void){

	GPIO_Handle_t GpioLed, GPIOButton;


//LED GPIO CONFIG
	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GpioLed.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

//BUTTON CONFIG
	GPIOButton.pGPIOx=GPIOD;
	GPIOButton.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIOButton.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_IT_FT;
	GPIOButton.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
//	GPIOButton.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_OD; //matters only when the mode is output
	GPIOButton.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_PU;


		GPIO_PeriClockControl(GPIOB,ENABLE);

		GPIO_Init(&GPIOButton);


		//IRQ CONFIG
		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	return 0;

}

void EXTI_5_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}










