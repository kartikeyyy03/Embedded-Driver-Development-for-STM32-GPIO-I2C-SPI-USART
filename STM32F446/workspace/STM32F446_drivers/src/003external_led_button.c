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



	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_8;
	GpioLed.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	GpioLed.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);


	GPIOButton.pGPIOx=GPIOB;
	GPIOButton.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_12;
	GPIOButton.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
//	GPIOButton.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_OD; //matters only when the mode is output
	GPIOButton.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_PU;


		GPIO_PeriClockControl(GPIOB,ENABLE);

		GPIO_Init(&GpioLed);



	while(1){
        if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_12)==BUTTON_PRESSED){
        delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);}


	}




	return 0;







}
