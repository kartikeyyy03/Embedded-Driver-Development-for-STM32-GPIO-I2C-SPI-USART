/*
 * 002led_button.c
*/



#include "stm32f446xx.h"
#define HIGH 				1
#define BUTTON_PRESSED		HIGH

void delay(void){

	for(uint32_t i=0;i<500000/2;i++);


}

int main(void){

	GPIO_Handle_t GpioLed, GPIOButton;



	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_OD;
	GpioLed.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);


	GPIOButton.pGPIOx=GPIOA;
	GPIOButton.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_0;
	GPIOButton.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_IN;
	GPIOButton.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
//	GPIOButton.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_OD; //matters only when the mode is output
	GPIOButton.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;


		GPIO_PeriClockControl(GPIOD,ENABLE);

		GPIO_Init(&GPIOButton);



	while(1){
        if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)==BUTTON_PRESSED){
        delay();
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);}


	}




	return 0;







}
