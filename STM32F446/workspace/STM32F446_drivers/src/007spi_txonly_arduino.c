/*
 * 007spi_txonly_arduino.c
 *
 *
 */





/*
 * 006spi_tx_texting.c
 *
 *
/
 *
 *
 */
///* PB12--> SPI2_NSS
///* PB13--> SPI2_SCLK
///* PB14--> SPI2_MISO
///* PB15--> SPI2_MOSI
// * ALT function mode: 5
// */
#include "stm32f446xx_spi_driver.h"
#include  "stm32f446xx.h"
#include <string.h>

void delay(void){

	for(uint32_t i=0;i<500000/2;i++);
}






void SPI2_GPIOInits(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_Pin_Config.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_Pin_Config.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_Pin_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_Pin_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_Pin_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//SCLK
	SPIPins.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

//	//MISO
//	SPIPins.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}


void SPI2_Inits(void){

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx =SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;   //generates sclk of 8 mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SSM = SPI_SSM_DI;   //hardware slave management enabled for NSS pin




	SPI_Init(&SPI2handle);








}



void GPIO_Button_Init(void){

	GPIO_Handle_t GPIOButton;

	GPIOButton.pGPIOx=GPIOA;
		GPIOButton.GPIO_Pin_Config.GPIO_PinNumber=GPIO_PIN_NO_0;
		GPIOButton.GPIO_Pin_Config.GPIO_PinMode=GPIO_MODE_IN;
		GPIOButton.GPIO_Pin_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
	//	GPIOButton.GPIO_Pin_Config.GPIO_PinOPType=GPIO_OP_TYPE_OD; //matters only when the mode is output
		GPIOButton.GPIO_Pin_Config.GPIO_PinPuPdControl=GPIO_NO_PUPD;


//			GPIO_PeriClockControl(GPIOD,ENABLE);

			GPIO_Init(&GPIOButton);










}




int main(void){
	char user_data[] = "Hello World";

	GPIO_Button_Init();




	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();



    //This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 *making SSOE 1 does NSS output enable
	 *making the NSS pin is automatically managed by the hardware
	 *i.e., when SPE=1, NSS will be pulled to low
	 *and NSS pin will be high when SPE=0
	 */

	SPI_SSOEConfig(SPI2, ENABLE);


	while(1){
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();


	//	SPI_SSIConfig(SPI2, ENABLE);

		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);



		//first send length information

		uint8_t dataLen=strlen(user_data);

		SPI_SendData(SPI2,&dataLen,1);



		//send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));




		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));// This function will return 0 if SPI is not busy, otherwise will keep returning 1.



		//this function will be called only when the above function return 0 i.e., when SPI is not busy.
		SPI_PeripheralControl(SPI2, DISABLE);


	}





	return 0;
}

