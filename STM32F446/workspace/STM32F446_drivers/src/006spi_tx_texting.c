/*
 * 006spi_tx_texting.c
 *
 *
/
 *
 *
 */
/* PB12--> SPI2_NSS
/* PB13--> SPI2_SCLK
/* PB14--> SPI2_MISO
/* PB15--> SPI2_MOSI
 * ALT function mode: 5
 */

#include  "stm32f446xx.h"
#include <string.h>





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

	//MISO
	SPIPins.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_Pin_Config.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}


void SPI2_Inits(void){

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx =SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;   //generates sclk of 8 mhz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SSM = SPI_SSM_EN;




	SPI_Init(&SPI2handle);








}




int main(void){
	char user_data[] = "Hello World";
	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();




	SPI2_Inits();


	SPI_SSIConfig(SPI2, ENABLE);

	//enable SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);



	SPI_SendData(SPI2,(uint8_t*)user_data, strlen(user_data));

	//lets confirm SPI is not busy

	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG)); //this function will return 0 if SPI is not busy, otherwise will keep returning 0



    //this function will be called only when the above function returns 0 i.e., when SPI is not busy
	SPI_PeripheralControl(SPI2, DISABLE);



	return 0;
}

