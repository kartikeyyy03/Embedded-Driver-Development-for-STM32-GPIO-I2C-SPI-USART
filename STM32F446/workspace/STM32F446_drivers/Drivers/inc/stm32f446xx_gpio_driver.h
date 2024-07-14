/*
 * stm32f407xx_gpio_driver.h
 *
 *
 *
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_


#include <stm32f446xx.h>




typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;          //possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;








//HANDLE STRUCTUTRE FOR GPIO PIN

typedef struct{

	GPIO_RegDef_t *pGPIOx;     //This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_Pin_Config;  //this holds GPIO pin configuration settings


}GPIO_Handle_t;


//@GPIO_PIN_MODES
#define GPIO_MODE_IN	 	0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4                        //falling edge trigger
#define GPIO_MODE_IT_RT     5                        //rising edge trigger
#define GPIO_MODE_IT_RFT    6                        //rising edge falling edge trigger






//GPIO PIN POSSIBLE OUTPUT TYPES



#define GPIO_OP_TYPE_PP		0                 //PUSH_PULL
#define GPIO_OP_TYPE_OD		1                 //OPEN DRAIN



//GPIO pin possible output speeds

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


//GPIO pin pull-up AND pull-down configuration macros


#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2


//@GPIO_PIN_NUMBERS
//GPIO pin numbers

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15







//*********************************************************************************************************
                             //APIs supported by this driver
//******************************************************************************************************************************
//peripheral clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init and de-Init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); //port is of 16 pins thats why uint_16
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


//IRQ Configuration and ISR Handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);





















#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
