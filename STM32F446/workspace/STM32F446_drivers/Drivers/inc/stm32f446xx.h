/*
 * stm32f407xx.h
 *
 *  Created on: Jun 12, 2024
 *      Author: Kartikeya Kandpal
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>
#include<stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/*************************Processor Specific Details*************************************************/
/*
* ARM Cortex Mx Processor NVIC ISER register Addresses
*/

#define NVIC_ISER0							((__vo uint32_t*)0xE00E100)
#define NVIC_ISER1							((__vo uint32_t*)0xE00E104)
#define NVIC_ISER2							((__vo uint32_t*)0xE00E108)
#define NVIC_ISER3							((__vo uint32_t*)0xE00E10C)

/*
* ARM Cortex Mx Processor NVIC ICER register Addresses
*/
#define NVIC_ICER0							((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1							((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2							((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3							((__vo uint32_t*)0xE000E18C)


//ARM Cortex Mx Processor Priority Register Address Calculation


#define NVIC_PR_BASE_ADDR					((__vo uint32_t*)0xE00E400)

//ARM Cortex Mx processor number of priority bits implemented in priority register

#define NO_PR_BITS_IMPLEMENTED				4


#define FLASH_BASEADDR						0x08000000U     //U TO DENOTE UNSIGNED INTEGER
#define SRAM1_BASEADDR                      0x20000000U     //SRAM1 ENDS AT 112 KBS
#define SRAM2_BASEADDR                      0x20001C00U     //SRAM2 FOLLOWS SRAM1 AND 112 IN HEXA IS 112*1024=1C00
#define ROM  								0x1FFF0000U //SYSTEM MEMORY BASE ADDRESS
#define SRAM SRAM1_BASEADDR 				SRAM1_BASEADDR

#define PERIPH_BASE 						0x40000000U
#define APB1PERIPH_BASEADDR 				PERIPH_BASE
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR 				0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U





//DEFINING THE PERIPHERALS HANGING ON THE AHB1 BUS: (AHB1PERIPH_BASEADDR + OFFSET)

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
//#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2000)//not available in STM32f446xx
#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)








//DEFINING THE PERIPHERALS HANGING ON THE APB1 BUS: (APB1PERIPH_BASEADDR + OFFSET)

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)




//DEFINING THE PERIPHERALS HANGING ON THE APB2 BUS: (APB2PERIPH_BASEADDR + OFFSET)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)



/***********************peripheral register definition structure**********************************************/

//peripheral register definition structure for GPIO
typedef struct{

	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];      //AFR[0]: ALTERNATE FUNCTION LOW REGISTER || AFR[1]: ALTERNATE FUNCTION HIGH REGISTER


}GPIO_RegDef_t;

//******************************************************************************************************************
//Peripheral register definition structure for RCC
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	 uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	 uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	 uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
      uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	 uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	 uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

}RCC_Reg_Def_t;

//peripheral register definition structure for EXTI:

typedef struct{
	__vo uint32_t		IMR;
	__vo uint32_t		EMR;
	__vo uint32_t		RTSR;
	__vo uint32_t		FTSR;
	__vo uint32_t		SWER;
	__vo uint32_t		PR;

}EXTI_RegDef_t;

//******************************************************************************************
//peripheral register definition structure for SYSCFG:

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;


//**************************************************************************************************

//PERIPHERAL REGISTER DEFINITION STRUCTURE FOR SPIx peripheral

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;




}SPI_RegDef_t;

//peripheral register definition structure for I2C:

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;








//PERIPHERAL DEFINITIONS (peripheral base addresses typecasted to xxx_RegDef_t)	:

#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
//#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)


#define RCC				((RCC_Reg_Def_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)


#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)


//*************************************************************************

//CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS


#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() 	(RCC->AHB1ENR |= (1 << 8))

//*************************************************************************


//*************************************************************************

//CLOCK ENABLE MACROS FOR I2C PERIPHERALS


#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21 ))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22 ))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23 ))


//*************************************************************************

//CLOCK ENABLE MACROS FOR SPIx PERIPHERALS

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

//*************************************************************************


//CLOCK ENABLE MACROS FOR USARTx PERIPHERALS

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

//*************************************************************************
//*************************************************************************

//CLOCK ENABLE MACROS FOR SYSCFG PERIPHERAL

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

//*************************************************************************
//*************************************************************************

//CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS

#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 7))
//#define GPIOI_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 8)) //not available in stm32f446xx

//*************************************************************************
//*************************************************************************

//CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS


#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21 ))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22 ))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23 ))

//*************************************************************************
//*************************************************************************

//*************************************************************************
//CLOCK DISABLE MACROS FOR SPIx PERIPHERALS
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
//#define SPI4_PCLK_EN()		(RCC->APB1ENR &= ~(1 << 13))
//*************************************************************************


//CLOCK DISABLE MACROS FOR USARTx PERIPHERALS

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

//*************************************************************************
//*************************************************************************

//CLOCK DISABLE MACROS FOR SYSCFG PERIPHERAL

#define SYSCFG_PCLK_DI()	(RCC->APB2 &= ~(1 << 14))


//*************************************************************************

//Macros to reset GPIOx Peripherals

#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<0));     (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<1));     (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<2));     (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<3));     (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<4));     (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<5));     (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<6));     (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<7));     (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1<<8));     (RCC->AHB1RSTR &= ~(1<<8));}while(0)



#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA) ? 0 :\
                                     (x == GPIOB) ? 1 :\
                                     (x == GPIOC) ? 2 :\
                                     (x == GPIOD) ? 3 :\
                                     (x == GPIOE) ? 4 :\
                                     (x == GPIOF) ? 5 :\
                                     (x == GPIOG) ? 6 :\
                                     (x == GPIOH) ? 7 :8)


/*
 * Macros to reset SPIx peripherals
 */


#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 12));    (RCC->APB2RSTR &= ~(1 << 12));}while(0);
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 14));    (RCC->APB1RSTR &= ~(1 << 14));}while(0);
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 15));    (RCC->APB1RSTR &= ~(1 << 15));}while(0);
#define SPI4_REG_RESET()	do{(RCC->APB2RSTR |= (1 << 13));    (RCC->APB2RSTR &= ~(1 << 13));}while(0);


/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 21));    (RCC->APB2RSTR &= ~(1 << 21));}while(0);
#define I2C2_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 22));    (RCC->APB2RSTR &= ~(1 << 22));}while(0);
#define I2C3_REG_RESET()	do{(RCC->APB1RSTR |= (1 << 23));    (RCC->APB2RSTR &= ~(1 << 23));}while(0);





/*IRQ(Interrupt Request) numbers of STM32F407x MCU
NOTE: update these macros with valid values according to your MCU*/


#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32



#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15
#define NVIC_IRQ_PRI16		16
#define NVIC_IRQ_PRI17		17
#define NVIC_IRQ_PRI118
#define NVIC_IRQ_PRI119

//some generic macros


#define ENABLE   				1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET                SET
#define FLAG_RESET				RESET

/**************************************************************************************
 * BIt position definition of SPI_CR1 Peripheral
 **********************************************************************/

#define SPI_CR1_CPHA					 0
#define SPI_CR1_BR						 3
#define SPI_CR1_DFF						 11
#define SPI_CR1_CPOL					 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	 10
#define SPI_CR1_CRCNEXT   			 	 12
#define SPI_CR1_CRCEN   			 	 13
#define SPI_CR1_BIDIOE     			 	 14
#define SPI_CR1_BIDIMODE      			 15




/**************************************************************************************
 * BIt position definition of SPI_CR2 Peripheral
 **********************************************************************/


#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7



/**************************************************************************************
 * BIt position definition of I2C_CR1 Peripheral
 **********************************************************************/

#define I2C_CR1_PE						0
#define I2C_CR1_SMBUS					1
#define I2C_CR1_SMBTYPE					3
#define I2C_CR1_ENARP					4
#define I2C_CR1_ENPEC					5
#define I2C_CR1_ENGC					6
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START					8
#define I2C_CR1_STOP					9
#define I2C_CR1_ACK						10
#define I2C_CR1_POS						11
#define I2C_CR1_PEC						12
#define I2C_CR1_ALERT					13
#define I2C_CR1_SWRST					15










/**************************************************************************************
 * BIt position definition of I2C_CR2 Peripheral
 **********************************************************************/

#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12









/**************************************************************************************
 * BIt position definition of I2C_SR1 Peripheral
 **********************************************************************/

#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RXNE					6
#define I2C_SR1_TXE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15














/**************************************************************************************
 * BIt position definition of I2C_SR2 Peripheral
 **********************************************************************/

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT				5
#define I2C_SR2_SMBHOST					6
#define I2C_SR2_DUALF					7
#define I2C_SR2_PEC						8

















/**************************************************************************************
 * BIt position definition of I2C_CCR Peripheral
 **********************************************************************/
#define I2C_CCR_CCR						0
#define I2C_CCR_DUTY					14
#define I2C_CCR_FS						15







#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"

#endif /* INC_STM32F446XX_H_ */
