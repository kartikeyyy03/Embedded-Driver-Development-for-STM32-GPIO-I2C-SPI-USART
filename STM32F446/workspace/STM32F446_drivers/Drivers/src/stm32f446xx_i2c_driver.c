/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Jun 26, 2024
 * */


#include "stm32f446xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_START);

}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SlaveAddr is Slave address + r/nw bit=0
	pI2Cx->DR = SlaveAddr;




}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;




}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummy_read;
		//check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//device is in master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize  == 1)
				{
					//first disable the ack
					I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

					//clear the ADDR flag ( read SR1 , read SR2)
					dummy_read = pI2CHandle->pI2Cx->SR1;
					dummy_read = pI2CHandle->pI2Cx->SR2;
					(void)dummy_read;
				}

			}
			else
			{
				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;

			}

		}
		else
		{
			//device is in slave mode
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
}


static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){


		pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

	}


void RCC_GetPLLOutputClock(void){


	return ;
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){


	if(EnorDi == I2C_ACK_ENABLE){


		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);



	}else{
		//disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

	}





}




uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};


uint16_t APB1_PreScaler[4] = {2,4,8,16};



uint32_t RCC_GetPCLK1Value(void){

	uint32_t pclk1,SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc= (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0){
		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	} else if(clksrc == 2){
//		SystemClk == RCC_GetPLLOutputClock();
	}

	//ahb1

	temp = ((RCC->CFGR >> 4) &0xF);

	if(temp<8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp-8];
	}

    //	apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp<4){
		apb1p = 1;
	}else{
		apb1p = APB1_PreScaler[temp-4];
	}


    pclk1 = (SystemClk/ahbp)/apb1p;
	return pclk1;
}






void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){

			pI2Cx->CR1 |= (1<<I2C_CR1_PE);

		}else{

			pI2Cx->CR1 &= ~(1<<0);

		}


}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
				if(pI2Cx==I2C1){
					I2C1_PCLK_EN();
				}else if(pI2Cx == I2C2){
					I2C2_PCLK_EN();
				}else if(pI2Cx == I2C3){
				I2C3_PCLK_EN();
				}



			}else{
				if(pI2Cx==I2C1){
							I2C1_PCLK_DI();
						}else if(pI2Cx == I2C2){
							I2C2_PCLK_DI();
						}else if(pI2Cx == I2C3){
							I2C3_PCLK_DI();
						}


			}


}






void I2C_Init(I2C_Handle_t *pI2CHandle){

	/*
	 * configuring CR1
	 */

	uint32_t tempreg=0;
    //ack control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
    pI2CHandle->pI2Cx->CR1 = tempreg;
    /*
     * Configuring CR2
     */
    //FREQ field

    tempreg=0;
    tempreg|=RCC_GetPCLK1Value()/1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

    //programming the device own address
    tempreg|=pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1<<14);
    pI2CHandle->pI2Cx->OAR1=tempreg;


    /*
     * Configuring CCR
     */

    //CCR Calculations

    uint16_t ccr_value = 0;
    tempreg=0;

    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){

    	//mode is standard mode
    	ccr_value = (RCC_GetPCLK1Value() / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	tempreg |= (ccr_value & 0xFFF);
    }else {
    	//mode is fast mode
    	tempreg |= (1<<15);
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
    	if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
    		ccr_value = (RCC_GetPCLK1Value() / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	}else{
    		ccr_value = (RCC_GetPCLK1Value() / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
    	}
    	tempreg |= (ccr_value & 0xFFF);
    }




   pI2CHandle->pI2Cx->CCR = tempreg;


}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName){

	if(pI2Cx->SR1 & FlagName){
	return FLAG_RESET;}
	return FLAG_SET;
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){
	//1. generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//Note: until SB is cleared SCL will be stretched (pulled to low)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/rw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);


	//4. confirm that address phase is completed by checking ADDR flag in the SR1

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//Note: until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6. Send the data until Len becomes 0

	while(Len>0){

		//first check whether TXE is set or not(data register empty or not):

		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;


	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//when BTF=1 SCL will be stretched (Pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));




	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//NOTE: generating STOP, automatically clears the BTF
	if(Sr==I2C_DISABLE_SR){
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}






}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){

	//1. generate START condition

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//Note: Until SB is cleared SCL will be stretched (Pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. send the address of the slave with the r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//procedure to read only 1 byte from the slave:

	if(Len ==1){


		//disable the ACKing:

		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);


		//wait until RXNE becomes 1

		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


		//generate stop condition:
		if(Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);




		//read data in to buffer:
		*pRxbuffer = pI2CHandle->pI2Cx->DR;





	}


	//procedure to read data from slave when Len >1

		if(Len>1){

			//clear the ADDR flag
			I2C_ClearADDRFlag(pI2CHandle);

			//read the data until Len becomes zero

			for (uint32_t i =Len; i > 0; i--){

				//wait until RXNE becomes 1
				while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


				if(i == 2)/*if last 2 bytes are remaining*/{
					//disable ACKing
					I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


					//generate STOP Condition
					if(Sr == I2C_DISABLE_SR){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					}

				}


				//read the data from data register in to buffer
				*pRxbuffer = pI2CHandle->pI2Cx->DR;


				//increment the buffer address
				pRxbuffer++;


			}



		}

		//re enable ACKing
		if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
		}





}


void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

}






/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState =I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition

		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);


		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);


		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}






uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition


		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);



		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


	}

	return busystate;

}












void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx==I2C1){
						I2C1_REG_RESET();
					}else if(pI2Cx == I2C2){
						I2C2_REG_RESET();
					}else if(pI2Cx == I2C3){
						I2C3_REG_RESET();
					}

}

}


