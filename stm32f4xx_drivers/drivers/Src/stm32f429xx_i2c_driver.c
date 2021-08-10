#include "stm32f429_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_PreScaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr <<= 1;
	SlaveAddr &= ~(1);	// Slave addr is slave addr + r/nw bit=0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr <<= 1;
	SlaveAddr |= 1;	// Slave addr is slave addr + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


// Not implemented, we are not using PLL clock
uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}

uint32_t RCC_GetPCKL1Value(void)
{
	uint32_t pckl1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;
	clksrc = ((RCC->CFGR >> 2) & 0x3);
	switch (clksrc) {
		case 0: SystemClk = 16000000; break;
		case 1: SystemClk =  8000000; break;
		case 2: SystemClk = RCC_GetPLLOutputClock(); break;
	}
	// For AHB
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp-8];
	}
	// For APB1
	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp < 8) {
		apb1p = 1;
	} else {
		apb1p = APB_PreScaler[temp-4];
	}

	pckl1 = (SystemClk / ahbp) / apb1p;

	return pckl1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	// ACK control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;
	// Configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCKL1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0xF3);
	// Program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;
	// CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode
		ccr_value = (RCC_GetPCKL1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
	} else {
		// Fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCKL1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccr_value = (RCC_GetPCKL1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
	}
	tempreg |= (ccr_value & 0xFFF);
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode
		tempreg = (RCC_GetPCKL1Value() / 1000000U) + 1;
	} else {
		// Fast mode
		tempreg = ((RCC_GetPCKL1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F); // TRISE takes last 6 bits (0x3F = 11 1111)
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that the START generation is completed by checking the SB flag
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) {
		// wait
	}
	// 3. Send the address of the slave with R/NW bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
	// 4. Confirm that the address phase is completed by checking the ADDR flag
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) {
		// wait
	}
	// 5. Clear the ADDR flag according to its software sequence
	// Note: until ADDR is cleared, SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
	// 6. Send data until Len becomes 0
	while (Len > 0) {
		while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)) {
			// wait
		}
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	// 7. When Len becomes 0, wait for TXE=2 and BFT=1 before generating the STOP condition
	// Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1, SCL will be stretched (pulled to LOW)
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)) {
		// wait
	}
	while ( !I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)) {
		// wait
	}
	// 8. Generate STOP condition and master need not to wait for the completion of stop condition
	// Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	// 2. Confirm that the START generation is completed by checking the SB flag
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) {
		// wait
	}
	// 3. Send the address of the slave with R/NW bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	// 4. Confirm that the address phase is completed by checking the ADDR flag
	while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) {
		// wait
	}
	// Procedure to read only 1 byte from slave
	if (Len == 1) {
		// Disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		// generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// wait until RXNE becomes 1
		while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)) {
			// wait
		}
		// read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	// Procedure to read data from slave when Len > 1
	if (Len > 1) {
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);
		// Read the data until Len becomes zero
		for (uint32_t i = 0; i > 0; --i) {
			// wait until RXNE becomes 1
			while ( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)) {
				// wait
			}
			if (i == 2) {
				// Clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			// read data
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			// increment buffer addr
			pRxBuffer++;
		}
	}
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE) {
		// Re enable ACKing
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == I2C_ACK_ENABLE) {
		// Enable ACK
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else {
		// Disable ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


