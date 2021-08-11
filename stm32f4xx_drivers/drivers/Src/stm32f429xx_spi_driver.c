#include "stm32f429xx_spi_driver.h"


/*
 * @fn			- SPI_Init
 *
 * @brief		- This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]	- base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	// Enable
	if (EnOrDi == ENABLE)
	{
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}
	}
	// Disable
	else
	{
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		} else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/*
 * SPI Initialization
 */
/*
 * @fn			- SPI_Init
 *
 * @brief		-
 *
 * @param[in]	- handler of the SPI peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	// previously -> enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// First, let's configure the SPI_CR1 register
	// 1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	// 2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDI_MODE);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDI_MODE);
		// RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RX_ONLY);
	}
	// 3. configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	// 4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	// 5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// Write CR1
	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*
 * SPI De-initialization (reset)
 */
/*
 * @fn			- SPI_DeInit
 *
 * @brief		-
 *
 * @param[in]	- base address of the SPI peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

/*
 * SPI send data
 */
/*
 * @fn			- SPI_SendData
 *
 * @brief		-
 *
 * @param[in]	- base address of the SPI peripheral
 * @param[in]	- pointer to the buffer with the data to send
 * @param[in]	- number of bytes to send
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// 1. wait until TX buffer is empty
		while (!(pSPIx->SR & (1 << SPI_SR_TXE))) {}
		// 2. check the DFF (data frame format) 8/16 bits
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bits DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*) pTxBuffer++;
		} else {
			// 8 bits DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*
 * SPI Peripheral control
 */
/*
 * @fn			- SPI_PeripheralControl
 *
 * @brief		-
 *
 * @param[in]	- base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE
 * @return		- none
 *
 * @Note		- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		// wait until peripheral is not busy
		while (! (SPI2->SR & (1 << SPI_SR_BSY)) ) {};
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * SPI SSI control
 */
/*
 * @fn			- SPI_SSIControl
 *
 * @brief		-
 *
 * @param[in]	- base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*
 * SPI SSOE control
 */
/*
 * @fn			- SPI_SSOEControl
 *
 * @brief		-
 *
 * @param[in]	- base address of the SPI peripheral
 * @param[in]	- ENABLE or DISABLE
 * @return		- none
 *
 * @Note		- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*
 * SPI receive data
 */
/*
 * @fn			- SPI_ReceiveData
 *
 * @brief		-
 *
 * @param[in]	- base address of the SPI peripheral
 * @param[in]	- pointer to the buffer to store the data
 * @param[in]	- number of bytes to send
 * @return		- none
 *
 * @Note		- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// 1. wait until RX buffer is empty
		while (!(pSPIx->SR & (1 << SPI_SR_RXNE))) {}
		// 2. check the DFF (data frame format) 8/16 bits
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16 bits DFF
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2;
			(uint16_t*) pRxBuffer++;
		} else {
			// 8 bits DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/*
 * SPI send data (interrupt based)
 */
/*
 * @fn			- SPI_SendDataIT
 *
 * @brief		-
 *
 * @param[in]	- pointer to SPI handler
 * @param[in]	- pointer to the buffer with the data to send
 * @param[in]	- number of bytes to send
 * @return		- SPI Tx state
 *
 * @Note		- none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer addr and len info in some global vars
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = Len;
		// 2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pHandle->TxState = SPI_BUSY_IN_TX;
		// 3. Enable the TXEIE control bit to get interrupt whenever the TXE is set in SR
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		// 4. Data transmission will be handled by the ISR code
	}

	return state;
}

/*
 * SPI receive data (interrupt based)
 */
/*
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		-
 *
 * @param[in]	- pointer to SPI handler
 * @param[in]	- pointer to the buffer to store the data
 * @param[in]	- number of bytes to send
 * @return		- SPI Rx state
 *
 * @Note		- none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx buffer addr and len info in some global vars
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxLen = Len;
		// 2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pHandle->RxState = SPI_BUSY_IN_RX;
		// 3. Enable the RXEIE control bit to get interrupt whenever the RXE is set in SR
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		// 4. Data transmission will be handled by the ISR code
	}

	return state;
}
