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
	// First, let's configure the SPI_CR1 register
	uint32_t tempreg = 0;
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

