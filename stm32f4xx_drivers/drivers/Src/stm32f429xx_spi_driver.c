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
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
	// 2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		// bidi mode should be cleared
		tempreg &= ~(1 << 15);
	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		// bidi mode should be set
		tempreg |= (1 << 15);
	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// bidi mode should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY bit must be set
		tempreg |= (1 << 10);
	}
	// 3. configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;
	// 4. configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;
	// 5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;
	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	// Write CR1
	pSPIHandle->pSPIx->CR1 = tempreg;

	tempreg = 0;
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
