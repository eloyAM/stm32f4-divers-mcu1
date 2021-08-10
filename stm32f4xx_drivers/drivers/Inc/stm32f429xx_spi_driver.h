#ifndef INC_STM32F429XX_SPI_DRIVER_H_
#define INC_STM32F429XX_SPI_DRIVER_H_

/*
 * Configuration structure for SPI peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_PHA;
	uint8_t SPI_SSM;
} SPI_Config_t;


/*
 * Handle structure for SPI peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;	// Base address of SPIx
	SPI_Config_t SPIConfig;
};


#endif /* INC_STM32F429XX_SPI_DRIVER_H_ */
