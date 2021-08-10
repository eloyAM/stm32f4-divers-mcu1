#ifndef INC_STM32F429XX_SPI_DRIVER_H_
#define INC_STM32F429XX_SPI_DRIVER_H_

#include "stm32f429xx.h"

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
} SPI_Handle_t;


/*******************************
 * APIs supported by this driver
 ******************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(SPI_Handle_t *pHandle);

#endif /* INC_STM32F429XX_SPI_DRIVER_H_ */
