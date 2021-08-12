#ifndef INC_STM32F429XX_USART_DRIVER_H_
#define INC_STM32F429XX_USART_DRIVER_H_

#include "stm32f429xx.h"


/*
 * Configuration structure for USART peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WortLen;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
} USART_Config_t;

/*
 * Handle structure for USART peripheral
 */
typedef struct
{
	USART_RegDef_t 	*pUSARTx;	// Base address of USARTx
	USART_Config_t	USARTConfig;
} USART_Handle_t;


#endif /* INC_STM32F429XX_USART_DRIVER_H_ */
