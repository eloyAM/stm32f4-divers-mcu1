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
	uint8_t USART_WordLength;
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



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
void USART_SendDataIT(USART_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveDataIT(USART_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDI);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pHandle, uint8_t AppEv);


#endif /* INC_STM32F429XX_USART_DRIVER_H_ */
