#ifndef INC_STM32F429XX_USART_DRIVER_H_
#define INC_STM32F429XX_USART_DRIVER_H_

#include "stm32f429xx.h"


/*
 * Configuration structure for USART peripheral
 */
typedef struct
{
	uint8_t USART_Mode;				// Transmit and/or receive
	uint32_t USART_Baud;			// Up to 3Mbps
	uint8_t USART_NoOfStopBits;		// Number of stop bits
	uint8_t USART_WordLength;		// 8 or 9 bits
	uint8_t USART_ParityControl;	// None/even/odd
	uint8_t USART_HWFlowControl;	// None or CTS and/or RTS. NOTE: not available for UART4 & UART5
} USART_Config_t;

/*
 * Handle structure for USART peripheral
 */
typedef struct
{
	USART_RegDef_t 	*pUSARTx;	// Base address of USARTx
	USART_Config_t	USARTConfig;
} USART_Handle_t;


/*
 * @USART_Mode
 */
#define USART_MODE_ONLY_TX	0
#define USART_MODE_ONLY_RX	1
#define USART_MODE_TXRX		2

/*
 * @USART_Baud
 * Standard options
 */
#define USART_STD_BAUD_1200		1200
#define USART_STD_BAUD_2400		2400
#define USART_STD_BAUD_9600		9600
#define USART_STD_BAUD_19200	19200
#define USART_STD_BAUD_38400	38400
#define USART_STD_BAUD_57600	57600
#define USART_STD_BAUD_115200	115200
#define USART_STD_BAUD_230400	230400
#define USART_STD_BAUD_460800	460800
#define USART_STD_BAUD_921600	921600
#define USART_STD_BAUD_2M		2000000
#define USART_STD_BAUD_3M		3000000

/*
 * @USART_ParityControl
 */
#define USART_PARITY_DISABLE	0
#define USART_PARITY_EN_EVEN	1
#define USART_PARITY_EN_ODD		2

/*
 * @USART_WordLength
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

/*
 * @USART_NoOfStopBits
 */
#define USART_STOPBITS_1		0	// 1 stop bit
#define USART_STOPBITS_0_5		1	// 0.5 stop bits
#define USART_STOPBITS_2		2	// 2 stop bits
#define USART_STOPBITS_1_5		3	// 1.5 stop bits

/*
 * @USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


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
