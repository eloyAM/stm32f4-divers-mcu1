#include "stm32f429xx_usart_driver.h"


/*
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given USART
 *
 * @param[in]	- base address of the USART peripheral
 * @param[in]	- ENABLE or DISABLE macros
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	// Enable
	if (EnOrDi == ENABLE)
	{
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_EN();
		}
	}
	// Disable
	else
	{
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3) {
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4) {
			UART4_PCLK_DI();
		}
	}
}

/*
 * USART De-initialization (reset)
 */
/*
 * @fn			- USART_DeInit
 *
 * @brief		-
 *
 * @param[in]	- base address of the USART peripheral
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if (pUSARTx == USART1) {
		USART1_REG_RESET();
	} else if (pUSARTx == USART2) {
		USART2_REG_RESET();
	} else if (pUSARTx == USART3) {
		USART3_REG_RESET();
	} else if (pUSARTx == UART4) {
		UART4_REG_RESET();
	}
}

/*
 * @fn			- USART_Init
 *
 * @brief		- Initializes USART peripheral
 *
 * @param[in]	- pointer to USART handler
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_Init(USART_Handle_t *pHandle)
{
	uint32_t tempreg = 0;

	// Enable peripheral clock
	USART_PeriClockControl(pHandle->pUSARTx, ENABLE);

	/********************
	 * CR1 configuration
	 *******************/

	// Enable Tx and/or Rx according to the USART mode
	if (pHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_RX) {
		tempreg |= (1 << USART_CR1_RE);
	} else if (pHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_TX) {
		tempreg |= (1 << USART_CR1_TE);
	} else if (pHandle->USARTConfig.USART_Mode == USART_MODE_TXRX) {
		tempreg |= ( (1 << USART_CR1_RE) | (1 << USART_CR1_TE) );
	}

	// Choose word length
	tempreg |= (pHandle->USARTConfig.USART_WordLength << USART_CR1_M);

	// Choose parity control
	if (pHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_EVEN) {
		tempreg |= (1 << USART_CR1_PCE); // Enable parity control
		// No need to reset USART_CR1_PS, default value (0) means even parity
	} else if (pHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_ODD) {
		tempreg |= (1 << USART_CR1_PCE); // Enable parity control
		tempreg |= (1 << USART_CR1_PS); // Select odd parity
	}

	// Save CR1
	pHandle->pUSARTx->CR1 = tempreg;

	/*******************
	 * CR2 configuration
	 *******************/

	tempreg = 0;

	// Choose number of stop bits
	tempreg |= (pHandle->USARTConfig.USART_NoOfStopBits << USART_CR2_STOP);

	// Save CR2
	pHandle->pUSARTx->CR2 = tempreg;

	/********************
	 * CR3 configuration
	 *******************/

	tempreg = 0;

	// Choose hardware flow control
	if (pHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		tempreg |= (1 << USART_CR3_CTSE);
	} else if (pHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
		tempreg |= (1 << USART_CR3_RTSE);
	} else if (pHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		tempreg |= ( (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE) );
	}

	// Save CR3
	pHandle->pUSARTx->CR3 = tempreg;
}














