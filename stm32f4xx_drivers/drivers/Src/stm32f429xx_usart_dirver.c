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



/*
 * @fn			- USART_SendData
 *
 * @brief		- Sends data
 *
 * @param[in]	- base address of the USART peripheral
 * @param[in]	- pointer to buffer with the data to send
 * @param[in]	- number of bytes to send
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_SendData(USART_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pData;

	for (uint32_t i = 0; i < Len; i++)
	{
		// Wait until transmit data registry is empty
		while (! (pHandle->pUSARTx->SR & (1 << USART_SR_TXE))) {}

		// Choose between 8 and 9 bits word length
		if (pHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/*9 bit length word*/
			// Load the DR with 2 bytes masking the non-relevant bits
			pData = (uint16_t*) pTxBuffer;
			pHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);
			// Check parity configuration
			if (pHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity. The 9 bits will be useful data
				// 2 bytes are consumed
				pTxBuffer += 2;
			} else {
				// Parity. 8 bits of useful data, the 9th will be replaced by hardware
				// 1 byte is consumed
				pTxBuffer++;
			}
		}
		else
		{
			/*8 bit length word*/
			pHandle->pUSARTx->DR = *pTxBuffer;

			// No need to check the parity configuration
			// The hardware will automatically replace the 8th bit if needed
			pTxBuffer++;
		}
	}

	// Wait until transmission is set as completed
	while (! (pHandle->pUSARTx->SR & (1 << USART_SR_TC))) {}
}

/*
 * @fn			- USART_ReceiveData
 *
 * @brief		- Receives data
 *
 * @param[in]	- base address of the USART peripheral
 * @param[in]	- pointer to buffer where to store the data
 * @param[in]	- number of bytes to receive
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_ReceiveData(USART_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		// Wait until received data is ready to be read
		while (! (pHandle->pUSARTx->SR & (1 << USART_SR_RXNE))) {}

		// Choose between 8 or 9 bits word length
		if (pHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			/*9 BIT OF WORD LENGTH*/
			// Check parity configuration
			if (pHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity. The 9 bits will be useful data
				// 2 bytes are consumed
				*( (uint16_t*) pRxBuffer) = (pHandle->pUSARTx->DR & (uint16_t)0x01FF);
				pRxBuffer += 2;
			} else {
				// Parity. 8 bits of useful data, the 9th will be replaced by hardware
				// 1 byte is consumed
				*pRxBuffer = (pHandle->pUSARTx->DR & (uint8_t)0x01FF);
				pRxBuffer++;
			}
		}
		else
		{
			/*8 BITS OF WORD LENGTH*/
			// Check parity configuration
			if (pHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE) {
				// Normal read of 8 bits
				*pRxBuffer = pHandle->pUSARTx->DR;
			} else {
				// Discard the 8th bit (used as parity control)
				*pRxBuffer = (pHandle->pUSARTx->DR & (uint8_t)0x7F);
			}
			pRxBuffer++;
		}
	}
}

/*
 * @fn			- USART_SendDataIT
 *
 * @brief		- Sends data - interruption based
 *
 * @param[in]	- pointer to USART handler
 * @param[in]	- pointer to buffer with the data to send
 * @param[in]	- number of bytes to send
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t USART_SendDataIT(USART_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t TxState = pHandle->TxState;

	if (TxState != USART_BUSY_IN_TX)
	{
		// Save fields
		pHandle->TxLen = Len;
		pHandle->pTxBuffer = pTxBuffer;
		// Indicate state as busy transmitting
		pHandle->TxState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return TxState;
}


/*
 * @fn			- USART_ReceiveDataIT
 *
 * @brief		- Receives data - interruption based
 *
 * @param[in]	- pointer to USART handler
 * @param[in]	- pointer to buffer where to store the data
 * @param[in]	- number of bytes to receive
 *
 * @return		- none
 *
 * @Note		- none
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t RxState = pHandle->TxState;

	if (RxState != USART_BUSY_IN_RX)
	{
		// Save fields
		pHandle->RxLen = Len;
		pHandle->pRxBuffer = pRxBuffer;
		// Indicate state as busy transmitting
		pHandle->RxState = USART_BUSY_IN_RX;

		// Enable interrupt for RXNE
		pHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return RxState;
}

/*
 * @fn			- USART_SetBaudRate
 *
 * @brief		- Sets USART baud rate
 *
 * @param[in]	- base address of the USART peripheral
 * @param[in]	- baud rate (bps)
 *
 * @return		- none
 *
 * @Note		- none
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	// APB clock
	uint32_t PCLKx;

	uint32_t USARTdiv;

	// Mantissa and Fraction values
	uint32_t mantissaPart, fractionPart;

	uint32_t tempreg = 0;

	// Get the value of APB bus clock
	if (pUSARTx == USART1 || pUSARTx == USART6) {
		// USART1 & USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	} else {
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		// OVER8 = 1, Oversampling by 8 is used
		USARTdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		// Oversampling by 16
		USARTdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	// Calculate the Mantissa part
	mantissaPart = USARTdiv / 100;

	// Place the Mantissa part in appropriate bit position. refer USART_BRR
	tempreg |= (mantissaPart << 4);

	// Extracting fraction part
	fractionPart = (USARTdiv - mantissaPart * 100);

	// Calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
		// Oversampling by 8
		fractionPart = (((fractionPart *  8) + 50) / 100) & ((uint8_t)0x07);
	} else {
		fractionPart = (((fractionPart * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	// Place the Fraction part in appropriate bit position. refer USART_BRR
	tempreg |= fractionPart;

	// Copy the value of tempreg into BRR register
	pUSARTx->BRR = tempreg;
}



