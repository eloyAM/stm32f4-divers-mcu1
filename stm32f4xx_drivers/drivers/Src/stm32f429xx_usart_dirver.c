#include "stm32f429xx_usart_driver.h"


/*
 * @fn			- USART_Init
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
