#include <string.h>
#include "stm32f429xx.h"

char msg[] = "UART TX testing...\r\n";

USART_Handle_t usart2_handle;

static void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;

	usart2_handle.USARTConfig.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USARTConfig.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USARTConfig.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USARTConfig.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USARTConfig.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USARTConfig.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart2_handle);
}

static void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;

	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	// USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpios);

	// USART RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpios);
}

int main_015uart()
{
	USART2_GPIOInit();

	USART2_Init();

	USART_PeriClockControl(usart2_handle.pUSARTx, ENABLE);

	USART_SendData(&usart2_handle, (uint8_t*)msg, strlen(msg));

	while(1);

	return 0;
}

