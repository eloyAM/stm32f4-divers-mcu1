#ifndef INC_STM32F429XX_RCC_DRIVER_H_
#define INC_STM32F429XX_RCC_DRIVER_H_

#include "stm32f429xx.h"

uint32_t RCC_GetPCKL1Value(void);
uint32_t RCC_GetPCKL2Value(void);

uint32_t RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F429XX_RCC_DRIVER_H_ */
