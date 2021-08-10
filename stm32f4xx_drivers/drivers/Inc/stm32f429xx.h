#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_

#include <stdio.h>

#define __vo volatile

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define SRAM 				SRAM1_BASEADDR
#define ROM					0x1FFF0000U // System memory


/*
 * AHBx and PBx Bus Peripheral base addresses
 */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U



/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800U)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00U)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000U)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400U)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800U)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00U)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000U)
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE + 0x2400U)
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE + 0x2800U)

#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800U)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00U)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00U)

#define USART2_BASE			(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASE			(APB1PERIPH_BASE + 0x4800U)

#define UART4_BASE			(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASE			(APB1PERIPH_BASE + 0x5000U)
#define UART7_BASE			(APB1PERIPH_BASE + 0x7C00U)
#define UART8_BASE			(APB1PERIPH_BASE + 0x7800U)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00U)

#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000U)
#define SPI4_BASEADDR		(APB2PERIPH_BASE + 0x3400U)
#define SPI5_BASEADDR		(APB2PERIPH_BASE + 0x5000U)
#define SPI6_BASEADDR		(APB2PERIPH_BASE + 0x5400U)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800U)

#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000U)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400U)



typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];		// AFR[0]: GPIO alternate function low register; AFR[1]: GPIO alternate function high register
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t RC;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
}RCC_RegDef_t;


typedef struct
{
	volatile uint32_t	CR1;
	volatile uint32_t	CR2;
	volatile uint32_t	OAR1;
	volatile uint32_t	OAR2;
	volatile uint32_t	DR;
	volatile uint32_t	SR1;
	volatile uint32_t	SR2;
	volatile uint32_t	CCR;
	volatile uint32_t	TRISE;
	volatile uint32_t	FLTR;
}I2C_RegDef_t;


typedef struct
{
	volatile uint32_t	CR1;		// Control 1
	volatile uint32_t	CR2;		// Control 2
	volatile uint32_t	SR;			// Status
	volatile uint32_t	DR;			// Data
	volatile uint32_t	CRCPR;		// CRC polynomial
	volatile uint32_t	RXCRCR;		// RX CRC
	volatile uint32_t	TXCRCR;		// TX CRC
	volatile uint32_t	I2SCFGR;	// I2S config
	volatile uint32_t	I2SSPR;		// I2S prescaler
} SPI_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)

#define I2C1		((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*) I2C3_BASEADDR)

#define SPI1		((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*) SPI4_BASEADDR)


/*TODO
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))


/*TODO
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))


/*
 * Clock Enable Macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))


/*TODO
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))


/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 *TODO
 * IRQ (Interrupt Request) Numbers of STM32F429x MCU
 */


// Some generic macros

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET



/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_SWRST		15
/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
//#define I2C_CR2_DMAEN		11
//#define I2C_CR2_LAST		12
/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14
/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR1_MSL			0
#define I2C_SR1_BUSY		1
#define I2C_SR1_TRA			2
#define I2C_SR1_GENCALL		4
#define I2C_SR1_DUALF		7
/*
 * Bit definition I2C_CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15


/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSB_FIRST 	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRC_NEXT	12
#define SPI_CR1_CRC_EN		13
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15
/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN 	0
#define SPI_CR2_TXDMAEN 	1
#define SPI_CR2_SSOE 		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE	 	5
#define SPI_CR2_RXNEIE	 	6
#define SPI_CR2_TXEIE		7
/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE 		0
#define SPI_SR_TXE 			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRC_ERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR	 		6
#define SPI_SR_BSY	 		7
#define SPI_SR_FRE	 		8


#endif /* INC_STM32F429XX_H_ */
