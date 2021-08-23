#include "ds1307.h"
#include <stdint.h>
#include <string.h>


I2C_Handle_t g_ds1307_I2CHandle;


// Prototypes
static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);


// Returns CH (1: Ok, 0: failed)
uint8_t ds1307_init(void)
{
	// 1. Init the I2c pins
	ds1307_i2c_pin_config();

	// 2. Init the I2C peripheral
	ds1307_i2c_config();

	// 3. Enable the I2C peripheral
	I2C_PeriClockControl(DS1307_I2C, ENABLE);

	// 4. Make cock halt = 0
	ds1307_write(0x00, DS1307_ADDR_SEC);

	// 5. Read back clock halt bit
	uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);

	return ((clock_state >> 7) & 0x01);
}



/*
 * Helper functions
 */

static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	// I2C1_SCL -> PB6
	// I2C1_SDA -> PB7

	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_scl);
}


static void ds1307_i2c_config(void)
{
	g_ds1307_I2CHandle.pI2Cx = DS1307_I2C;
	g_ds1307_I2CHandle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	g_ds1307_I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
	I2C_Init(&g_ds1307_I2CHandle);
}


static void ds1307_write(uint8_t value, uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;
	I2C_MasterSendData(&g_ds1307_I2CHandle, tx, 2, DS1307_I2C_ADDRESS);
}


static uint8_t ds1307_read(uint8_t reg_addr)
{
	I2C_MasterSendData(&g_ds1307_I2CHandle, &reg_addr, 1, DS1307_I2C_ADDRESS);
	uint8_t data;
	I2C_MasterReceiveData(&g_ds1307_I2CHandle, &data, 1, DS1307_I2C_ADDRESS);

	return data;
}
