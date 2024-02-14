#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"

#define EEPROM_HW_ADDRESS      0xA0   /* E0 = E1 = E2 = 0 */
#define I2C_EE             I2C1//interface number

uint8_t I2C_EE_ByteRead( uint16_t ReadAddr);
void I2C_EE_ByteWrite(uint8_t val, uint16_t WriteAddr);
void I2C_Configuration(void);