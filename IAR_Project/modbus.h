#include <stdint.h>
#include "main.h"

#define REG_SIZE 23

#define TYPE_MODEL_REG 0x00
#define ACTUAL_VOLTAGE_REG 0x01
#define ACTUAL_CURR_REG 0x02
#define DEFECTS_REG 0x03
#define HIGH_VOL_REG 0x04
#define UST_VOLTAGE_REG 0x05
#define UST_CURR_REG 0x06
#define UST_LEN_REG 0x07
#define MAX_DEFECT_LENGTH 0x08
#define STOP_LINE_FLAG 0x09
#define ET_VOL1        0x0A
#define ET_VOL2        0x0B
#define ET_CUR1        0x0C
#define ET_CUR2        0x0D
#define EEPROM_SET     0x0E
#define K_VOL_HIGH     0x0F
#define K_VOL_LOW      0x10
#define B_VOL_HIGH     0x11
#define B_VOL_LOW      0x12
#define K_CUR_HIGH     0x13
#define K_CUR_LOW      0x14
#define B_CUR_HIGH     0x15
#define B_CUR_LOW      0x16

void modbusInit(uint8_t add);
uint8_t setReg(uint16_t val, uint8_t index);
uint16_t getReg(uint8_t index);
uint8_t toBuf(uint8_t n);
uint8_t modbusProcess();
uint8_t modbus3();
uint8_t modbus6();
uint8_t modbus16();
unsigned short crc16(unsigned char *data_p, unsigned short len);