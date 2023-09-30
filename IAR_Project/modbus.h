#include <stdint.h>

#define REG_SIZE 10

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

void modbusInit(uint8_t add);
uint8_t setReg(uint16_t val, uint8_t index);
uint16_t getReg(uint8_t index);
uint8_t toBuf(uint8_t n);
uint8_t modbusProcess();
uint8_t modbus3();
uint8_t modbus6();
uint8_t modbus16();
unsigned short crc16(unsigned char *data_p, unsigned short len);