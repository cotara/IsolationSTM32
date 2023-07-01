#include <stdint.h>

#define RX_BUF_SIZE 100
#define TX_BUF_SIZE 100

#define TX485EN GPIO_SetBits(GPIOA, GPIO_Pin_5)    
#define RX485EN GPIO_ResetBits(GPIOA, GPIO_Pin_5)   


void usart_init(void);
void clear_RXBuffer(void);
void setRxi(uint16_t i);
uint8_t toBuf(uint8_t n);
uint8_t toBuf(uint8_t n);
uint8_t fromBuf( uint16_t i);
void USART1_put_char(uint8_t c);
void USART1_put_string(unsigned char *string, uint32_t l);
