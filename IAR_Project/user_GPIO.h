#include <stdint.h>

#define START_LINE GPIO_SetBits(GPIOB,GPIO_Pin_13)                                //stop line rele
#define  STOP_LINE GPIO_ResetBits(GPIOB,GPIO_Pin_13)

#define HV_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_0)                                //Управление семистором
#define HV_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define HV_LED_TOOGLE GPIOB->ODR ^= GPIO_Pin_0

#define PROBOY_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_1)                                //Управление семистором
#define PROBOY_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_1)

void GPIO_init(void);