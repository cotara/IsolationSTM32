#include <stdint.h>

#define HV_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_0)                                //Управление семистором
#define HV_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_0)

#define PROBOY_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_1)                                //Управление семистором
#define PROBOY_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_1)

void GPIO_init(void);