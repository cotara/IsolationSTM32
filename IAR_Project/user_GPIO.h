#include <stdint.h>

#define STOP_LINE GPIO_SetBits(GPIOB,GPIO_Pin_13)                                //stop line rele
#define START_LINE  GPIO_ResetBits(GPIOB,GPIO_Pin_13)
//SIBKABEL
//#define HV_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_0)                                //���������� ����������
//#define HV_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_0)
//#define HV_LED_TOOGLE GPIOB->ODR ^= GPIO_Pin_0
//
//#define PROBOY_LED_OFF GPIO_SetBits(GPIOB,GPIO_Pin_1)                                //���������� ����������
//#define PROBOY_LED_ON GPIO_ResetBits(GPIOB,GPIO_Pin_1)

//LITE
#define HV_LED_OFF GPIO_SetBits(GPIOD,GPIO_Pin_4)                                //ӯ𠢫孨報嬨񲮰
#define HV_LED_ON GPIO_ResetBits(GPIOD,GPIO_Pin_4)
#define HV_LED_TOOGLE GPIOD->ODR ^= GPIO_Pin_4

#define PROBOY_LED_OFF GPIO_SetBits(GPIOD,GPIO_Pin_5)                                //ӯ𠢫孨報嬨񲮰
#define PROBOY_LED_ON GPIO_ResetBits(GPIOD,GPIO_Pin_5)
#define PROBOY_LED_TOOGLE GPIOD->ODR ^= GPIO_Pin_5
void GPIO_init(void);