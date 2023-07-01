#include <stdint.h>

#define M_PWM_OFF GPIO_SetBits(GPIOA,GPIO_Pin_6)                                //Управление семистором
#define M_PWM_ON GPIO_ResetBits(GPIOA,GPIO_Pin_6)

#define LED1_OFF GPIO_SetBits(GPIOE,GPIO_Pin_0)                                  //Управление светодиодом
#define LED1_ON GPIO_ResetBits(GPIOE,GPIO_Pin_0)
#define LED1_TOOGLE GPIOE->ODR ^= GPIO_Pin_0;

#define LED2_OFF GPIO_SetBits(GPIOE,GPIO_Pin_5)                                  //Управление светодиодом
#define LED2_ON GPIO_ResetBits(GPIOE,GPIO_Pin_5)

#define SHAR_STOP GPIO_SetBits(GPIOE,GPIO_Pin_7)                                //Стопор шара
#define SHAR_START GPIO_ResetBits(GPIOE,GPIO_Pin_7)

#define SHAR_OPEN GPIO_SetBits(GPIOD,GPIO_Pin_0)                                //ШАР
#define SHAR_CLOSE GPIO_ResetBits(GPIOD,GPIO_Pin_0)

#define VALVE1_OPEN GPIO_SetBits(GPIOD,GPIO_Pin_1)                              //ЭМ1
#define VALVE1_CLOSE GPIO_ResetBits(GPIOD,GPIO_Pin_1)

#define VALVE2_OPEN GPIO_SetBits(GPIOD,GPIO_Pin_2)                              //ЭМ2
#define VALVE2_CLOSE GPIO_ResetBits(GPIOD,GPIO_Pin_2)

#define POMP_ON GPIO_SetBits(GPIOD,GPIO_Pin_3)                                  //Компрессор1
#define POMP_OFF GPIO_ResetBits(GPIOD,GPIO_Pin_3)

#define BIGVALVE_OPEN GPIO_ResetBits(GPIOD,GPIO_Pin_4)                              //Большой клапан для LPG
#define BIGVALVE_CLOSE GPIO_SetBits(GPIOD,GPIO_Pin_4)

#define M_ON GPIO_SetBits(GPIOD,GPIO_Pin_5)                                     //Реле вкл мотора
#define M_OFF GPIO_ResetBits(GPIOD,GPIO_Pin_5)

#define IS_VALVE_IN_LPG GPIO_ReadInputDataBit (GPIOD, GPIO_Pin_14)              //Джампер 1
#define IS_VAKUUM GPIO_ReadInputDataBit (GPIOD, GPIO_Pin_15)                    //Джампер 2

void USART_IRQProcessFunc(uint8_t);
void nextionEvent(void);
void Nextion_SetValue_Number(char *ValueName, uint32_t Value);
void Nextion_SetValue_String(char *ValueName, char *Value);
uint8_t getFLAG_END_LINE(void);
uint8_t getJP();
uint8_t getMode();
uint8_t getMotorPower();
uint8_t getWaveform();
int8_t getValvePower();
void setValvePower(int8_t diff);
void setSharPos(uint8_t pos);
void sendAck();
void sendAckExit();
uint32_t getErrorTick();
void setErrorTick(uint32_t tick);
void switchOffAll();