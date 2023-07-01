#include "user_GPIO.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void GPIO_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;		                        //Меандр 1 кГц
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
  GPIO_Init(GPIOB, &GPIO_InitStructure);
         
}



