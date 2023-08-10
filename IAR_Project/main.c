#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"
#include "user_USART.h"
#include "user_GPIO.h"
#include "user_TIMER.h"
#include "user_ADC.h"
#include "modbus.h"

#include "main.h"
#include <stdlib.h>

int32_t actualVoltage=0;
int32_t actualCurrent=0;
int16_t ustVoltage=0;
double regKoefCur = 0.6;
double regKoef = 0.06;
double CCR, minCCR,maxCCR = 9000000/PWM_FREQ;
int32_t maxCurrent=250;                                                         //25 mkA*10
double err;                                                                     //Рассогласование
uint8_t defectDetected = 0;                                                     
int32_t defectPos=0;
extern int32_t position;
extern uint8_t errorCounter;

uint32_t counter__ = 0;
uint8_t adress=0;

int main()
{    
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency /1000);
  
  GPIO_init();
  
  for (int i=0;i< 3;i++){
    PROBOY_LED_ON;
    Delay(100);
    HV_LED_ON;
    Delay(100);
    PROBOY_LED_OFF;
    Delay(100);
    HV_LED_OFF;
    Delay(100);
  }
  
  minCCR = maxCCR*0.5;
  CCR = maxCCR; 
  
  tim3_pwm_init(PWM_FREQ);
 
  Delay(20);
  
  adcVoltage_init();
  adcCurrent_init();
  
  tim4_init(); 
   
  adress=gerAdress();
  modbusInit(adress);
  tim2_init();
  usart_init();
  tim5_init();                                                                  //Контроль обрыва связи
  
  while(1){
      ustVoltage = (int16_t)getReg(UST_VOLTAGE_REG);
    
  }
}

void updateVoltage(double val){
    if(CCR == maxCCR)                                                           //Шим выключен
      actualVoltage = 0;
    else
      actualVoltage =(int32_t)(val*1.5584 + 6.5);
    if(errorCounter==0) {                                                        //Штатный режим(нет ошибки связи), без мигания
      if(getReg(HIGH_VOL_REG) || actualVoltage>0)
        HV_LED_ON;
      else
        HV_LED_OFF;
    }
    setReg(actualVoltage,ACTUAL_VOLTAGE_REG);
    //setReg(val,ACTUAL_VOLTAGE_REG);                                           //Чтобы выводить тугрики для калибровки
}

void updateCurrent(double val){
    if(val<637){                                                                //637 соответствует 4.37 мкА, что соответствует 437 В. То есть до 437В используем одну формулу, а потом другую
      actualCurrent =(int32_t)(0.0768*val-4.5);
      if(actualCurrent<0)       actualCurrent=0;
    }
    else
      actualCurrent =(int32_t)(0.0665*val+1.5);
   
    setReg(actualCurrent,ACTUAL_CURR_REG);
    //setReg(val,ACTUAL_CURR_REG);                                             //Чтобы выводить тугрики для калибровки
    
    if( actualCurrent>getReg(UST_CURR_REG) ){
      PROBOY_LED_ON;
      if(!defectDetected){
        defectDetected=1;
        defectPos=position;
        setReg(getReg(DEFECTS_REG)+1, DEFECTS_REG);
      }
      else{
        if(abs(position-defectPos)>= getReg(MAX_DEFECT_LENGTH)){
          defectPos=position;
          setReg(getReg(DEFECTS_REG)+1, DEFECTS_REG);
        }
      }
    }
    else{ 
      defectDetected=0;
      PROBOY_LED_OFF;
    }
}



uint8_t gerAdress(){
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;		                        //?????? 1 ???
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  uint8_t pin0, pin1, pin2, pin3;
  
  pin0 = GPIO_ReadInputDataBit (GPIOD, GPIO_Pin_0);
  pin1 = GPIO_ReadInputDataBit (GPIOD, GPIO_Pin_1);
  pin2 = GPIO_ReadInputDataBit (GPIOD, GPIO_Pin_2);
  pin3 = GPIO_ReadInputDataBit (GPIOD, GPIO_Pin_3);
  
  return 0x0f-(pin0*8 + pin1*4 +  pin2*2 + pin3);
}

void regulatorAct(){
  if(ustVoltage==0)
    CCR=maxCCR;
  if(actualCurrent >= maxCurrent && ustVoltage>actualVoltage)                   //If there is an overcurrent and the voltage setting is higher than the real values
    err = regKoefCur*(maxCurrent-actualCurrent);
  else
    err = regKoef*(ustVoltage-actualVoltage);
  
  CCR = CCR - err;
       
  if(CCR<minCCR) 
    CCR = minCCR;
  else if(CCR > maxCCR) 
    CCR = maxCCR;

  setCcr3Tim((uint16_t)CCR);
}