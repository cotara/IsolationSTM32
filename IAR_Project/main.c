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
int32_t actualCurrent=0, maxDefectCurrent=0;
int16_t ustVoltage=0;
double regKoefCur = 0.6;
double regKoef = 0.06;
double CCR, minCCR,maxCCR = 9000000/PWM_FREQ;
int32_t maxCurrent=250;                                                         //25 mkA*10
double err;                                                                     //Рассогласование
uint8_t defectDetected = 0;                                                     
int32_t defectPos=0;
extern int32_t position;
extern int32_t speed;
extern uint8_t errorCounter;
extern uint8_t readFlag;  
uint32_t counter__ = 0;
uint8_t adress=0;
float k_vol=1.5, b_vol=0, k_cur=0.06, b_cur=0;
double ADC_ActualVoltage=0,ADC_VoltageEt1=0, ADC_VoltageEt2=0,ADC_ActualCurrent=0,ADC_CurrentEt1=0,ADC_CurrentEt2=0;

typedef union {
  uint16_t sh[2];
  float fl;
}flTosh_t;

flTosh_t m_flTosh;

//Различия в пинах индикации (user_GPIO.c и user_GPIO.h) 
int main()
{    
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency /1000);
  
  GPIO_init();
  
  m_flTosh.fl = k_vol;
  setReg(m_flTosh.sh[0],K_VOL_HIGH); 
  setReg(m_flTosh.sh[1],K_VOL_LOW); 
  m_flTosh.fl = b_vol;
  setReg(m_flTosh.sh[0],B_VOL_HIGH); 
  setReg(m_flTosh.sh[1],B_VOL_LOW); 
  m_flTosh.fl = k_cur;
  setReg(m_flTosh.sh[0],K_CUR_HIGH); 
  setReg(m_flTosh.sh[1],K_CUR_LOW); 
  m_flTosh.fl = b_cur;
  setReg(m_flTosh.sh[0],B_CUR_HIGH); 
  setReg(m_flTosh.sh[1],B_CUR_LOW); 
      
  //Стартовый "светофор"
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
  
  minCCR = maxCCR*0.65;                                                          //Устанавливаем максимальную скважность 50%
  CCR = maxCCR;                                                                 //Устанавливаем текущую скважность в 0
  
  tim3_pwm_init(PWM_FREQ);                                                      //Инит таймера шима с частотой PWM_FREQ
 
  Delay(20);
  
  adcVoltage_init();                                                            //Инициализация АЦП напряжения
  adcCurrent_init();                                                            //Инициализация АЦП тока
  
  tim4_init();                                                                  //Инит таймера регулятора
   
  adress=gerAdress();                                                           //Получаем адрес прибора
  modbusInit(adress);                                                           //Инит модбас адресом
  tim2_init();                                                                  //Юарт фейл-контроль
  usart_init();                                                                 //Инит юарт
  tim5_init();                                                                  //Контроль обрыва связи
  
  while(1){
      ustVoltage = (int16_t)getReg(UST_VOLTAGE_REG);                            //Обновляем уставку из регистров
      if(getReg(STOP_LINE_FLAG)){                                               //Если в регистре появилась команда на стоп, дёргаем релюшку на секунду
        STOP_LINE;
        Delay(1000);
        START_LINE;
        setReg(0,STOP_LINE_FLAG);
      }
      if(getReg(EEPROM_SET)){
        saveToEEPROM();

          
        setReg(0,EEPROM_SET);
      }
        
  }
}

//ОБНОВЛЯЕМ ДЕЙСТВУЮЩЕЕ НАПРЯЖЕНИЕ
void updateVoltage(double val){
    ADC_ActualVoltage = val;
    if(CCR == maxCCR)                                                           //Шим выключен
      actualVoltage = 0;
    else
      //actualVoltage =(int32_t)(val*1.5584 + 6.5);
      actualVoltage =(int32_t)(val*k_vol + b_vol);
    
    if(errorCounter==0) {                                                        //Штатный режим(нет ошибки связи), без мигания
      if(getReg(HIGH_VOL_REG) || actualVoltage>0)
        HV_LED_ON;
      else
        HV_LED_OFF;
    }
    setReg(actualVoltage,ACTUAL_VOLTAGE_REG);                                   //Записываем в регистры действующее напряжение
    //setReg(val,ACTUAL_VOLTAGE_REG);                                           //Чтобы выводить тугрики для калибровки
}

//ОБНОВЛЯЕМ ТОК
void updateCurrent(double val){
    ADC_ActualCurrent = val;
    
//    if(val<637){                                                                //637 соответствует 4.37 мкА, что соответствует 437 В. То есть до 437В используем одну формулу, а потом другую
//      actualCurrent =(int32_t)(0.0768*val-4.5);
//      if(actualCurrent<0)       actualCurrent=0;
//    }
//    else
//      actualCurrent =(int32_t)(0.0665*val+1.5);
    
    actualCurrent = (int32_t)(k_cur*val+b_cur);
    
    //Ток записываем с поправкой на скорость  из расчета, что при 100 м/мин паразитный ток 10 мкА.
    //Т.к. ток в регистрах модбас хранится умноженный на 10, то можно просто вычесть текущее значение скорости
    //actualCurrent-=speed;    
    
    //setReg(val,ACTUAL_CURR_REG);                                              //Чтобы выводить тугрики для калибровки
    
    if(actualCurrent>getReg(UST_CURR_REG) ){                                   //Проверка на дефект
      PROBOY_LED_ON;
      if(!defectDetected){                                                      //Если это первая точка дефекта
        defectDetected=1;
        readFlag=0;                                                             //Новый дефект и он еще не отправлен
        maxDefectCurrent = 0;                                                   //Сбрасываем максимальный ток текущего дефетка
        defectPos=position;                                                     //Запоминаем позицию дефекта
        setReg(getReg(DEFECTS_REG)+1, DEFECTS_REG);                             //Инкрементируем дефект
      }
      else{                                                                     //Если это не первая точка длинного дефетка
        if(abs(position-defectPos)>= getReg(MAX_DEFECT_LENGTH)){                //Если дефект уже длиннее, чем максимальная длина дефекта
          defectDetected=0;                                                     //сбрасываем факт дефекта, начиная новый дефект
          PROBOY_LED_OFF;
        }
      }
    }
    else{ 
      defectDetected=0;
      PROBOY_LED_OFF;
    }
    
    //Что запихаем в регистр на отправку?
    //Надо сохранить максимальный ток, который был в момент дефекта/дефектов, если их было более одного
    if(readFlag == 0){                                                          //Если случившийся дефект еще не отправлен,
       if(actualCurrent > maxDefectCurrent){                                    // сохраняем только наибольший ток
          maxDefectCurrent = actualCurrent;
          setReg(maxDefectCurrent,ACTUAL_CURR_REG);                                                               
       }
    }
    else{
      setReg(actualCurrent,ACTUAL_CURR_REG);                                    //Иначе, храним актуальный ток
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

    if(getReg(HIGH_VOL_REG) == 0 || ustVoltage==0)                              //Регулируем только если включено высокое и уставка не ноль
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

void saveADC_ActualVoltageEt1(){
  ADC_VoltageEt1 = ADC_ActualVoltage;
}
void saveADC_ActualVoltageEt2(){
  ADC_VoltageEt2 = ADC_ActualVoltage;
}
void saveADC_ActualCurrentEt1(){
  ADC_CurrentEt1 = ADC_ActualCurrent;
}
void saveADC_ActualCurrentEt2(){
  ADC_CurrentEt2 = ADC_ActualCurrent;
}
//Функции записи коэффициентов в EEPROM
void saveToEEPROM(){

  k_vol = ((double)getReg(ET_VOL2) - (double)getReg(ET_VOL1))/(ADC_VoltageEt2 - ADC_VoltageEt1);
  b_vol = getReg(ET_VOL1)-k_vol*ADC_VoltageEt1;
  k_cur = (float)(getReg(ET_CUR2) - getReg(ET_CUR1)/(ADC_CurrentEt2 - ADC_CurrentEt1));
  b_cur = getReg(ET_CUR1)- k_cur*ADC_CurrentEt1;
  
  m_flTosh.fl = k_vol;
  setReg(m_flTosh.sh[0],K_VOL_HIGH); 
  setReg(m_flTosh.sh[1],K_VOL_LOW); 
  m_flTosh.fl = b_vol;
  setReg(m_flTosh.sh[0],B_VOL_HIGH); 
  setReg(m_flTosh.sh[1],B_VOL_LOW); 
  m_flTosh.fl = k_cur;
  setReg(m_flTosh.sh[0],K_CUR_HIGH); 
  setReg(m_flTosh.sh[1],K_CUR_LOW); 
  m_flTosh.fl = b_cur;
  setReg(m_flTosh.sh[0],B_CUR_HIGH); 
  setReg(m_flTosh.sh[1],B_CUR_LOW); 
  
    PROBOY_LED_ON;
    HV_LED_ON;
    Delay(1000);
    PROBOY_LED_OFF;
    HV_LED_OFF;


}