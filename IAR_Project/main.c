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
#include "eeprom.h"

int32_t actualVoltage=0;
int32_t actualCurrent=0, maxDefectCurrent=0;
int16_t ustVoltage=0;
double regKoefCur = 0.1, regKoef = 0.03;

double CCR=0, minCCR=0,maxCCR =9000000/PWM_FREQ ;
int32_t maxCurrent=300;                                                         //25 mkA*10
double err;                                                                     //Рассогласование
uint8_t defectDetected = 0;                                                     
int32_t defectPos=0;
extern int32_t position;
extern int32_t speed;
extern uint8_t errorCounter;
extern uint8_t readFlag;  
uint32_t counter__ = 0;
uint8_t adress=0;
float k_vol, b_vol, k_cur, b_cur;
float k_vol_default=1, b_vol_default=50, k_cur_default=0.07, b_cur_default=3;
double ADC_ActualVoltage=0,ADC_VoltageEt1=0, ADC_VoltageEt2=0,ADC_ActualCurrent=0,ADC_CurrentEt1=0,ADC_CurrentEt2=0;
double dutyCycle=0;
typedef union {
  uint16_t sh[2];
  float fl;
}flTosh_t;

typedef union {
  uint8_t ch[4];
  float fl;
}flToCh_t;

flTosh_t m_flTosh;
flToCh_t m_flToCh;
//Различия в пинах индикации (user_GPIO.c и user_GPIO.h) 
int main()
{    
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency /1000);
  
  adress=gerAdress();                                                           //Получаем адрес прибора
  modbusInit(adress);                                                           //Инит модбас адресом
  
  GPIO_init();
  
  I2C_Configuration();                                                          //ИНИТ EEPROM        
  
  LoadSettings();   
  
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
  
  
                                                           //Загрузка настроек
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

  maxCCR = MAX_DUTY/100.0*maxCCR;                                               //Устанавливаем максимальную скважность 50%
  CCR = minCCR;                                                                 //Устанавливаем текущую скважность в 0
  
  tim3_pwm_init(PWM_FREQ);                                                      //Инит таймера шима с частотой PWM_FREQ
 
  Delay(20);
  
  adcVoltage_init();                                                            //Инициализация АЦП напряжения
  adcCurrent_init();                                                            //Инициализация АЦП тока
  
  tim4_init();                                                                  //Инит таймера регулятора
   

  tim2_init();                                                                  //Юарт фейл-контроль
  usart_init();                                                                 //Инит юарт
  tim5_init();                                                                  //Контроль обрыва связи
  
  while(1){
      ustVoltage = (int16_t)getReg(UST_VOLTAGE_REG);                            //Обновляем уставку из регистров

      if(getReg(EEPROM_SET)!=0){
        saveToEEPROM();
        setReg(0,EEPROM_SET);
      }
  }
}

//ОБНОВЛЯЕМ ДЕЙСТВУЮЩЕЕ НАПРЯЖЕНИЕ
void updateVoltage(double val){
    ADC_ActualVoltage = val;
    if(CCR == minCCR)                                                           //Шим выключен
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
    
    actualCurrent = (int16_t)(k_cur*val+b_cur);
    if(actualCurrent<0)       actualCurrent=0;
    //actualCurrent = val*3.3/4096.0/11*1000*10;                                  //(Напряжение на АЦП/11000) - это ток в амперах. Умножаем на 10^6 - это микроамперы. И умножаем на 10 - это для передачи
    
    
    //Ток записываем с поправкой на скорость  из расчета, что при 100 м/мин паразитный ток 10 мкА.
    //Т.к. ток в регистрах модбас хранится умноженный на 10, то можно просто вычесть текущее значение скорости
    //actualCurrent-=speed;    
    
    //setReg((unsigned short)val,ACTUAL_CURR_REG);                                //Чтобы выводить тугрики для калибровки
    
    if(actualCurrent>getReg(UST_CURR_REG) ){                                    //Проверка на дефект
      PROBOY_LED_ON;
      STOP_LINE;
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
      START_LINE;
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
      CCR=minCCR;
    else{
      //if(actualCurrent >= maxCurrent && ustVoltage>actualVoltage)                   //If there is an overcurrent and the voltage setting is higher than the real values
      if(actualCurrent >= maxCurrent)   
        err = regKoefCur*(maxCurrent-actualCurrent);
      else
        err = regKoef*(ustVoltage-actualVoltage);
      
      CCR = CCR + err;
      
      if(CCR<minCCR) 
        CCR = minCCR;
      else if(CCR > maxCCR) 
        CCR = maxCCR;
    }
    dutyCycle = CCR/maxCCR*MAX_DUTY;
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
void update_maxSystemCur(){
  maxCurrent = getReg(MAX_CUR);
}
//Функции записи коэффициентов в EEPROM
void saveToEEPROM(){
  unsigned short calibCommand = getReg(EEPROM_SET);
  
  if(calibCommand==1){
    k_vol = ((double)getReg(ET_VOL2) - (double)getReg(ET_VOL1))/(ADC_VoltageEt2 - ADC_VoltageEt1);
    b_vol = getReg(ET_VOL1)-k_vol*ADC_VoltageEt1;
  }
  else if(calibCommand==2){
    k_cur = (float)((double)getReg(ET_CUR2) - (double)getReg(ET_CUR1))/(ADC_CurrentEt2 - ADC_CurrentEt1);
    b_cur = getReg(ET_CUR1)- k_cur*ADC_CurrentEt1;
  }
  
  else if(calibCommand==3){                                                      //Восстановить настройки по умолчанию
    k_vol = k_vol_default;
    b_vol = b_vol_default;
    k_cur = k_cur_default;
    b_cur = b_cur_default;
  }
  SaveSettings();                                                               //Сохранение настроек в EEPROM

  //Пишем коэффициенты в модбас, чтобы было видно в проге
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

void LoadSettings()
{
  m_flToCh.ch[0] = I2C_EE_ByteRead(0);
  m_flToCh.ch[1] = I2C_EE_ByteRead(1);
  m_flToCh.ch[2] = I2C_EE_ByteRead(2);
  m_flToCh.ch[3] = I2C_EE_ByteRead(3);
  k_vol = m_flToCh.fl;
  
  m_flToCh.ch[0] = I2C_EE_ByteRead(4);
  m_flToCh.ch[1] = I2C_EE_ByteRead(5);
  m_flToCh.ch[2] = I2C_EE_ByteRead(6);
  m_flToCh.ch[3] = I2C_EE_ByteRead(7);
  b_vol = m_flToCh.fl;
  
  m_flToCh.ch[0] = I2C_EE_ByteRead(8);
  m_flToCh.ch[1] = I2C_EE_ByteRead(9);
  m_flToCh.ch[2] = I2C_EE_ByteRead(10);
  m_flToCh.ch[3] = I2C_EE_ByteRead(11);
  k_cur = m_flToCh.fl;
  
  m_flToCh.ch[0] = I2C_EE_ByteRead(12);
  m_flToCh.ch[1] = I2C_EE_ByteRead(13);
  m_flToCh.ch[2] = I2C_EE_ByteRead(14);
  m_flToCh.ch[3] = I2C_EE_ByteRead(15);
  b_cur = m_flToCh.fl;
}

void SaveSettings(){
   m_flToCh.fl = k_vol;
   I2C_EE_ByteWrite(m_flToCh.ch[0],0);
   I2C_EE_ByteWrite(m_flToCh.ch[1],1);
   I2C_EE_ByteWrite(m_flToCh.ch[2],2);
   I2C_EE_ByteWrite(m_flToCh.ch[3],3);
   
   m_flToCh.fl = b_vol;
   I2C_EE_ByteWrite(m_flToCh.ch[0],4);
   I2C_EE_ByteWrite(m_flToCh.ch[1],5);
   I2C_EE_ByteWrite(m_flToCh.ch[2],6);
   I2C_EE_ByteWrite(m_flToCh.ch[3],7);
   
   m_flToCh.fl = k_cur;
   I2C_EE_ByteWrite(m_flToCh.ch[0],8);
   I2C_EE_ByteWrite(m_flToCh.ch[1],9);
   I2C_EE_ByteWrite(m_flToCh.ch[2],10);
   I2C_EE_ByteWrite(m_flToCh.ch[3],11);
   
   m_flToCh.fl = b_cur;
   I2C_EE_ByteWrite(m_flToCh.ch[0],12);
   I2C_EE_ByteWrite(m_flToCh.ch[1],13);
   I2C_EE_ByteWrite(m_flToCh.ch[2],14);
   I2C_EE_ByteWrite(m_flToCh.ch[3],15);
}
