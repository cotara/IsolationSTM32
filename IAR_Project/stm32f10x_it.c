#include "stm32f10x_it.h"
#include "stm32f10x_usart.h"
#include "user_TIMER.h"
#include "user_USART.h"
#include "user_GPIO.h"
#include "stm32f10x_tim.h"
#include <stdlib.h>
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "modbus.h"
#include "main.h"


extern volatile uint8_t   tx_buffer[TX_BUF_SIZE];
extern volatile unsigned long  tx_wr_index,tx_rd_index,tx_counter;
uint32_t dmaVoltageCounter=0,vel_adc=0;
uint32_t adcVoltageSum=0,adcVoltageSample=940,adcVoltageCounter=0;
uint32_t adcCurrentSum=0,adcCurrentSample=940,adcCurrentCounter=0;
uint8_t currentBuf[3500];
extern uint32_t ADC1ConvertedValue,ADC3ConvertedValue;
double voltageADCValue,currentADCValue;
uint32_t ms_counter=0;

uint32_t tempCounter=0;
uint8_t errorCounter=0;

uint32_t selfCalibValue=0;

void HardFault_Handler(void){
  while (1)
  {}
}

void MemManage_Handler(void){
  while (1)
  {}
}

void BusFault_Handler(void){
   while (1)
  {}
}


void SysTick_Handler(void){
  TimingDelay_Decrement();
  ms_counter++;
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
//USART fail control
void TIM2_IRQHandler(void){
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    
    TIM_Cmd(TIM2, DISABLE);
    modbusProcess();
}
//PWM
void TIM3_IRQHandler(void){
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    
    
}
//Regulator
void TIM4_IRQHandler(void){
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    regulatorAct();
}
//РћР‘СЂС‹РІ 485
void TIM5_IRQHandler(void){
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    
    if(errorCounter>=10){
        setReg(0,UST_VOLTAGE_REG);
        setReg(0,HIGH_VOL_REG);
        HV_LED_TOOGLE;
    }
    else
      errorCounter++;
}

void DMA1_Channel1_IRQHandler(void){
  if((DMA_GetITStatus(DMA1_IT_TC1) == SET)){ 
    adcVoltageSum+=ADC1ConvertedValue;
    adcVoltageCounter++;
    if(adcVoltageCounter>=adcVoltageSample){
      voltageADCValue = adcVoltageSum/adcVoltageSample;                         //Average Voltage
      updateVoltage(voltageADCValue);
      adcVoltageCounter=0;
      adcVoltageSum=0;
      dmaVoltageCounter++;
      vel_adc = dmaVoltageCounter*1000/ms_counter;
    }
  }
  DMA_ClearITPendingBit(DMA1_IT_TC1);
}

void DMA2_Channel4_5_IRQHandler(void){
    if((DMA_GetITStatus(DMA2_IT_TC5) == SET)){ 
      //currentBuf[adcCurrentCounter]=ADC3ConvertedValue;
      adcCurrentSum+=ADC3ConvertedValue;
      adcCurrentCounter++;
      if(adcCurrentCounter>=adcCurrentSample){
        currentADCValue = (double)adcCurrentSum/(adcCurrentSample/10.0);                //Average Current*10
                                 
        updateCurrent(currentADCValue);
        adcCurrentCounter=0;
        adcCurrentSum=0;
      }   
  }
  DMA_ClearITPendingBit(DMA2_IT_TC5);
}

void USART1_IRQHandler(void){
  if ((USART1->SR & USART_FLAG_RXNE)){
    TIM2->CNT = 0;
    TIM_Cmd(TIM2, ENABLE);
    toBuf(USART_ReceiveData(USART1));
    errorCounter=0;
  }

  if(USART_GetITStatus(USART1, USART_IT_TXE) == SET) {                           //прерывание по передаче
    USART_ClearITPendingBit(USART1, USART_IT_TXE);
    
    if (tx_counter) {                                                         //если есть что передать
      --tx_counter;                                                           // уменьшаем количество не переданных данных
      USART_SendData(USART1,tx_buffer[tx_rd_index++]);                        //передаем данные инкрементируя хвост буфера
      if (tx_rd_index == TX_BUF_SIZE) tx_rd_index=0;                       //идем по кругу
    }
    else {
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);                           //если нечего передать, запрещаем прерывание по передаче
      USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    }
  }
  if (USART_GetITStatus(USART1, USART_IT_TC) != RESET) {  
     USART_ClearITPendingBit(USART1, USART_IT_TC); 
     RX485EN;
     USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  } 
}
/*******************************************************************************/

