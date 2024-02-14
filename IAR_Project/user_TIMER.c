#include "user_TIMER.h"
#include "stm32f10x_tim.h"
#include "main.h"

volatile uint32_t TimingDelay;
uint16_t CCR1_Val = 9000000/PWM_FREQ;

void tim2_init(void){
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 72-1;                                   //1 мкс
    TIMER_InitStructure.TIM_Period = 1000-1;                                    //1 мс
    TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    //TIM_Cmd(TIM2, ENABLE);
  
    NVIC_SetPriority (TIM2_IRQn, 0);
    NVIC_EnableIRQ (TIM2_IRQn);
}



void tim3_init(void){
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 72-1;                                   //1 мкс
    TIMER_InitStructure.TIM_Period = 333-1;                                     //2000 Гц
    TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
  
    NVIC_SetPriority (TIM3_IRQn, 0);
    NVIC_EnableIRQ (TIM3_IRQn);
}

void tim3_pwm_init(uint16_t startFreq){
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  

  /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;

    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 8-1;                                    //9Мгц
    TIMER_InitStructure.TIM_Period = 9000000/startFreq;                         //500 Гц
    TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);  

  
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  
  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |  RCC_APB2Periph_AFIO, ENABLE);
  
  
  /* GPIOA Configuration:TIM3 Channel1, 2, 3 and 4 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
}

void setCcr3Tim(uint16_t val){
  //TIM3 - CH2 -> шим PA7
  
  TIM3->CCR2 = val;
}

uint16_t getCcr3Tim(){
  return TIM3->CCR2;
}

//Таймер регулятора
void tim4_init(void){
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 7200-1;                                 //0.1 мс
    TIMER_InitStructure.TIM_Period = 200-1;                                     //20 мс
    TIM_TimeBaseInit(TIM4, &TIMER_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
  
    NVIC_SetPriority (TIM4_IRQn, 0);
    NVIC_EnableIRQ (TIM4_IRQn);
}

//Таймер обрыва связи
void tim5_init(void){
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 7200-1;                                 //0.1 мс
    TIMER_InitStructure.TIM_Period = 5000-1;                                   //0.5с 
    TIM_TimeBaseInit(TIM5, &TIMER_InitStructure);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
  
    NVIC_SetPriority (TIM5_IRQn, 0);
    NVIC_EnableIRQ (TIM5_IRQn);
}
/*Задержка в nTime 10мс*/
void Delay(volatile uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}
/**/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

