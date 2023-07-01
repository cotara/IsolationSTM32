#include <stdint.h>


void Delay(volatile uint32_t nTime);
void TimingDelay_Decrement(void);
void tim2_init(void);
void tim3_pwm_init(uint16_t startFreq);
void tim3_init(void);
uint16_t getCcr3Tim();
void setCcr3Tim(uint16_t val);
void tim4_init(void);