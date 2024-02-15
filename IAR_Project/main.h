#define PWM_FREQ 500
#define MAX_DUTY 40
void updateVoltage(double val);
void updateCurrent(double val);

uint8_t gerAdress();
void regulatorAct();
void saveADC_ActualVoltageEt1();
void saveADC_ActualVoltageEt2();
void saveADC_ActualCurrentEt1();
void saveADC_ActualCurrentEt2();
void saveToEEPROM();

void SaveSettings();
void LoadSettings();