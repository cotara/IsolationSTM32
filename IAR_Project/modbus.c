#include "modbus.h"
#include "user_USART.h"


extern uint8_t RXi;
extern uint8_t TXi;
extern uint8_t rxBuf[RX_BUF_SIZE];
extern uint8_t txBuf[TX_BUF_SIZE];
extern uint8_t currentBuf[3500];
uint8_t readFlag = 1;                                                           //PROCHITANO!
unsigned short CRC16 = 0;
uint16_t regs[REG_SIZE] = {'\0'};
uint8_t modAdd;
int32_t position=0;
int32_t speed=0;
void modbusInit(uint8_t add){
  modAdd = add;
  regs[0] = (4 << 8) + 1;                                                       //Type + model
  regs[1]=0;                                                                    // действующее значение напряжения
  regs[2]=0;                                                                    //действующая сила тока
  regs[3]=0;                                                                    //количество дефектов
  regs[4]=0;                                                                    //высокое вкл/выкл
  regs[5]=0;                                                                    //Уставка по напряжению
  regs[6]=200;                                                                  //уставка по току
  regs[7]=100;                                                                  //уставка по длине
  regs[8]=10;                                                                   //максимальная длина дефекта
  regs[9]=0;                                                                    //стоп линия
  //Калибровка
  regs[10]=0;                                                                   //Эталонное напряжение 1 
  regs[11]=0;                                                                   //Эталонное напряжение 2
  regs[12]=0;                                                                   //Эталонный ток 1
  regs[13]=0;                                                                   //Эталонный ток 2
  regs[14]=0;                                                                   //Записать значение в EEPROM
  regs[15]=0;                                                                   //Коэффициенты калибровки напряжения
  regs[16]=0;
  regs[17]=0;
  regs[18]=0;
  regs[19]=0;                                                                   //Коэффициенты калибровки тока
  regs[20]=0;
  regs[21]=0;
  regs[22]=0;
  
  regs[23]=300;                                                                 //Максимальный ток в системе
}
uint8_t setReg(uint16_t val, uint8_t index){
    if(index>REG_SIZE-1) return 1;
    else{
      regs[index] = val;
      return 0;
    }
}

uint16_t getReg(uint8_t index){
    if(index>REG_SIZE-1) return 0;
    else 
      return regs[index];
}
uint8_t modbusProcess(){
  if(RXi<8){ RXi = 0;    return 1;  }
  
  if(rxBuf[0] != modAdd) {
    if( rxBuf[0] == 2){                       //sniffing ID answer
      position = 65536*((rxBuf[11]<<8) + rxBuf[12]) + ((rxBuf[13]<<8) + rxBuf[14] );
      speed = 65536*((rxBuf[23]<<8) + rxBuf[24]) + ((rxBuf[25]<<8) + rxBuf[26] ) / 10;  //speed in m/min in int!
    }  
    RXi = 0;      
    return 1;
  }
  if(rxBuf[1] == 3)
    return modbus3();
  

  else if(rxBuf[1] == 6)
    return modbus6();  
  else if(rxBuf[1] == 16)
    return modbus16(); 
  
  else return 1;
}

uint8_t modbus3(){
  RXi=0;
  CRC16 = crc16(rxBuf,6);
      
  if(((CRC16&0xFF) == rxBuf[6] )&& ((CRC16>>8) == rxBuf[7])){                   //������� ���
    if(rxBuf[2] == 0 && (rxBuf[5]+rxBuf[3]) <= REG_SIZE){                          //�������� �������������� ���������
      txBuf[0] = modAdd;
      txBuf[1] = 0x03;
      txBuf[2] = rxBuf[5] *2;                                                   // � ������ ���� � 2 ���� ������ ��� ���������
   
      for(int b_count  =0; b_count < rxBuf[5]; b_count++)     {
       txBuf[b_count*2+3] = regs[b_count+ rxBuf[3]]>>8;
       txBuf[b_count*2+4] = regs[b_count+rxBuf[3]]&0xFF;
      }
      CRC16 = crc16(txBuf,rxBuf[5]*2+3);
   
      txBuf[rxBuf[5]*2+3] = CRC16&0xFF;                                         //CRC H
      txBuf[rxBuf[5]*2+4] = CRC16>>8;                                           //CRC L

      USART1_put_string(txBuf,rxBuf[5]*2+5);
      readFlag=1;
      return 0;
    }
    else return 1;
  }
  else return 1;
}
           
uint8_t modbus6(){
   RXi=0; 
   txBuf[0] = modAdd;
   txBuf[1] = 0x06;
   CRC16 = crc16(rxBuf,6);
   
   if(((CRC16&0xFF) == rxBuf[6] )&& ((CRC16>>8) == rxBuf[7])){                         //������� ���
    txBuf[2] = rxBuf[2];
    txBuf[3] = rxBuf[3];
    txBuf[4] = rxBuf[4];
    txBuf[5] = rxBuf[5]; 
    if(rxBuf[3]<REG_SIZE)                                                     //������ �������� �� ������
      regs[rxBuf[3]] = (rxBuf[4]<<8)|rxBuf[5];    
    
    if(rxBuf[3] == 0xA)                                                        //Установили etVol1
       saveADC_ActualVoltageEt1();
     else if(rxBuf[3] == 0xB)                                                   //Установили etVol2
       saveADC_ActualVoltageEt2();
     else if(rxBuf[3] == 0xC)                                                        //Установили etCur1
       saveADC_ActualCurrentEt1();
     else if(rxBuf[3] == 0xD)                                                        //Установили etCur2
       saveADC_ActualCurrentEt2(); 
     else if(rxBuf[3] == 0x17)                                                        //Установили максимальный ток в системе
       update_maxSystemCur();
   }
   else{
      txBuf[2] = 0;
      txBuf[3] = 0;
      txBuf[4] = 0;
      txBuf[5] = 0; 
   }
     
   CRC16 = crc16(txBuf,6);
   
   txBuf[6] = CRC16&0xFF;                                                       //CRC L
   txBuf[7] = CRC16>>8;                                                         //CRC H
   
    USART1_put_string(txBuf,8);
      return 0;
}           

uint8_t modbus16(){
  int tempCount = RXi-2;
  RXi=0;
  CRC16 = crc16(rxBuf,tempCount);
  txBuf[0] = modAdd;
  txBuf[1] = 0x10;
  txBuf[2] = rxBuf[2];
  txBuf[3] = rxBuf[3];
  txBuf[4] = rxBuf[4];
  if(((CRC16&0xFF) == rxBuf[tempCount]) && (CRC16>>8 == rxBuf[tempCount+1])){                         //������� ���

                                                    
     uint8_t countSet=0;
     if(rxBuf[3]<REG_SIZE){                                                     //������ �������� �� ������
       for(int i=0;i<rxBuf[5] && (rxBuf[3] + i < REG_SIZE);i++){
          regs[rxBuf[3]+i] = (rxBuf[7+2*i]<<8) + rxBuf[8+2*i];
          countSet++;
       }
     }
     
     
    txBuf[5] = countSet;
   }
   else
     txBuf[5] = 0;
   CRC16 = crc16(txBuf,6);
   txBuf[6] = CRC16&0xFF;                                                       //CRC L
   txBuf[7] = CRC16>>8; 
   USART1_put_string(txBuf,8);
   return 0;
} 

unsigned short crc16(unsigned char *data_p, unsigned short len)
{
 unsigned short  crc = 0xFFFF;
 for (int i=0;i<len;i++)
 {
   crc=crc^data_p[i];
   for (int j=0;j<8;++j){
      if (crc&0x01)     crc =(crc>>1)^0xA001;
      else              crc=crc>>1;
   }
 }
 return(crc);
}