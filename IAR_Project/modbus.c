#include "modbus.h"
#include "user_USART.h"


extern uint8_t RXi;
extern uint8_t TXi;
extern uint8_t rxBuf[RX_BUF_SIZE];
extern uint8_t txBuf[TX_BUF_SIZE];
extern uint8_t currentBuf[3500];

unsigned short CRC16 = 0;
uint16_t regs[REG_SIZE] = {'\0'};
uint8_t modAdd;
int32_t position;

void modbusInit(uint8_t add){
  modAdd = add;
  regs[0] = (4 << 8) + 1;                                                         //Type + model
  regs[1]=0;
  regs[2]=0;
  regs[3]=0;
  regs[4]=0;
  regs[5]=350;
  regs[6]=200;
  regs[7]=100;  
  regs[8]=10; 
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
      
  if(((CRC16&0xFF) == rxBuf[6] )&& ((CRC16>>8) == rxBuf[7])){                   //совпало срс
    if(rxBuf[2] == 0 && (rxBuf[5]+rxBuf[3]) <= REG_SIZE){                          //проверка запрашиваемого диапазона
      txBuf[0] = modAdd;
      txBuf[1] = 0x03;
      txBuf[2] = rxBuf[5] *2;                                                   // в ответе байт в 2 раза больше чем регистров
   
      for(int b_count  =0; b_count < rxBuf[5]; b_count++)     {
       txBuf[b_count*2+3] = regs[b_count+ rxBuf[3]]>>8;
       txBuf[b_count*2+4] = regs[b_count+rxBuf[3]]&0xFF;
      }
      CRC16 = crc16(txBuf,rxBuf[5]*2+3);
   
      txBuf[rxBuf[5]*2+3] = CRC16&0xFF;                                         //CRC H
      txBuf[rxBuf[5]*2+4] = CRC16>>8;                                           //CRC L

      USART1_put_string(txBuf,rxBuf[5]*2+5);
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
   txBuf[2] = rxBuf[2];
   txBuf[3] = rxBuf[3];
   txBuf[4] = rxBuf[4];
   txBuf[5] = rxBuf[5];  
   
   CRC16 = crc16(txBuf,6);
   
   txBuf[6] = CRC16&0xFF;                                                       //CRC L
   txBuf[7] = CRC16>>8;                                                         //CRC H
   
   if((txBuf[6] == rxBuf[6]) && (txBuf[7] == rxBuf[7]))                         //совпало срс
    {
      if(rxBuf[3]<REG_SIZE)                                                     //запись значения по адресу
        regs[rxBuf[3]] = (rxBuf[4]<<8)|rxBuf[5];    
    }
    USART1_put_string(txBuf,8);
      return 0;
}           

uint8_t modbus16(){
  int tempCount = RXi-2;
  RXi=0;
  CRC16 = crc16(rxBuf,tempCount);
  
  if(((CRC16&0xFF) == rxBuf[tempCount]) && (CRC16>>8 == rxBuf[tempCount+1])){                         //совпало срс
     txBuf[0] = modAdd;
     txBuf[1] = 0x10;
     txBuf[2] = rxBuf[2];
     txBuf[3] = rxBuf[3];
     txBuf[4] = rxBuf[4];
                                                    
     uint8_t countSet=0;
     if(rxBuf[3]<REG_SIZE){                                                     //запись значения по адресу
       for(int i=0;i<rxBuf[5] && (rxBuf[3] + i < REG_SIZE);i++){
          regs[rxBuf[3]+i] = (rxBuf[7+2*i]<<8) + rxBuf[8+2*i];
          countSet++;
       }
     }
     txBuf[5] = countSet;
     CRC16 = crc16(txBuf,6);
     txBuf[6] = CRC16&0xFF;                                                       //CRC L
     txBuf[7] = CRC16>>8; 
   }
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