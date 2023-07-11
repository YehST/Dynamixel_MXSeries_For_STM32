
#include "main.h"
#include "Dynamixel_Protocol_1.h"

/* -------------------------- user define ----------------------------- */
/* User private variables -------------------------------------- */
UART_HandleTypeDef* Dynamixel_huart;
/* User private defined -------------------------------------- */
#define DYNAMIXEL_TX_MODE HAL_HalfDuplex_EnableTransmitter(Dynamixel_huart);
#define DYNAMIXEL_RX_MODE HAL_HalfDuplex_EnableReceiver(Dynamixel_huart);
/* User private function prototypes -----------------------------------------------*/
void Dynamixel_Init_Handle(UART_HandleTypeDef* huart){
  Dynamixel_huart = huart;
}

static uint8_t CheckSUM(uint8_t* data, int8_t len){
  uint8_t sum=0;
  for(int i = 0; i < len; i++){
    sum += data[i];
    // printData("%x ", data[i]);
  }
  return ~sum;
  
}
static void Send_Data(uint8_t* data, int8_t len){
  DYNAMIXEL_TX_MODE;
  uint8_t TxData[len+2];
  TxData[0] = 0xFF;
  TxData[1] = 0xFF;
  memcpy(&TxData[2], data, len);
  HAL_UART_Transmit(Dynamixel_huart, TxData, sizeof(TxData), 10);
  // for(int i = 0; i<len+2; i++)
  //   printData("%x ", TxData[i]);
  // printData("\n");
}
static void Receive_Data(Dynamixel_SERVO* servo, uint8_t* data){
  DYNAMIXEL_RX_MODE;
  int8_t len;
  uint8_t CMD;
  if (data[2] == DYNAMIXEL_INST_Read){
    len = data[4]+6;
    CMD = data[3];}
  else if (data[2] == DYNAMIXEL_INST_Write){
    len = 6;
    CMD = 0xFF;}
  else if (data[2] == DYNAMIXEL_INST_Ping){
    len = 6;
    CMD = DYNAMIXEL_INST_Ping;}
  uint8_t RxData[len];  
  HAL_UART_Receive(Dynamixel_huart, RxData, sizeof(RxData), 10);
  // for(int i = 0; i<sizeof(RxData); i++)
  //   printData("%x ", RxData[i]);
  // printData("\n");
  Decode_Data(servo, CMD, RxData, sizeof(RxData));
}
static void Decode_Data(Dynamixel_SERVO* servo, uint8_t CMD, uint8_t* RxData, int8_t len){
  // for(int i = 0; i<len; i++)
  //   printData("%x ", RxData[i]);
  // printData("\n");
  if(RxData[0] == 0xFF && RxData[1] == 0xFF && RxData[2] == servo->ID && RxData[len-1] == CheckSUM(&RxData[2], len-3)){
    servo->Error = RxData[4];
    switch(CMD){
      case DYNAMIXEL_INST_Ping:
        printData("Ping motor ID: %d\n", servo->ID);
        break;
      case DYNAMIXEL_CT_EEPROM_ModelNumber:
        servo->ModelNumber = (int8_t)((RxData[6]<<8)|RxData[5]);
        servo->FirmVersion = RxData[7];
        printData("ModelNumber: %d FirmVersion: %x\n", servo->ModelNumber, servo->FirmVersion);
        break;
      case DYNAMIXEL_CT_RAM_D_Gain:
        servo->controller.Kp = (int8_t)(RxData[7]);
        servo->controller.Ki = (int8_t)(RxData[6]);
        servo->controller.Kd = (int8_t)(RxData[5]);
        break;
      case DYNAMIXEL_CT_RAM_PresentPosition:
        servo->state.Raw_Encoder = (int)((RxData[6]<<8)|RxData[5]);
        servo->state.Present_Position = (float)servo->state.Raw_Encoder*360./4096.;
        break;
      
      case 0xFF:
        break;
    }
  }
  else{printData("Wrong data received\n");}
}

void Dynamixel_PING(Dynamixel_SERVO* servo){
  uint8_t data[4];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Ping;
  data[3] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_R_Model(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_EEPROM_ModelNumber;
  data[4] = 0x03;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_ID(Dynamixel_SERVO* servo, uint8_t ID){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_ID;
  data[4] = ID;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_BaudRate(Dynamixel_SERVO* servo, int BaudRate){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_BaudRate;
  data[4] = (uint8_t)((2000000./BaudRate-1)+0.5);
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_ReturnDelay(Dynamixel_SERVO* servo, int8_t ReturnDelay_us){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_ReturnDelay;
  data[4] = (uint8_t)(ReturnDelay_us/2);
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_CWAngleLimit(Dynamixel_SERVO* servo, float Pos){
  uint8_t data[7];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_CWAngleLimit;
  uint16_t Position = (uint16_t)(Pos*4096/360);
  data[4] = (uint8_t)(Position);
  data[5] = (uint8_t)(Position>>8); 
  data[6] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_CCWAngleLimit(Dynamixel_SERVO* servo, float Pos){
  uint8_t data[7];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_CCWAngleLimit;
  uint16_t Position = (uint16_t)(Pos*4096/360);
  data[4] = (uint8_t)(Position);
  data[5] = (uint8_t)(Position>>8); 
  data[6] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_TemperatureLimit(Dynamixel_SERVO* servo, int8_t Temperature){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_TempLimit;
  data[4] = (uint8_t)(Temperature);
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_MinVoltageLimit(Dynamixel_SERVO* servo, float MinVotalge){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_MinVoltLimit;
  data[4] = (uint8_t)(MinVotalge*10); //5.0V-16.0V
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_MaxVoltageLimit(Dynamixel_SERVO* servo, float MaxVotalge){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_MaxVoltLimit;
  data[4] = (uint8_t)(MaxVotalge*10); //5.0V-16.0V
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_MaxTorque(Dynamixel_SERVO* servo, uint8_t percent){
  uint8_t data[7];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_MaxTorque;
  uint16_t MaxTorque = (uint16_t)1023*percent/100;
  data[4] = (uint8_t)(MaxTorque);
  data[5] = (uint8_t)(MaxTorque>>8); 
  data[6] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_TorqueEnable(Dynamixel_SERVO* servo, uint8_t Enable){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_RAM_TorqueEnable;
  data[4] = Enable; // Torque enable
  // data[5] = Enable; // LED
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}
void Dynamixel_W_LED(Dynamixel_SERVO* servo, uint8_t Enable){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_RAM_LED;
  data[4] = Enable;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}
void Dynamixel_W_PID(Dynamixel_SERVO* servo, uint8_t Kp, uint8_t Ki, uint8_t Kd){
  uint8_t data[8];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_RAM_D_Gain;
  data[4] = Kd;
  data[5] = Ki; 
  data[6] = Kp; 
  data[7] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}
void Dynamixel_R_PID(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_RAM_D_Gain;
  data[4] = 0x03;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}
void Dynamixel_W_GoalPosition(Dynamixel_SERVO* servo, float Pos){
  uint8_t data[7];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_RAM_GoalPosition;
  uint16_t Position = (uint16_t)(Pos*4096/360);
  data[4] = (uint8_t)(Position);
  data[5] = (uint8_t)(Position>>8); 
  data[6] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}
void Dynamixel_R_PresentPos(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_RAM_PresentPosition;
  data[4] = 0x02; //return data length
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}