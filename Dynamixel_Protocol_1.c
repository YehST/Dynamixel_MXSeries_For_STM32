
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
  HAL_Delay(10);
}
void Dynamixel_Servo_Init(Dynamixel_SERVO* Allservo, int8_t NumOfServo){
  for (int i = 0; i<NumOfServo; i++){
    Allservo[i].StatusReturnLevel = 0x02;
    printData("---------------Initialize Servo-----------------\n");
    Dynamixel_PING(&Allservo[i]);
    Dynamixel_R_StatusReturnLevel(&Allservo[i]);
    Dynamixel_R_Model(&Allservo[i]);
    Dynamixel_R_PID(&Allservo[i]);
    Dynamixel_R_AllLimit(&Allservo[i]);
    printData("-------------------Done---------------------\n");
  }
}
static void Errorchk(Dynamixel_SERVO* servo){
  if (servo->Error.Error != 0x00){
    if (servo->Error.ErrorType.InputVoltage)
      printData("ERROR :: ID:%d Servo has Input Voltage Error.\n", servo->ID);
    if (servo->Error.ErrorType.AngleLimit)
      printData("ERROR :: ID:%d Servo has Angle Limit Error.\n", servo->ID);
    if (servo->Error.ErrorType.OverHeating)
      printData("ERROR :: ID:%d Servo has OverHeating Error.\n", servo->ID);
    if (servo->Error.ErrorType.Range)
      printData("ERROR :: ID:%d Servo has Range Error.\n", servo->ID);
    if (servo->Error.ErrorType.CheakSum)
      printData("ERROR :: ID:%d Servo has CheakSum Error.\n", servo->ID);
    if (servo->Error.ErrorType.OverLoad)
      printData("ERROR :: ID:%d Servo has OverLoad Error.\n", servo->ID);
    if (servo->Error.ErrorType.Instruction)
      printData("ERROR :: ID:%d Servo has Instruction Error.\n", servo->ID);
  }
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
  if (servo->StatusReturnLevel == 0x02 || (servo->StatusReturnLevel == 0x01 && data[2] != DYNAMIXEL_INST_Write)|| (servo->StatusReturnLevel == 0x00 && data[2] == DYNAMIXEL_INST_Ping)){
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
}
static void Decode_Data(Dynamixel_SERVO* servo, uint8_t CMD, uint8_t* RxData, int8_t len){
  // for(int i = 0; i<len; i++)
  //   printData("%x ", RxData[i]);
  // printData("\n");
  uint16_t tmp_data;
  if(RxData[0] == 0xFF && RxData[1] == 0xFF && RxData[2] == servo->ID && RxData[len-1] == CheckSUM(&RxData[2], len-3)){
    servo->Error.Error = RxData[4];
    Errorchk(servo);
    if (CMD == DYNAMIXEL_CT_RAM_PresentPosition && RxData[3] == 0x0A){
      CMD = DYNAMIXEL_READ_ALL_PresentData;
    }
    switch(CMD){
      case DYNAMIXEL_INST_Ping:
        printData("Ping motor ID: %d\n", servo->ID);
        break;
      case DYNAMIXEL_CT_EEPROM_CWAngleLimit:
        servo->Limit.CW_angle_limit = ((RxData[6]<<8)|RxData[5]);
        servo->Limit.CCW_angle_limit = ((RxData[8]<<8)|RxData[7]);
        servo->Limit.Temperature_limit = (RxData[10]);
        servo->Limit.Min_Volt_limit = (RxData[11]);
        servo->Limit.Max_Volt_limit = (RxData[12]);
        servo->Limit.Torque_limit = ((RxData[14]<<8)|RxData[13]);
        printData("%x %x %x %x %x %x\n", servo->Limit.CW_angle_limit, servo->Limit.CCW_angle_limit, servo->Limit.Temperature_limit, servo->Limit.Min_Volt_limit, servo->Limit.Max_Volt_limit, servo->Limit.Torque_limit);
        break;
      case DYNAMIXEL_CT_EEPROM_ModelNumber:
        servo->ModelNumber = (int8_t)((RxData[6]<<8)|RxData[5]);
        servo->FirmVersion = RxData[7];
        printData("ModelNumber: %d FirmVersion: %x\n", servo->ModelNumber, servo->FirmVersion);
        break;
      case DYNAMIXEL_CT_EEPROM_StatusReturnLevel:
        servo->StatusReturnLevel = RxData[5];
        printData("Status: %x\n", servo->StatusReturnLevel);
        break;
      case DYNAMIXEL_CT_EEPROM_AlarmLED:
        printData("Alarm: %x, ShutDown: %x\n", RxData[5], RxData[6]);
        break;
      case DYNAMIXEL_CT_EEPROM_MultiTurnOffset:
        tmp_data = (int)((RxData[6]<<8)|RxData[5]);
        servo->MultiTurnOffset = (float)tmp_data*360./4096.;
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
      case DYNAMIXEL_CT_RAM_PresentSpeed:
        tmp_data = ((RxData[6]<<8)|RxData[5]);
        if ((tmp_data & 0x400)==0)
          servo->state.Present_Speed = (float)(tmp_data & 0x03FF)*0.114;
        else
          servo->state.Present_Speed = -(float)(tmp_data & 0x03FF)*0.114;
        break;
      case DYNAMIXEL_CT_RAM_PresentLoad:
        tmp_data = ((RxData[6]<<8)|RxData[5]);
        if ((tmp_data & 0x400)==0)
          servo->state.Present_Load = (float)(tmp_data & 0x03FF)*100./1024.;
        else
          servo->state.Present_Load = -(float)(tmp_data & 0x03FF)*100./1024.;
        break;
      case DYNAMIXEL_CT_RAM_PresentVoltage:
        tmp_data = (int)RxData[5];
        servo->state.Present_voltage = (float)tmp_data;
        break;
      case DYNAMIXEL_CT_RAM_PresentTemperature:
        servo->state.Present_Temperature = (int8_t)(RxData[5]);
        break;
      case DYNAMIXEL_READ_ALL_PresentData:
        servo->state.Raw_Encoder = (int)((RxData[6]<<8)|RxData[5]);
        servo->state.Present_Position = (float)servo->state.Raw_Encoder*360./4096.;
        tmp_data = ((RxData[8]<<8)|RxData[7]);
        if ((tmp_data & 0x400)==0)
          servo->state.Present_Speed = (float)(tmp_data & 0x03FF)*0.114;
        else
          servo->state.Present_Speed = -(float)(tmp_data & 0x03FF)*0.114;
        tmp_data = ((RxData[10]<<8)|RxData[9]);
        if ((tmp_data & 0x400)==0)
          servo->state.Present_Load = (float)(tmp_data & 0x03FF)*100./1024.;
        else
          servo->state.Present_Load = -(float)(tmp_data & 0x03FF)*100./1024.;
        tmp_data = (int)RxData[11];
        servo->state.Present_voltage = (float)tmp_data/10.;
        servo->state.Present_Temperature = (int8_t)(RxData[12]);
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

void Dynamixel_R_AllLimit(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_EEPROM_CWAngleLimit;
  data[4] = 0x0A;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_StatusReturnLevel(Dynamixel_SERVO* servo, uint8_t Mode){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_StatusReturnLevel;
  data[4] = Mode;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_R_StatusReturnLevel(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_EEPROM_StatusReturnLevel;
  data[4] = 0x01;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  // printData("%d\n", sizeof(data)/sizeof(data[0]));
  Receive_Data(servo, data);
}

void Dynamixel_W_AlarmLED_ErrorType(Dynamixel_SERVO* servo, uint8_t ErrorType){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_AlarmLED;
  data[4] = ErrorType;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_W_ShutDown_ErrorType(Dynamixel_SERVO* servo, uint8_t ErrorType){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_ShutDown;
  data[4] = ErrorType;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_R_AlarmErrorType(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_EEPROM_AlarmLED;
  data[4] = 0x02;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_W_MultiTurnOffset(Dynamixel_SERVO* servo, float Offset){
  // oddset's range is from -8640 to 8640 (degrees)
  uint8_t data[7];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_MultiTurnOffset;
  uint16_t Position = (uint16_t)(Offset*4096./360.);
  data[4] = (uint8_t)(Position);
  data[5] = (uint8_t)(Position>>8); 
  data[6] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_R_MultiTurnOffset(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_EEPROM_MultiTurnOffset;
  data[4] = 0x02;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_W_ResolutionDivider(Dynamixel_SERVO* servo, uint8_t Divider){
  // Divider's range is from 1 to 4
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_EEPROM_ResolutionDivider;
  data[4] = Divider;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_R_ResolutionDivider(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_EEPROM_ResolutionDivider;
  data[4] = 0x01;
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}


void Dynamixel_Select_Model(Dynamixel_SERVO* servo, uint8_t Mode){
  /*
        0x01: Wheel Mode
        0x02: Joint Mode
        0x03: Multi-turn Mode
  */
 uint8_t data[9];
 uint16_t CMD;
  switch(Mode){
    case 0x01:
      data[0] = servo->ID;
      data[1] = sizeof(data)-2; //len
      data[2] = DYNAMIXEL_INST_Write;
      data[3] = DYNAMIXEL_CT_EEPROM_CWAngleLimit;
      CMD = 0;
      data[4] = (uint8_t)(CMD);
      data[5] = (uint8_t)(CMD>>8); 
      data[6] = (uint8_t)(CMD);
      data[7] = (uint8_t)(CMD>>8); 
      data[8] = CheckSUM(data, data[1]+1);
      break;
    case 0x02:
      data[0] = servo->ID;
      data[1] = sizeof(data)-2; //len
      data[2] = DYNAMIXEL_INST_Write;
      data[3] = DYNAMIXEL_CT_EEPROM_CWAngleLimit;
      CMD = 0;
      data[4] = (uint8_t)(CMD);
      data[5] = (uint8_t)(CMD>>8); 
      CMD = 4095;
      data[6] = (uint8_t)(CMD);
      data[7] = (uint8_t)(CMD>>8); 
      data[8] = CheckSUM(data, data[1]+1);
      break;
    case 0x03:
      data[0] = servo->ID;
      data[1] = sizeof(data)-2; //len
      data[2] = DYNAMIXEL_INST_Write;
      data[3] = DYNAMIXEL_CT_EEPROM_CWAngleLimit;
      CMD = 4095;
      data[4] = (uint8_t)(CMD);
      data[5] = (uint8_t)(CMD>>8); 
      data[6] = (uint8_t)(CMD);
      data[7] = (uint8_t)(CMD>>8); 
      data[8] = CheckSUM(data, data[1]+1);
      break;
  }
  servo->Mode = Mode;
  Send_Data(data, sizeof(data));
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
void Dynamixel_W_MovingSpeed(Dynamixel_SERVO* servo, float Speed){
  // from 0 to 116.62 in Joint Mode
  // from 0 to 233.35 in Wheel Mode
  uint8_t data[7];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Write;
  data[3] = DYNAMIXEL_CT_RAM_MovingSpeed;
  uint16_t V = (uint16_t)(Speed/0.114);
  data[4] = (uint8_t)(V);
  data[5] = (uint8_t)(V>>8); 
  data[6] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
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
  Receive_Data(servo, data);
}

void Dynamixel_R_PresentSpeed(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_RAM_PresentSpeed;
  data[4] = 0x02; //return data length
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_R_PresentLoad(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_RAM_PresentLoad;
  data[4] = 0x02; //return data length
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_R_PresentVoltage(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_RAM_PresentVoltage;
  data[4] = 0x01; //return data length
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_R_PresentTemperature(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_RAM_PresentTemperature;
  data[4] = 0x01; //return data length
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}

void Dynamixel_R_AllPresentData(Dynamixel_SERVO* servo){
  uint8_t data[6];
  data[0] = servo->ID;
  data[1] = sizeof(data)-2; //len
  data[2] = DYNAMIXEL_INST_Read;
  data[3] = DYNAMIXEL_CT_RAM_PresentPosition;
  data[4] = 0x08; //return data length
  data[5] = CheckSUM(data, data[1]+1);
  Send_Data(data, sizeof(data));
  Receive_Data(servo, data);
}