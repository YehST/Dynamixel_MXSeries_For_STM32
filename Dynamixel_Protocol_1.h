
#ifndef DYNAMIXEL_PROTOCOL_1_H
#define DYNAMIXEL_PROTOCOL_1_H

#ifdef __cplusplus
extern "C" {
#endif

/* user type structure----------------------- */
typedef union{
    uint8_t Error;
    struct {
        uint8_t InputVoltage : 1;
        uint8_t AngleLimit : 1;
        uint8_t OverHeating : 1;
        uint8_t Range : 1;
        uint8_t CheakSum : 1;
        uint8_t OverLoad : 1;
        uint8_t Instruction : 1;
    } ErrorType;
} SERVO_Error;

typedef struct{
    float Present_Position;
    int Raw_Encoder;
    float Present_Speed;
    float Present_Load;
    float Present_voltage;
    int8_t Present_Temperature;
} SERVO_State;

typedef struct{
    int8_t Kp;
    int8_t Ki;
    int8_t Kd;
} SERVO_Controller;

typedef struct{
    uint16_t CW_angle_limit;
    uint16_t CCW_angle_limit;
    uint8_t Temperature_limit;
    uint8_t Min_Volt_limit;
    uint8_t Max_Volt_limit;
    uint16_t Torque_limit;
} SERVO_Constrain;

typedef enum {
    MX_Wheel_Mode = 1,
    MX_Joint_Mode = 2,
    MX_MultiTurn_Mode = 3,
} SERVO_Mode;

typedef struct{
    SERVO_Mode Mode;
    /*
        0x01: Wheel Mode
        0x02: Joint Mode
        0x03: Multi-turn Mode
    */
    uint8_t ID;
    SERVO_Error Error;
    SERVO_State state;
    SERVO_Controller controller;
    int8_t ModelNumber;
    uint8_t FirmVersion;
    SERVO_Constrain Limit;
    uint8_t StatusReturnLevel;
    float MultiTurnOffset;
    int16_t RealTime_Tick;
} Dynamixel_SERVO;
/* User private variables -------------------------------------- */


/* User private defined -------------------------------------- */
#define DYNAMIXEL_INST_Ping 0x01
#define DYNAMIXEL_INST_Read 0x02
#define DYNAMIXEL_INST_Write 0x03
#define DYNAMIXEL_INST_RegWrite 0x04
#define DYNAMIXEL_INST_Action 0x05
#define DYNAMIXEL_INST_FactoryReset 0x06
#define DYNAMIXEL_INST_Reboot 0x08
#define DYNAMIXEL_INST_SyncWrite 0x83
#define DYNAMIXEL_INST_BulkRead 0x92

// Control Table --------------------------------
#define DYNAMIXEL_CT_EEPROM_ModelNumber 0x00 // R
#define DYNAMIXEL_CT_EEPROM_FirmVersion 0x02 // R
#define DYNAMIXEL_CT_EEPROM_ID 0x03 // RW
#define DYNAMIXEL_CT_EEPROM_BaudRate 0x04 // RW
#define DYNAMIXEL_CT_EEPROM_ReturnDelay 0x05 // RW
#define DYNAMIXEL_CT_EEPROM_CWAngleLimit 0x06 // RW
#define DYNAMIXEL_CT_EEPROM_CCWAngleLimit 0x08 // RW
#define DYNAMIXEL_CT_EEPROM_TempLimit 0x0B // RW
#define DYNAMIXEL_CT_EEPROM_MinVoltLimit 0x0C // RW
#define DYNAMIXEL_CT_EEPROM_MaxVoltLimit 0x0D // RW
#define DYNAMIXEL_CT_EEPROM_MaxTorque 0x0E // RW
#define DYNAMIXEL_CT_EEPROM_StatusReturnLevel 0x10 // RW
#define DYNAMIXEL_CT_EEPROM_AlarmLED 0x11 // RW
#define DYNAMIXEL_CT_EEPROM_ShutDown 0x12 // RW
#define DYNAMIXEL_CT_EEPROM_MultiTurnOffset 0x14 // RW
#define DYNAMIXEL_CT_EEPROM_ResolutionDivider 0x16 // RW

#define DYNAMIXEL_CT_RAM_TorqueEnable 0x18 // RW
#define DYNAMIXEL_CT_RAM_LED 0x19 // RW
#define DYNAMIXEL_CT_RAM_D_Gain 0x1A // RW
#define DYNAMIXEL_CT_RAM_I_Gain 0x1B // RW
#define DYNAMIXEL_CT_RAM_P_Gain 0x1C // RW
#define DYNAMIXEL_CT_RAM_GoalPosition 0x1E // RW
#define DYNAMIXEL_CT_RAM_MovingSpeed 0x20 // RW
#define DYNAMIXEL_CT_RAM_TorqueLimit 0x22 // RW
#define DYNAMIXEL_CT_RAM_PresentPosition 0x24 // R
#define DYNAMIXEL_CT_RAM_PresentSpeed 0x26 // R
#define DYNAMIXEL_CT_RAM_PresentLoad 0x28 // R
#define DYNAMIXEL_CT_RAM_PresentVoltage 0x2A // R
#define DYNAMIXEL_CT_RAM_PresentTemperature 0x2B // R
#define DYNAMIXEL_READ_ALL_PresentData 0xAA
#define DYNAMIXEL_CT_RAM_Registered 0x2C // R
#define DYNAMIXEL_CT_RAM_Moving 0x2E // R
#define DYNAMIXEL_CT_RAM_Lock 0x2F // RW
#define DYNAMIXEL_CT_RAM_Punch 0x30 // RW
#define DYNAMIXEL_CT_RAM_RT_Tick 0x32 // R
#define DYNAMIXEL_CT_RAM_GoalAcc 0x49 // RW

/* User private function prototypes -----------------------------------------------*/
void Dynamixel_Init_Handle(UART_HandleTypeDef* huart);
void Dynamixel_Servo_Init(Dynamixel_SERVO* Allservo, int8_t NumOfServo);
static void Errorchk(Dynamixel_SERVO* servo);
static uint8_t CheckSUM(uint8_t* data, int8_t len);
static void Send_Data(uint8_t* data, int8_t len);
static void Receive_Data(Dynamixel_SERVO* servo, uint8_t* TxData);
static void Decode_Data(Dynamixel_SERVO* servo, uint8_t CMD, uint8_t* RxData, int8_t len);

void Dynamixel_PING(Dynamixel_SERVO* servo);
void Dynamixel_R_Model(Dynamixel_SERVO* servo);
void Dynamixel_W_ID(Dynamixel_SERVO* servo, uint8_t ID);
void Dynamixel_W_BaudRate(Dynamixel_SERVO* servo, int BaudRate);
void Dynamixel_W_ReturnDelay(Dynamixel_SERVO* servo, int8_t ReturnDelay_us);
void Dynamixel_W_CWAngleLimit(Dynamixel_SERVO* servo, float Pos);
void Dynamixel_W_CCWAngleLimit(Dynamixel_SERVO* servo, float Pos);
void Dynamixel_W_MinVoltageLimit(Dynamixel_SERVO* servo, float MinVotalge);
void Dynamixel_W_MaxVoltageLimit(Dynamixel_SERVO* servo, float MaxVotalge);
void Dynamixel_W_MaxTorque(Dynamixel_SERVO* servo, uint8_t percent);
void Dynamixel_R_AllLimit(Dynamixel_SERVO* servo);
void Dynamixel_W_StatusReturnLevel(Dynamixel_SERVO* servo, uint8_t Mode);
void Dynamixel_R_StatusReturnLevel(Dynamixel_SERVO* servo);
void Dynamixel_W_AlarmLED_ErrorType(Dynamixel_SERVO* servo, uint8_t ErrorType);
void Dynamixel_W_ShutDown_ErrorType(Dynamixel_SERVO* servo, uint8_t ErrorType);
void Dynamixel_R_AlarmErrorType(Dynamixel_SERVO* servo);
void Dynamixel_W_MultiTurnOffset(Dynamixel_SERVO* servo, float Offset);
void Dynamixel_R_MultiTurnOffset(Dynamixel_SERVO* servo);
void Dynamixel_W_ResolutionDivider(Dynamixel_SERVO* servo, uint8_t Divider);
void Dynamixel_R_ResolutionDivider(Dynamixel_SERVO* servo);

void Dynamixel_Select_Model(Dynamixel_SERVO* servo, SERVO_Mode Mode);
void Dynamixel_W_TorqueEnable(Dynamixel_SERVO* servo, uint8_t Enable);
void Dynamixel_W_LED(Dynamixel_SERVO* servo, uint8_t Enable);
void Dynamixel_W_PID(Dynamixel_SERVO* servo, uint8_t Kp, uint8_t Ki, uint8_t Kd);
void Dynamixel_R_PID(Dynamixel_SERVO* servo);
void Dynamixel_W_GoalPosition(Dynamixel_SERVO* servo, float Pos);
void Dynamixel_W_MovingSpeed(Dynamixel_SERVO* servo, float Speed);
void Dynamixel_R_PresentPos(Dynamixel_SERVO* servo);
void Dynamixel_R_PresentSpeed(Dynamixel_SERVO* servo);
void Dynamixel_R_PresentLoad(Dynamixel_SERVO* servo);
void Dynamixel_R_PresentVoltage(Dynamixel_SERVO* servo);
void Dynamixel_R_PresentTemperature(Dynamixel_SERVO* servo);
void Dynamixel_R_AllPresentData(Dynamixel_SERVO* servo);
void Dynamixel_R_Registered(Dynamixel_SERVO* servo);
void Dynamixel_W_Lock(Dynamixel_SERVO* servo, uint8_t lock);
void Dynamixel_R_Lock(Dynamixel_SERVO* servo);
void Dynamixel_W_Punch(Dynamixel_SERVO* servo, uint16_t Punch);
void Dynamixel_R_Punch(Dynamixel_SERVO* servo);
void Dynamixel_R_RealTimeTick(Dynamixel_SERVO* servo);
void Dynamixel_W_GoalAcc(Dynamixel_SERVO* servo, float ACC);

#ifdef __cplusplus
}
#endif

#endif
