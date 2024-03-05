#include "feet_motor.h"

FDCAN_TxHeaderTypeDef  can_tx_message;
uint8_t              MotorSendBuffer[8];

feet_motor_measure_t LeftFootMotorMeasure;
feet_motor_measure_t RightFootMotorMeasure;

void FEET_CONTROL(int16_t FEET_MOTOR_LEFT, int16_t FEET_MOTOR_RIGHT)
{
		
    MotorSendBuffer[(FEET_MOTOR1_TRANSMIT_ID - 0x281)*2 ]               =   FEET_MOTOR_LEFT >> 8;
    MotorSendBuffer[(FEET_MOTOR1_TRANSMIT_ID - 0x281)*2 + 1]        		    =   FEET_MOTOR_LEFT;
    MotorSendBuffer[(FEET_MOTOR2_TRANSMIT_ID - 0x281)*2]             =   FEET_MOTOR_RIGHT >> 8;
    MotorSendBuffer[(FEET_MOTOR2_TRANSMIT_ID - 0x281)*2 + 1]        		    =   FEET_MOTOR_RIGHT;
  

    can_tx_message.Identifier=0x200;                           //32位ID
    can_tx_message.IdType=FDCAN_STANDARD_ID;                  //标准ID
    can_tx_message.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    can_tx_message.DataLength=FDCAN_DLC_BYTES_8;              //数据长度
    can_tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    can_tx_message.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    can_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    can_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    can_tx_message.MessageMarker=0;      

    
    /* 添加数据到TX FIFO */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_message, MotorSendBuffer);
		
		
		
} 



