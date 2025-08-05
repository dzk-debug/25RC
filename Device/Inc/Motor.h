#ifndef Motor_H
#define Motor_H

#include "bsp_can.h"
#include "stdbool.h"

typedef enum{

  DM_4310,
  DM_6220,
  DM_6215,
  DM_Motor_Type_Num,

}DM_Motor_Type_e;


typedef struct 
{
  int16_t  State; 	/*!< Motor ERROR Message */
  uint16_t  P_int;
  uint16_t  V_int;
  uint16_t  T_int;
  float  Position;   /*!< Motor Positon */
  float  Velocity;   /*!< Motor Velocity  */
  float  Torque;  /*!< Motor Torque */
  float  Temperature_MOS;   /*!< Motor Temperature_MOS */
  float  Temperature_Rotor;   /*!< Motor Temperature_Rotor */
  float  Angle;	
}DM_Motor_GeneralInfo_Typedef;


typedef struct
{
  uint32_t TxStdId;   
  uint32_t RxStdId;  
}DM_Motor_CANFrameInfo_typedef;

typedef struct
{
	bool Init;
	uint16_t ID;
  DM_Motor_CANFrameInfo_typedef DM_Motor_CANFrameInfo;
	DM_Motor_Type_e Type;
	DM_Motor_GeneralInfo_Typedef Data;   /*!< information for the Motor Device */

}DM_Motor_Info_Typedef;

typedef struct
{
	float  KP;
	float  KD;
	float  Position;   /*!< Motor Positon */
    float  Velocity;   /*!< Motor Velocity  */
    float  Torque;  /*!< Motor Torque */
	
}DM_Motor_Control_Typedef;

extern DM_Motor_Info_Typedef DM_4310_Motor[3],DM_6220_Motor[4],DM_6215_Motor[4];

extern DM_Motor_Control_Typedef DM_6220_Motor_Control[4];

extern void DM_Motor_Enable(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor);

extern void DM_Motor_Info_Update(uint8_t *rxBuf,DM_Motor_Info_Typedef *DM_Motor);




extern void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque);


#endif