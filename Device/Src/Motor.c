
#include "Motor.h"

DM_Motor_Info_Typedef DM_6220_Motor[4] = {
 
 [0] = {
   .Init = 0,
	 .DM_Motor_CANFrameInfo = {
		.TxStdId = 0x01, 
	  .RxStdId = 0x00, 
	 },
   
	 .Type = DM_6220,
 },
 [1] = {
   .Init = 0,
   .DM_Motor_CANFrameInfo = {
	  .TxStdId = 0x02, 
	  .RxStdId = 0x00, 
	 },
	 .Type = DM_6220,
 },
 [2] = {
   .Init = 0,
   .DM_Motor_CANFrameInfo = {
	  .TxStdId = 0x03, 
	  .RxStdId = 0x00, 
	 },
	 .Type = DM_6220,
 },
 [3] = {
   .Init = 0,
   .DM_Motor_CANFrameInfo = {
	  .TxStdId = 0x04, 
	  .RxStdId = 0x00, 
	 },
	 .Type = DM_6220,
 },

};

DM_Motor_Info_Typedef DM_6215_Motor[4] = {
 
 [0] = {
   .Init = 0,
	 .DM_Motor_CANFrameInfo = {
		.TxStdId = 0x01, 
	  .RxStdId = 0x00, 
	 },
   
	 .Type = DM_6215,
 },
 [1] = {
   .Init = 0,
   .DM_Motor_CANFrameInfo = {
	  .TxStdId = 0x02, 
	  .RxStdId = 0x00, 
	 },
	 .Type = DM_6215,
 },
 [2] = {
   .Init = 0,
   .DM_Motor_CANFrameInfo = {
	  .TxStdId = 0x03, 
	  .RxStdId = 0x00, 
	 },
	 .Type = DM_6215,
 },
 [3] = {
   .Init = 0,
   .DM_Motor_CANFrameInfo = {
	  .TxStdId = 0x04, 
	  .RxStdId = 0x00, 
	 },
	 .Type = DM_6215,
 },

};

DM_Motor_Control_Typedef DM_6220_Motor_Control[4];

static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float X_float, float X_min, float X_max, int bits){
    float span = X_max - X_min;
    float offset = X_min;
    return (int) ((X_float-offset)*((float)((1<<bits)-1))/span);
}


void DM_Motor_Enable(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor){

	 TxFrame->Header.Identifier = DM_Motor->DM_Motor_CANFrameInfo.TxStdId;
  	
	 TxFrame->Data[0] = 0xFF;
   TxFrame->Data[1] = 0xFF;
 	 TxFrame->Data[2] = 0xFF;
	 TxFrame->Data[3] = 0xFF;
	 TxFrame->Data[4] = 0xFF;
	 TxFrame->Data[5] = 0xFF;
	 TxFrame->Data[6] = 0xFF;
	 TxFrame->Data[7] = 0xFC;

  HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);

}

void DM_Motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *TxFrame,DM_Motor_Info_Typedef *DM_Motor,float Postion, float Velocity, float KP, float KD, float Torque){

   uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
   
	 float P_MAX,V_MAX,T_MAX;
	
   switch(DM_Motor->Type){
		 
		  case DM_6220 :
	       P_MAX = 3.141593f; V_MAX = 45.f; T_MAX = 10.f;
	    break;
      
			case DM_6215 :
	       P_MAX = 12.5f; V_MAX = 45.f; T_MAX = 10.f;
      break;
      
			default:
	    break;   
	}
	 
   Postion_Tmp  =  float_to_uint(Postion,-P_MAX,P_MAX,16) ;
   Velocity_Tmp =  float_to_uint(Velocity,-V_MAX,V_MAX,12);
   Torque_Tmp = float_to_uint(Torque,-T_MAX,T_MAX,12);
   KP_Tmp = float_to_uint(KP,0,500,12);
   KD_Tmp = float_to_uint(KD,0,5,12);

   TxFrame->Header.Identifier = DM_Motor->DM_Motor_CANFrameInfo.TxStdId;
 	 TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
	 TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
	 TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
	 TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	 TxFrame->Data[4] = (uint8_t)(KP_Tmp);
	 TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
	 TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	 TxFrame->Data[7] = (uint8_t)(Torque_Tmp);
  
    HAL_FDCAN_AddMessageToTxFifoQ(TxFrame->hcan,&TxFrame->Header,TxFrame->Data);

}

void DM_Motor_Info_Update(uint8_t *Data,DM_Motor_Info_Typedef *DM_Motor)
{
	
		
   float P_MAX,V_MAX,T_MAX;
	
   switch(DM_Motor->Type){
      
		  case DM_6220 :
	       P_MAX = 3.141593f; V_MAX = 45.f; T_MAX = 10.f;
	    break;
      
			case DM_6215 :
	       P_MAX = 12.5f; V_MAX = 45.f; T_MAX = 10.f;
      break;
			default:
	    break;   

   }
		
	  DM_Motor->Data.State = Data[0]>>4;
		DM_Motor->Data.P_int = ((uint16_t)(Data[1]) <<8) | ((uint16_t)(Data[2]));
		DM_Motor->Data.V_int = ((uint16_t)(Data[3]) <<4) | ((uint16_t)(Data[4])>>4);
		DM_Motor->Data.T_int = ((uint16_t)(Data[4]&0xF) <<8) | ((uint16_t)(Data[5]));
		DM_Motor->Data.Torque=  uint_to_float(DM_Motor->Data.T_int,-T_MAX,T_MAX,12);
		DM_Motor->Data.Position=uint_to_float(DM_Motor->Data.P_int,-P_MAX,P_MAX,16);
    DM_Motor->Data.Velocity=uint_to_float(DM_Motor->Data.V_int,-V_MAX,V_MAX,12);
    DM_Motor->Data.Temperature_MOS   = (float)(Data[6]);
		DM_Motor->Data.Temperature_Rotor = (float)(Data[7]);

	 if(DM_Motor->Init == 0){
	 
	   
			 DM_Motor->Init = 1;
				
				
			}

}
	 







