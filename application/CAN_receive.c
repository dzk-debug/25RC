/* fdcan_receive.c */
#include "can_receive.h"
#include "string.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern int clamp(int value);

static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval 
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
#define get_joint_motor_measure(ptr, data)                                                  \
{                                                                                       \
		(ptr)->motorid  = data[0];                                                          \
		(ptr)->position = uint_to_float(((data[1] << 8 ) | (data[2])),P_MIN,P_MAX,16);      \
		(ptr)->velocity = uint_to_float(((data[3] << 4 ) | (data[4]>>4)),V_MIN,V_MAX,12);   \
		(ptr)->current  = uint_to_float(((data[4] << 4 ) | (data[5])),T_MIN,T_MAX,12);      \
}		

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

fp32  M3508_currentpos[10] = {0};	//底盘0~109373.941176   227/17             传球电机0~155629  19/1
 motor_measure_t motor_chassis[15];
joint_motor_measure_t joint_motor[2];
static FDCAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static FDCAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static FDCAN_TxHeaderTypeDef  steer_tx_message;
static uint8_t              steer_can_send_data[8];

//static uint8_t              send_data1[2];		
//static uint8_t              send_data2[2];			
//static uint8_t              send_data3[2];	
//static uint8_t              send_data4[2];		

//static uint8_t              send_data11[2];		
//static uint8_t              send_data22[2];			
//static uint8_t              send_data33[2];	
//static uint8_t              send_data44[2];		
/* FDCAN接收回调函数 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
        
        if(hfdcan->Instance == FDCAN1)
        {
            switch (rx_header.Identifier)
            {
                case CAN_3508_M1_ID:
                case CAN_3508_M2_ID:
                case CAN_3508_M3_ID:
                case CAN_3508_M4_ID:
								case CAN_3508_T_ID:
								case CAN_3508_FANG_ID:	
//                case CAN_3508_S1_ID:
//                case CAN_3508_S2_ID:
//                case CAN_3508_S3_ID:
//								case CAN_3508_S4_ID:
                {
									
                    uint8_t i = rx_header.Identifier - CAN_3508_M1_ID;
									if(i<=3)
									{
                    get_motor_measure(&motor_chassis[i], rx_data);
										break;
									}
                  else 
									{
									  get_motor_measure(&motor_chassis[i+4], rx_data);
										motor_ecd_update(i+4);
										break;
									}										
                }
                default: break;
            }
					}
				
					
				else if(hfdcan->Instance == FDCAN2)
         {
            switch(rx_data[0])
            {
                case CAN_HT04_ID:
									
								
                {		
                    uint8_t i = rx_data[0] - CAN_HT04_ID ;
                    get_joint_motor_measure(&joint_motor[i], rx_data);
										
                    break;
                }
                default: break;
            } 
	
				}
				else if(hfdcan->Instance == FDCAN3)
         {
            switch(rx_header.Identifier)
            {
                case CAN_3508_S1_ID:
                case CAN_3508_S2_ID:
                case CAN_3508_S3_ID:
								case CAN_3508_S4_ID:
                {		
                    uint8_t i = rx_header.Identifier - CAN_3508_M1_ID ;
                    get_motor_measure(&motor_chassis[i], rx_data);
										if(i>=4&&i<=7)
										{motor_ecd_update(i);}
                    break;
                }
                default: break;
            } 
	
				}
					

    }
}

/* 发送云台电机命令 */
void FDCAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    gimbal_tx_message.Identifier = 0x1FF;
    gimbal_tx_message.IdType = FDCAN_STANDARD_ID;
    gimbal_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    gimbal_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    gimbal_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    gimbal_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    gimbal_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    gimbal_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &gimbal_tx_message, gimbal_can_send_data);

}

/* 发送底盘复位命令 */
void FDCAN_cmd_chassis_reset_ID(void)
{
    chassis_tx_message.Identifier = 0x700;
    chassis_tx_message.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    memset(chassis_can_send_data, 0, 8);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &chassis_tx_message, chassis_can_send_data);
}




void FDCAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
 
    chassis_tx_message.Identifier = 0x200;
    chassis_tx_message.IdType = FDCAN_STANDARD_ID;
    chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    chassis_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    chassis_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    chassis_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    chassis_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    chassis_can_send_data[0] = (uint8_t)(motor1 >> 8);
    chassis_can_send_data[1] = (uint8_t)motor1;
    chassis_can_send_data[2] = (uint8_t)(motor2 >> 8);
    chassis_can_send_data[3] = (uint8_t)motor2;
    chassis_can_send_data[4] = (uint8_t)(motor3 >> 8);
    chassis_can_send_data[5] = (uint8_t)motor3;
    chassis_can_send_data[6] = (uint8_t)(motor4 >> 8);
    chassis_can_send_data[7] = (uint8_t)motor4;

   HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &chassis_tx_message, chassis_can_send_data);


}

/* 以下获取数据指针函数保持不变 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void) { return &motor_chassis[4]; }
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void) { return &motor_chassis[5]; }
const motor_measure_t *get_trigger_motor_measure_point(void) { return &motor_chassis[6]; }
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i) { return &motor_chassis[(i & 0x03)]; }
const motor_measure_t *get_steer_motor_measure_point(uint8_t i) { return &motor_chassis[(i & 0x03)+6]; }



/* 发送转向电机命令 */
void FDCAN_cmd_steer(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    steer_tx_message.Identifier = 0x1FF;
    steer_tx_message.IdType = FDCAN_STANDARD_ID;
    steer_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    steer_tx_message.DataLength = FDCAN_DLC_BYTES_8;
    steer_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    steer_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    steer_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
    steer_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    steer_can_send_data[0] = (motor1 >> 8);
    steer_can_send_data[1] = motor1;
    steer_can_send_data[2] = (motor2 >> 8);
    steer_can_send_data[3] = motor2;
    steer_can_send_data[4] = (motor3 >> 8);
    steer_can_send_data[5] = motor3;
    steer_can_send_data[6] = (motor4 >> 8);
    steer_can_send_data[7] = motor4;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &steer_tx_message, steer_can_send_data);
}
// 在FDCAN_receive.c中新增处理函数
volatile uint8_t can_rx_buffer[32]; // 接收缓冲区

//void process_can_rx_buffers(void)
//{
//    // 快速拷贝数据（避免在中断中处理耗时操作）
//    __disable_irq();
//    memcpy(motor_chassis, can_rx_buffer, sizeof(motor_chassis));
//    __enable_irq();
//}
//dzk4.21
void motor_ecd_update(uint8_t i)
{
		
	if(motor_chassis[i].ecd - motor_chassis[i].last_ecd > 4096)//逆时针过零点
	{
		M3508_currentpos[i] += (motor_chassis[i].ecd - motor_chassis[i].last_ecd-8191);
	}
	else if(motor_chassis[i].ecd - motor_chassis[i].last_ecd < -4096)//顺时针过零点
	{
		M3508_currentpos[i] += (motor_chassis[i].ecd - motor_chassis[i].last_ecd+8191);
	}
	else//一圈之内
	{
		M3508_currentpos[i] += (motor_chassis[i].ecd - motor_chassis[i].last_ecd);
	}
	
//     // 限幅逻辑
//    if (M3508_currentpos[i] > 100000) {
//        M3508_currentpos[i] = 100000;
//    } else if (M3508_currentpos[i] < -100000) {
//        M3508_currentpos[i] = -100000;
//    }
	
}
//void motor_ecd_update(uint8_t j)
//{
//		
//	if(motor_chassis[j].ecd - motor_chassis[j].last_ecd > 4096)//逆时针过零点
//	{
//		M3508_currentpos[j-4] += (motor_chassis[j].ecd - motor_chassis[j].last_ecd-8191);
//	}
//	else if(motor_chassis[j].ecd - motor_chassis[j].last_ecd < -4096)//顺时针过零点
//	{
//		M3508_currentpos[j-4] += (motor_chassis[j].ecd - motor_chassis[j].last_ecd+8191);
//	}
//	else//一圈之内
//	{
//		M3508_currentpos[j-4] += (motor_chassis[j].ecd - motor_chassis[j].last_ecd);
//	}

//	
//}