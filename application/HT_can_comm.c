#include "HT_can_comm.h"
#include "fdcan.h"


#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


volatile float CurVelocity = 0;

//static void _CanFilter(void)
//{
//    CAN_FilterTypeDef   sCAN_Filter;
//    
//    sCAN_Filter.FilterBank = 0;                         /* 指定将被初始化的过滤器 */  
//    sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* 过滤模式为屏蔽位模式 */
//    sCAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT;    /* 指定滤波器的规模 */
//    sCAN_Filter.FilterIdHigh = 00;
//    sCAN_Filter.FilterIdLow = 00;             
//    sCAN_Filter.FilterMaskIdHigh = 00;
//    sCAN_Filter.FilterMaskIdLow = 00;
//    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    sCAN_Filter.FilterActivation = ENABLE;              /* 启用或禁用过滤器 */
//    sCAN_Filter.SlaveStartFilterBank = 0;               /* 选择启动从过滤器组 */
//    
//    HAL_CAN_ConfigFilter(&hcan1, &sCAN_Filter);
//}

/**
  * @brief  CAN接口初始化
  * @param
  * @retval 
  */
//void CanComm_Init(void)
//{
//   _CanFilter();
//    HAL_CAN_Start(&hcan2);               /* 开启CAN通信 */  
//    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);    /* 开启挂起中断允许 */
//}

/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval 
  */
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

/**
  * @brief          send parameter data from CAN1 to control joint motor
  * @param[in]      buf: 8 bytes data, including motor control parameter information
  * @param[in]      len: size of buf
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          通过CAN1发送参数数据去控制关节电机
  * @param[in]      buf: 8个字节的数据，包含电机控制参数信息
  * @param[in]      len: buf的长度
  * @param[in]      motor_id: 电机的ID，从0x01到0x04
  * @retval         none
  */
static void CanTransmit(uint8_t *buf, uint8_t len, uint32_t motor_id)
{
    FDCAN_TxHeaderTypeDef TxHead = {
        .Identifier = motor_id,        // 标准ID
        .IdType = FDCAN_STANDARD_ID,   // 标准标识符
        .TxFrameType = FDCAN_DATA_FRAME, // 数据帧
        .DataLength = FDCAN_DLC_BYTES_8, // 数据长度编码
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,    // 关闭速率切换
        .FDFormat = FDCAN_CLASSIC_CAN,     // 经典CAN格式
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

    if((buf != NULL) && (len != 0))
    {
        // 使用阻塞模式发送
        if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHead, buf) != HAL_OK)
        {
            Error_Handler();
        }
    }
}



/**
  * @brief          send control parameters of joint motor (0x01, 0x02, 0x03, 0x04)
  * @param[in]      f_p: position command, range [-95.5,95.5] rad
  * @param[in]      f_v: velocity command, range [-45,45] rad/s
  * @param[in]      f_kp: kp parameter, range [0,500] N.m/rad
  * @param[in]      f_kd: kd parameter, range [0,5] N.m/rad/s
  * @param[in]      f_t:  torque command,range [-18,18] N.m
  * @param[in]      motor_id: id of joint motor,0x01~0x04
  * @retval         none
  */
/**
  * @brief          发送关节电机控制参数(0x01,0x02,0x03,0x04)
  * @param[in]      f_p: 目标位置，范围 [-95.5,95.5] rad
  * @param[in]      f_v: 目标转速，范围 [-45,45] rad/s
  * @param[in]      f_kp: kp参数， 范围 [0，500] N.m/rad
  * @param[in]      f_kd: kd参数,  范围 [0,5] N.m/rad/s
  * @param[in]      f_t: 目标力矩, 范围 [-18,18] N.m
  * @param[in]      motor_id: 电机的ID，从0x01到0x04
  * @retval         none
  */
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,   P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,   V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,   T_MIN,  T_MAX,  12);
    
    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    /* 通过CAN接口把buf中的内容发送出去 */
    CanTransmit(buf, sizeof(buf),motor_id);
}

void CanComm_ControlCmd(uint8_t cmd,uint32_t motor_id)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
        case CMD_MOTOR_MODE:
            buf[7] = 0xFC;
            break;
        
        case CMD_RESET_MODE:
            buf[7] = 0xFD;
        break;
        
        case CMD_ZERO_POSITION:
            buf[7] = 0xFE;
        break;
        
        default:
        return; /* 直接退出函数 */
    }
    CanTransmit(buf, sizeof(buf),motor_id);
}

float CanComm_GetCurVelocity(void)
{
    return CurVelocity;
}






