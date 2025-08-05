#include "HT_can_comm.h"
#include "fdcan.h"


#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


volatile float CurVelocity = 0;

//static void _CanFilter(void)
//{
//    CAN_FilterTypeDef   sCAN_Filter;
//    
//    sCAN_Filter.FilterBank = 0;                         /* ָ��������ʼ���Ĺ����� */  
//    sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* ����ģʽΪ����λģʽ */
//    sCAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT;    /* ָ���˲����Ĺ�ģ */
//    sCAN_Filter.FilterIdHigh = 00;
//    sCAN_Filter.FilterIdLow = 00;             
//    sCAN_Filter.FilterMaskIdHigh = 00;
//    sCAN_Filter.FilterMaskIdLow = 00;
//    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    sCAN_Filter.FilterActivation = ENABLE;              /* ���û���ù����� */
//    sCAN_Filter.SlaveStartFilterBank = 0;               /* ѡ�������ӹ������� */
//    
//    HAL_CAN_ConfigFilter(&hcan1, &sCAN_Filter);
//}

/**
  * @brief  CAN�ӿڳ�ʼ��
  * @param
  * @retval 
  */
//void CanComm_Init(void)
//{
//   _CanFilter();
//    HAL_CAN_Start(&hcan2);               /* ����CANͨ�� */  
//    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);    /* ���������ж����� */
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
  * @brief          ͨ��CAN1���Ͳ�������ȥ���ƹؽڵ��
  * @param[in]      buf: 8���ֽڵ����ݣ�����������Ʋ�����Ϣ
  * @param[in]      len: buf�ĳ���
  * @param[in]      motor_id: �����ID����0x01��0x04
  * @retval         none
  */
static void CanTransmit(uint8_t *buf, uint8_t len, uint32_t motor_id)
{
    FDCAN_TxHeaderTypeDef TxHead = {
        .Identifier = motor_id,        // ��׼ID
        .IdType = FDCAN_STANDARD_ID,   // ��׼��ʶ��
        .TxFrameType = FDCAN_DATA_FRAME, // ����֡
        .DataLength = FDCAN_DLC_BYTES_8, // ���ݳ��ȱ���
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,    // �ر������л�
        .FDFormat = FDCAN_CLASSIC_CAN,     // ����CAN��ʽ
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

    if((buf != NULL) && (len != 0))
    {
        // ʹ������ģʽ����
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
  * @brief          ���͹ؽڵ�����Ʋ���(0x01,0x02,0x03,0x04)
  * @param[in]      f_p: Ŀ��λ�ã���Χ [-95.5,95.5] rad
  * @param[in]      f_v: Ŀ��ת�٣���Χ [-45,45] rad/s
  * @param[in]      f_kp: kp������ ��Χ [0��500] N.m/rad
  * @param[in]      f_kd: kd����,  ��Χ [0,5] N.m/rad/s
  * @param[in]      f_t: Ŀ������, ��Χ [-18,18] N.m
  * @param[in]      motor_id: �����ID����0x01��0x04
  * @retval         none
  */
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t motor_id)
{
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    
    /* ��������Ĳ����ڶ���ķ�Χ�� */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* ����Э�飬��float��������ת�� */
    p = float_to_uint(f_p,   P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,   V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,   T_MIN,  T_MAX,  12);
    
    /* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;
    
    /* ͨ��CAN�ӿڰ�buf�е����ݷ��ͳ�ȥ */
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
        return; /* ֱ���˳����� */
    }
    CanTransmit(buf, sizeof(buf),motor_id);
}

float CanComm_GetCurVelocity(void)
{
    return CurVelocity;
}






