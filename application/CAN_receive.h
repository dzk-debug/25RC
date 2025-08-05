/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       fdcan_receive.c/h
  * @brief      there is FDCAN interrupt function to receive motor data,
  *             and FDCAN send function to send motor current to control motor.
  *             这里是FDCAN中断接收函数，接收电机数据,FDCAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "main.h"

#define CHASSIS_FDCAN hfdcan1
#define GIMBAL_FDCAN  hfdcan2

/*HT定义关节电机的控制参数幅值*/
#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

/* FDCAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    //驱动电机3508，FDCAN1
	//0-3号电机motor_chassis[0-3]
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	//8号电机motor_chassis[8]
		CAN_3508_T_ID =0x205,
	//9号电机motor_chassis[9]
		CAN_3508_FANG_ID =0x206,
    //转向电机3508，FDCAN3
	//4-7号电机motor_chassis[4-7]
    CAN_3508_S1_ID = 0x205,
    CAN_3508_S2_ID = 0x206,
    CAN_3508_S3_ID = 0x207,
    CAN_3508_S4_ID = 0x208,
    //FDCAN2
	//独立接收joint_motor[]
    CAN_HT04_ID = 0x01,
		
	
	
    CAN_PIT_MOTOR_ID = 0x202,
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_EXTER_ALL_ID = 0x2FF,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern  motor_measure_t motor_chassis[15];
typedef struct
{
	  uint32_t motorid;        //电机ID值
    float position;          //转子位置，单位rad
    float velocity;          //转子速度，单位rad/s
    float current;           //电流值
} joint_motor_measure_t;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void FDCAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void FDCAN_cmd_steer(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          send FDCAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的FDCAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void FDCAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void FDCAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回转向电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[6,9]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_steer_motor_measure_point(uint8_t i);

/**
  * @brief          send external device control command
  * @param[in]      yaw: yaw axis control value
  * @param[in]      pitch: pitch axis control value
  * @param[in]      shoot: shoot control value
  * @param[in]      rev: reserve value
  * @retval         none
  */
extern void FDCAN_cmd_exter(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//dzk4.21
extern fp32 M3508_currentpos[10];	
extern void motor_ecd_update(uint8_t i);

#endif
