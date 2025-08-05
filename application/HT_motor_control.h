#ifndef __HT_MOTOR_CONTROL_H
#define __HT_MOTOR_CONTROL_H

#include "struct_typedef.h"
#include "stdbool.h"

typedef struct
{

	fp32 torque_set;            //关节电机力矩设定值
	fp32 p_set;									//关节电机位置设定值
	fp32 v_set;									//关节电机速度设定值
	fp32 kp_set;								//关节电机位置系数设定值
	fp32 kd_set;								//关节电机速度系数设定值

} joint_motor_t;


void MotorControl_Start(void);
void MotorControl_Stop(void);
bool MotorControl_PositionHandler(void);
bool MotorControl_velocityHandler(uint32_t motor_id);

#endif

