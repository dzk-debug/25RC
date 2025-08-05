#ifndef __HT_MOTOR_CONTROL_H
#define __HT_MOTOR_CONTROL_H

#include "struct_typedef.h"
#include "stdbool.h"

typedef struct
{

	fp32 torque_set;            //�ؽڵ�������趨ֵ
	fp32 p_set;									//�ؽڵ��λ���趨ֵ
	fp32 v_set;									//�ؽڵ���ٶ��趨ֵ
	fp32 kp_set;								//�ؽڵ��λ��ϵ���趨ֵ
	fp32 kd_set;								//�ؽڵ���ٶ�ϵ���趨ֵ

} joint_motor_t;


void MotorControl_Start(void);
void MotorControl_Stop(void);
bool MotorControl_PositionHandler(void);
bool MotorControl_velocityHandler(uint32_t motor_id);

#endif

