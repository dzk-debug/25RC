#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "remote_control.h"
#include "user_lib.h"

#define GIMBAL_CONTROL_TIME 2
#define GIMBAL_TASK_INIT_TIME 3500//2

//传球准备电机 速度环 
#define Pass_SPEED_KP 8.0f//0.12f//
#define Pass_SPEED_KI 0.0f
#define Pass_SPEED_KD 0.2f
#define Pass_SPEED_I_LIMIT 10000.0f
#define Pass_SPEED_MAX  10000.0f
static  fp32 PID_Pass_SPEED[] = {Pass_SPEED_KP, Pass_SPEED_KI, Pass_SPEED_KD};

//传球准备电机 角度环 
#define Pass_ANGLE_KP 0.5f
#define Pass_ANGLE_KI 0.0f
#define Pass_ANGLE_KD 10.0f
#define Pass_ANGLE_I_LIMIT 10000.0f
#define Pass_ANGLE_MAX  10000.0f
static  fp32 PID_Pass_ANGLE[] = {Pass_ANGLE_KP, Pass_ANGLE_KI, Pass_ANGLE_KD};

//防守3508电机 速度环 
#define Fang_SPEED_KP 8.0f//0.12f//
#define Fang_SPEED_KI 0.0f
#define Fang_SPEED_KD 0.2f
#define Fang_SPEED_I_LIMIT 10000.0f
#define Fang_SPEED_MAX  10000.0f
static  fp32 PID_Fang_SPEED[] = {Fang_SPEED_KP, Fang_SPEED_KI, Fang_SPEED_KD};

//防守3508电机 角度环 
#define Fang_ANGLE_KP 0.5f
#define Fang_ANGLE_KI 0.0f
#define Fang_ANGLE_KD 10.0f
#define Fang_ANGLE_I_LIMIT 10000.0f
#define Fang_ANGLE_MAX  10000.0f
static  fp32 PID_Fang_ANGLE[] = {Fang_ANGLE_KP, Fang_ANGLE_KI, Fang_ANGLE_KD};
typedef struct
{
//const Remote_Info_Typedef *gimbal_rc_ctrl;  //王草凡版
const	Remote_Info_Typedef *gimbal_rc_ctrl;

	
}gimbal_control_t;
	
// 在全局或文件作用域定义状态机枚举和变量
typedef enum {
    PASS_READY_IDLE,
    PASS_READY_SET_3508,
    PASS_READY_WAIT_3508,  // 新增状态：等待3508到位
    PASS_READY_RELEASE_BALL,
    PASS_READY_SET_HAITAI
} pass_ready_state_t;
 typedef enum
{
LUP=0,
LMIDDLE,
LDOWN,	
}key_s1;


 typedef enum
{
RUP=0,
RMIDDLE,
RDOWN,	
}key_s2;









extern void Gimbal_Task(void const * argument);
void gimbal_auto_control();
void gimbal_manual_control();
#endif

