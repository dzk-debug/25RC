#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "can_receive.h"
#include "remote_control.h"
#include "pid.h"
#include "user_lib.h"

//摇杆死区
#define CHASSIS_RC_DEADLINE 500

// 霍尔传感器状态读取宏
#define HALL_FL_ACTIVE()    (HAL_GPIO_ReadPin(HALL_FL_GPIO_Port, HALL_FL_Pin) == GPIO_PIN_SET)
#define HALL_FR_ACTIVE()    (HAL_GPIO_ReadPin(HALL_FR_GPIO_Port, HALL_FR_Pin) == GPIO_PIN_SET)
#define HALL_BL_ACTIVE()    (HAL_GPIO_ReadPin(HALL_BL_GPIO_Port, HALL_BL_Pin) == GPIO_PIN_SET)
#define HALL_BR_ACTIVE()    (HAL_GPIO_ReadPin(HALL_BR_GPIO_Port, HALL_BR_Pin) == GPIO_PIN_SET)
extern float Steer_FL_ANGLE;
extern float Steer_FR_ANGLE; 
extern float Steer_BL_ANGLE;
extern float Steer_BR_ANGLE;

// 校准参数
#define CALIBRATION_SPEED        500     // 校准转速（RPM）
#define CALIBRATION_TIMEOUT      5000    // 校准超时时间（ms）

#define CHASSIS_TASK_INIT_TIME  3500	//352
#define CHASSIS_CONTROL_TIME		2			//射击任务间隔时间

//驱动电机 速度环 LT1
#define CAR_LT1_KP  1.0f//1.0f
#define CAR_LT1_KI  0.005f//0.002f
#define CAR_LT1_KD  1.0f//2.0f
#define CAR_LT1_I_LIMIT 10000.0f
#define CAR_LT1_MAX 10000.0f
static  fp32 PID_CAR_LT1[] = {CAR_LT1_KP, CAR_LT1_KI, CAR_LT1_KD};

//驱动电机 速度环 LT2
#define CAR_LT2_KP 1.0f
#define CAR_LT2_KI 0.005f
#define CAR_LT2_KD 1.0f
#define CAR_LT2_I_LIMIT 10000.0f
#define CAR_LT2_MAX 10000.0f
static  fp32 PID_CAR_LT2[] = {CAR_LT2_KP, CAR_LT2_KI, CAR_LT2_KD};

//驱动电机 速度环 RT1
#define CAR_RT1_KP 1.0f
#define CAR_RT1_KI 0.005f
#define CAR_RT1_KD 1.0f
#define CAR_RT1_I_LIMIT 10000.0f
#define CAR_RT1_MAX 10000.0f
static  fp32 PID_CAR_RT1[] = {CAR_RT1_KP, CAR_RT1_KI, CAR_RT1_KD};

//驱动电机 速度环 RT2
#define CAR_RT2_KP 1.0f
#define CAR_RT2_KI 0.005f
#define CAR_RT2_KD 1.0f
#define CAR_RT2_I_LIMIT 10000.0f
#define CAR_RT2_MAX 10000.0f
static  fp32 PID_CAR_RT2[] = {CAR_RT2_KP, CAR_RT2_KI, CAR_RT2_KD};


//转向电机 速度环 FL
#define Steer_SPEED_FL_KP  15.0f//3.0f
#define Steer_SPEED_FL_KI  0.08f
#define Steer_SPEED_FL_KD  15.0f
#define Steer_SPEED_FL_I_LIMIT 10000.0f
#define Steer_SPEED_FL_MAX 	13000.0f //2000.0f//14000.0f
static  fp32 PID_Steer_SPEED_FL[] = {Steer_SPEED_FL_KP, Steer_SPEED_FL_KI, Steer_SPEED_FL_KD};

//转向电机 角度环 FL
#define Steer_ANGLE_FL_KP 0.074f// 0.015f//0.12f//
#define Steer_ANGLE_FL_KI 0.0f
#define Steer_ANGLE_FL_KD 0.0f
#define Steer_ANGLE_FL_I_LIMIT 10000.0f
#define Steer_ANGLE_FL_MAX  10000.0f
static  fp32 PID_Steer_ANGLE_FL[] = {Steer_ANGLE_FL_KP, Steer_ANGLE_FL_KI, Steer_ANGLE_FL_KD};



//转向电机 速度环 FR
#define Steer_SPEED_FR_KP 15.0f
#define Steer_SPEED_FR_KI 0.08f
#define Steer_SPEED_FR_KD 15.0f
#define Steer_SPEED_FR_I_LIMIT 10000.0f
#define Steer_SPEED_FR_MAX 13000.0f//14000.0f
static  fp32 PID_Steer_SPEED_FR[] = {Steer_SPEED_FR_KP, Steer_SPEED_FR_KI, Steer_SPEED_FR_KD};

//转向电机 角度环 FR
#define Steer_ANGLE_FR_KP 0.074f
#define Steer_ANGLE_FR_KI 0.0f
#define Steer_ANGLE_FR_KD 0.09f
#define Steer_ANGLE_FR_I_LIMIT 10000.0f
#define Steer_ANGLE_FR_MAX 10000.0f
static  fp32 PID_Steer_ANGLE_FR[] = {Steer_ANGLE_FR_KP, Steer_ANGLE_FR_KI, Steer_ANGLE_FR_KD};

//转向电机 速度环 BL
#define Steer_SPEED_BL_KP 15.0f
#define Steer_SPEED_BL_KI 0.08f
#define Steer_SPEED_BL_KD 15.0f//30.0f
#define Steer_SPEED_BL_I_LIMIT 10000.0f
#define Steer_SPEED_BL_MAX 13000.0f//14000.0f
static  fp32 PID_Steer_SPEED_BL[] = {Steer_SPEED_BL_KP, Steer_SPEED_BL_KI, Steer_SPEED_BL_KD};

//转向电机 角度环 BL
#define Steer_ANGLE_BL_KP 0.074f
#define Steer_ANGLE_BL_KI 0.0f
#define Steer_ANGLE_BL_KD 0.09f
#define Steer_ANGLE_BL_I_LIMIT 10000.0f
#define Steer_ANGLE_BL_MAX 10000.0f
static  fp32 PID_Steer_ANGLE_BL[] = {Steer_ANGLE_BL_KP, Steer_ANGLE_BL_KI, Steer_ANGLE_BL_KD};

//转向电机 速度环 BR
#define Steer_SPEED_BR_KP 15.0f
#define Steer_SPEED_BR_KI 0.08f
#define Steer_SPEED_BR_KD 15.0f//30.0f
#define Steer_SPEED_BR_I_LIMIT 10000.0f
#define Steer_SPEED_BR_MAX 13000.0f//14000.f
static  fp32 PID_Steer_SPEED_BR[] = {Steer_SPEED_BR_KP, Steer_SPEED_BR_KI, Steer_SPEED_BR_KD};

//转向电机 角度环 BR
#define Steer_ANGLE_BR_KP 0.074f
#define Steer_ANGLE_BR_KI 0.0//0.00000001f
#define Steer_ANGLE_BR_KD 0.1f
#define Steer_ANGLE_BR_I_LIMIT 10000.0f
#define Steer_ANGLE_BR_MAX 10000.0f
static  fp32 PID_Steer_ANGLE_BR[] = {Steer_ANGLE_BR_KP, Steer_ANGLE_BR_KI, Steer_ANGLE_BR_KD};




// 在main.c中添加VOFA发送函数（基于原有代码）
typedef struct {
    float target;    // 目标角度
    float actual;    // 实际角度
    float speed_out; // 速度环输出
    float curr_out;  // 电流环输出
	  float angle_out;	//角度环输出
		float rpm;
} PID_DebugData;


// 校准状态机
typedef enum {
    CALIB_IDLE,
    CALIB_FL_SEARCH,
    CALIB_FR_SEARCH,
    CALIB_BL_SEARCH,
    CALIB_BR_SEARCH,
    CALIB_COMPLETE
} CalibState_t;

typedef struct {
    fp32 yaw_target;       // 目标偏航角
    uint8_t manual_rotate; // 手动旋转标志位
} imu_correction_t;

extern imu_correction_t imu_correction;
extern pid_type_def pid_yaw_correction;
















//in the beginning of task ,wait a time
//任务开始空闲一段时间
//#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode,
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//in following yaw angle mode, rocker value add to angle 
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f//CHASSIS_CONTROL_TIME/(CHASSIS_CONTROL_TIME+CHASSIS_ACCEL_X_NUM)为滤波系数，愈大（num越小）响应越快
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

////rocker value deadline
////摇杆死区
//#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.22f

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
//#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//press the key, chassis will swing
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_GYRO_KEY KEY_PRESSED_OFFSET_SHIFT
#define CHASSIS_servo_KEY KEY_PRESSED_OFFSET_G            




//m3508 rmp change to chassis speed,
//m3508转化成底盘速度(m/s)的比例，
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.0f///在中心为0

//when chassis is not set to move, swing max angle
//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f///15000
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 20.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//int32_t current_wz;
////轮组pid
//const fp32 chassis_0speed_pid[3] = {12, 0, 0};
//const fp32 chassis_1speed_pid[3] = {12, 0, 0};
//const fp32 chassis_2speed_pid[3] = {12, 0, 0};
//const fp32 chassis_3speed_pid[3] = {12, 0, 0};

//const fp32 chassis_wz_pid[3] = {3, 0, 2};
//int16_t  target_3508_0speed,target_3508_1speed, target_3508_2speed, target_3508_3speed;//底盘四个电机目标转速
//int16_t chassis_wz_buf[600] = {0};//保存上一秒数据
//int16_t chassis_vx,chassis_vy,chassis_wz;

//pid_type_def PID_3508_0speed, PID_3508_1speed, PID_3508_2speed, PID_3508_3speed;//底盘PID参数初始化
//pid_type_def PID_chassis_wz;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;

} chassis_motor_t;

typedef struct
{
  //const Remote_Info_Typedef *chassis_RC;      //王草凡版         //底盘使用的遥控器指针, the point to remote control
  const Remote_Info_Typedef *chassis_RC;
	const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针

  chassis_motor_t chassis_motor[8];          //chassis motor data.底盘电机数据
  pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
  fp32 chassis_relative_angle_set;  //the set relative angle.设置相对云台控制角度
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
  fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的yaw角度
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的pitch角度
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的roll角度
	fp32 vx_set_channel,vy_set_channel,wz_set_channel;
} chassis_move_t;

//dzk 4.20
// 全局变量声明
typedef struct {
    int32_t total_ecd;    // 累计编码器值（多圈）
    uint16_t last_ecd;    // 上次编码器值
    int16_t turns;        // 圈数计数器
} Encoder_State_t;



//////尝试一阶低通滤波

//double electric_alpha = 0.1f;  // 滤波系数
//double electric_y = 0.0f;      // 滤波后的输出信号（初始值）
//double electric_x = 0.0f;      // 输入信号（传感器反馈）
//double electric_out;

//double electric_alpha1 = 0.1f;  // 滤波系数
//double electric_y1 = 0.0f;      // 滤波后的输出信号（初始值）
//double electric_x1 = 0.0f;      // 输入信号（传感器反馈）
//double electric_out1;


//extern void pid_chassis_all_init(void);
//extern void chassis_target_calc();
//extern void chassis_calc_cmd(); //底盘PID计算及输出后函数

extern void Chassis_Task(void const * argument);





#endif
