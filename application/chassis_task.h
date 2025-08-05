#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "can_receive.h"
#include "remote_control.h"
#include "pid.h"
#include "user_lib.h"

//ҡ������
#define CHASSIS_RC_DEADLINE 500

// ����������״̬��ȡ��
#define HALL_FL_ACTIVE()    (HAL_GPIO_ReadPin(HALL_FL_GPIO_Port, HALL_FL_Pin) == GPIO_PIN_SET)
#define HALL_FR_ACTIVE()    (HAL_GPIO_ReadPin(HALL_FR_GPIO_Port, HALL_FR_Pin) == GPIO_PIN_SET)
#define HALL_BL_ACTIVE()    (HAL_GPIO_ReadPin(HALL_BL_GPIO_Port, HALL_BL_Pin) == GPIO_PIN_SET)
#define HALL_BR_ACTIVE()    (HAL_GPIO_ReadPin(HALL_BR_GPIO_Port, HALL_BR_Pin) == GPIO_PIN_SET)
extern float Steer_FL_ANGLE;
extern float Steer_FR_ANGLE; 
extern float Steer_BL_ANGLE;
extern float Steer_BR_ANGLE;

// У׼����
#define CALIBRATION_SPEED        500     // У׼ת�٣�RPM��
#define CALIBRATION_TIMEOUT      5000    // У׼��ʱʱ�䣨ms��

#define CHASSIS_TASK_INIT_TIME  3500	//352
#define CHASSIS_CONTROL_TIME		2			//���������ʱ��

//������� �ٶȻ� LT1
#define CAR_LT1_KP  1.0f//1.0f
#define CAR_LT1_KI  0.005f//0.002f
#define CAR_LT1_KD  1.0f//2.0f
#define CAR_LT1_I_LIMIT 10000.0f
#define CAR_LT1_MAX 10000.0f
static  fp32 PID_CAR_LT1[] = {CAR_LT1_KP, CAR_LT1_KI, CAR_LT1_KD};

//������� �ٶȻ� LT2
#define CAR_LT2_KP 1.0f
#define CAR_LT2_KI 0.005f
#define CAR_LT2_KD 1.0f
#define CAR_LT2_I_LIMIT 10000.0f
#define CAR_LT2_MAX 10000.0f
static  fp32 PID_CAR_LT2[] = {CAR_LT2_KP, CAR_LT2_KI, CAR_LT2_KD};

//������� �ٶȻ� RT1
#define CAR_RT1_KP 1.0f
#define CAR_RT1_KI 0.005f
#define CAR_RT1_KD 1.0f
#define CAR_RT1_I_LIMIT 10000.0f
#define CAR_RT1_MAX 10000.0f
static  fp32 PID_CAR_RT1[] = {CAR_RT1_KP, CAR_RT1_KI, CAR_RT1_KD};

//������� �ٶȻ� RT2
#define CAR_RT2_KP 1.0f
#define CAR_RT2_KI 0.005f
#define CAR_RT2_KD 1.0f
#define CAR_RT2_I_LIMIT 10000.0f
#define CAR_RT2_MAX 10000.0f
static  fp32 PID_CAR_RT2[] = {CAR_RT2_KP, CAR_RT2_KI, CAR_RT2_KD};


//ת���� �ٶȻ� FL
#define Steer_SPEED_FL_KP  15.0f//3.0f
#define Steer_SPEED_FL_KI  0.08f
#define Steer_SPEED_FL_KD  15.0f
#define Steer_SPEED_FL_I_LIMIT 10000.0f
#define Steer_SPEED_FL_MAX 	13000.0f //2000.0f//14000.0f
static  fp32 PID_Steer_SPEED_FL[] = {Steer_SPEED_FL_KP, Steer_SPEED_FL_KI, Steer_SPEED_FL_KD};

//ת���� �ǶȻ� FL
#define Steer_ANGLE_FL_KP 0.074f// 0.015f//0.12f//
#define Steer_ANGLE_FL_KI 0.0f
#define Steer_ANGLE_FL_KD 0.0f
#define Steer_ANGLE_FL_I_LIMIT 10000.0f
#define Steer_ANGLE_FL_MAX  10000.0f
static  fp32 PID_Steer_ANGLE_FL[] = {Steer_ANGLE_FL_KP, Steer_ANGLE_FL_KI, Steer_ANGLE_FL_KD};



//ת���� �ٶȻ� FR
#define Steer_SPEED_FR_KP 15.0f
#define Steer_SPEED_FR_KI 0.08f
#define Steer_SPEED_FR_KD 15.0f
#define Steer_SPEED_FR_I_LIMIT 10000.0f
#define Steer_SPEED_FR_MAX 13000.0f//14000.0f
static  fp32 PID_Steer_SPEED_FR[] = {Steer_SPEED_FR_KP, Steer_SPEED_FR_KI, Steer_SPEED_FR_KD};

//ת���� �ǶȻ� FR
#define Steer_ANGLE_FR_KP 0.074f
#define Steer_ANGLE_FR_KI 0.0f
#define Steer_ANGLE_FR_KD 0.09f
#define Steer_ANGLE_FR_I_LIMIT 10000.0f
#define Steer_ANGLE_FR_MAX 10000.0f
static  fp32 PID_Steer_ANGLE_FR[] = {Steer_ANGLE_FR_KP, Steer_ANGLE_FR_KI, Steer_ANGLE_FR_KD};

//ת���� �ٶȻ� BL
#define Steer_SPEED_BL_KP 15.0f
#define Steer_SPEED_BL_KI 0.08f
#define Steer_SPEED_BL_KD 15.0f//30.0f
#define Steer_SPEED_BL_I_LIMIT 10000.0f
#define Steer_SPEED_BL_MAX 13000.0f//14000.0f
static  fp32 PID_Steer_SPEED_BL[] = {Steer_SPEED_BL_KP, Steer_SPEED_BL_KI, Steer_SPEED_BL_KD};

//ת���� �ǶȻ� BL
#define Steer_ANGLE_BL_KP 0.074f
#define Steer_ANGLE_BL_KI 0.0f
#define Steer_ANGLE_BL_KD 0.09f
#define Steer_ANGLE_BL_I_LIMIT 10000.0f
#define Steer_ANGLE_BL_MAX 10000.0f
static  fp32 PID_Steer_ANGLE_BL[] = {Steer_ANGLE_BL_KP, Steer_ANGLE_BL_KI, Steer_ANGLE_BL_KD};

//ת���� �ٶȻ� BR
#define Steer_SPEED_BR_KP 15.0f
#define Steer_SPEED_BR_KI 0.08f
#define Steer_SPEED_BR_KD 15.0f//30.0f
#define Steer_SPEED_BR_I_LIMIT 10000.0f
#define Steer_SPEED_BR_MAX 13000.0f//14000.f
static  fp32 PID_Steer_SPEED_BR[] = {Steer_SPEED_BR_KP, Steer_SPEED_BR_KI, Steer_SPEED_BR_KD};

//ת���� �ǶȻ� BR
#define Steer_ANGLE_BR_KP 0.074f
#define Steer_ANGLE_BR_KI 0.0//0.00000001f
#define Steer_ANGLE_BR_KD 0.1f
#define Steer_ANGLE_BR_I_LIMIT 10000.0f
#define Steer_ANGLE_BR_MAX 10000.0f
static  fp32 PID_Steer_ANGLE_BR[] = {Steer_ANGLE_BR_KP, Steer_ANGLE_BR_KI, Steer_ANGLE_BR_KD};




// ��main.c�����VOFA���ͺ���������ԭ�д��룩
typedef struct {
    float target;    // Ŀ��Ƕ�
    float actual;    // ʵ�ʽǶ�
    float speed_out; // �ٶȻ����
    float curr_out;  // ���������
	  float angle_out;	//�ǶȻ����
		float rpm;
} PID_DebugData;


// У׼״̬��
typedef enum {
    CALIB_IDLE,
    CALIB_FL_SEARCH,
    CALIB_FR_SEARCH,
    CALIB_BL_SEARCH,
    CALIB_BR_SEARCH,
    CALIB_COMPLETE
} CalibState_t;

typedef struct {
    fp32 yaw_target;       // Ŀ��ƫ����
    uint8_t manual_rotate; // �ֶ���ת��־λ
} imu_correction_t;

extern imu_correction_t imu_correction;
extern pid_type_def pid_yaw_correction;
















//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
//#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode,
//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//rocker value (max 660) change to horizontal speed (m/s)
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f
//in following yaw angle mode, rocker value add to angle 
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f//CHASSIS_CONTROL_TIME/(CHASSIS_CONTROL_TIME+CHASSIS_ACCEL_X_NUM)Ϊ�˲�ϵ��������numԽС����ӦԽ��
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

////rocker value deadline
////ҡ������
//#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f


#define MOTOR_DISTANCE_TO_CENTER 0.22f

//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s
//#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//press the key, chassis will swing
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D
#define CHASSIS_GYRO_KEY KEY_PRESSED_OFFSET_SHIFT
#define CHASSIS_servo_KEY KEY_PRESSED_OFFSET_G            




//m3508 rmp change to chassis speed,
//m3508ת���ɵ����ٶ�(m/s)�ı�����
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

#define CHASSIS_WZ_SET_SCALE 0.0f///������Ϊ0

//when chassis is not set to move, swing max angle
//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f///15000
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 20.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//int32_t current_wz;
////����pid
//const fp32 chassis_0speed_pid[3] = {12, 0, 0};
//const fp32 chassis_1speed_pid[3] = {12, 0, 0};
//const fp32 chassis_2speed_pid[3] = {12, 0, 0};
//const fp32 chassis_3speed_pid[3] = {12, 0, 0};

//const fp32 chassis_wz_pid[3] = {3, 0, 2};
//int16_t  target_3508_0speed,target_3508_1speed, target_3508_2speed, target_3508_3speed;//�����ĸ����Ŀ��ת��
//int16_t chassis_wz_buf[600] = {0};//������һ������
//int16_t chassis_vx,chassis_vy,chassis_wz;

//pid_type_def PID_3508_0speed, PID_3508_1speed, PID_3508_2speed, PID_3508_3speed;//����PID������ʼ��
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
  //const Remote_Info_Typedef *chassis_RC;      //���ݷ���         //����ʹ�õ�ң����ָ��, the point to remote control
  const Remote_Info_Typedef *chassis_RC;
	const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��

  chassis_motor_t chassis_motor[8];          //chassis motor data.���̵������
  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�
	fp32 vx_set_channel,vy_set_channel,wz_set_channel;
} chassis_move_t;

//dzk 4.20
// ȫ�ֱ�������
typedef struct {
    int32_t total_ecd;    // �ۼƱ�����ֵ����Ȧ��
    uint16_t last_ecd;    // �ϴα�����ֵ
    int16_t turns;        // Ȧ��������
} Encoder_State_t;



//////����һ�׵�ͨ�˲�

//double electric_alpha = 0.1f;  // �˲�ϵ��
//double electric_y = 0.0f;      // �˲��������źţ���ʼֵ��
//double electric_x = 0.0f;      // �����źţ�������������
//double electric_out;

//double electric_alpha1 = 0.1f;  // �˲�ϵ��
//double electric_y1 = 0.0f;      // �˲��������źţ���ʼֵ��
//double electric_x1 = 0.0f;      // �����źţ�������������
//double electric_out1;


//extern void pid_chassis_all_init(void);
//extern void chassis_target_calc();
//extern void chassis_calc_cmd(); //����PID���㼰�������

extern void Chassis_Task(void const * argument);





#endif
