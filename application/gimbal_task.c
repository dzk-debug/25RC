#include "main.h"
#include "gimbal_task.h"
#include "HT_can_comm.h"
#include "HT_motor_control.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "pid.h"
#include "tim.h"
void gimbal_init();
static void gimbal_S1(void);
static void gimbal_S2(void);
static void gimbal_pat(void);
static void gimbal_pass_ready(void);
static void gimbal_dribble_ready(void);
static void gimbal_control_loop();
void sensor_detection();
static void pass_ready_state_handler(void);
extern uint8_t exit_flag;
extern uint8_t rising_falling_flag;
static uint8_t sensor_state=0;
pid_type_def pid_pass_motor_speed;
pid_type_def pid_pass_motor_angle;
pid_type_def pid_fang_motor_speed;
pid_type_def pid_fang_motor_angle;

fp32 PID_pass_angle_out,PID_pass_speed_out;
fp32 PID_fang_angle_out,PID_fang_speed_out;
static uint8_t last_s1=2;
static uint8_t last_s2=2;
static uint8_t last_ch4=0;
uint8_t current_s1=0;
uint8_t current_s2=0;
fp32 current_ch4=0;
fp32 gimbal_3508_set_ecd;
static fp32 gimbal_fang_set_ecd;
static pass_ready_state_t pass_ready_state = PASS_READY_IDLE;
static uint32_t release_ball_timestamp = 0;
//��̨���������������
gimbal_control_t gimbal_control;
joint_motor_t joint_control;

 void Gimbal_Task(void const * argument)
{
	
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	
	gimbal_init();
	for(;;)
	{ //ң�������ƹ���
		gimbal_auto_control();//�Զ����������У��пյ�����Ԥ����
		// ������׼��״̬��
		pass_ready_state_handler();
		//��̨����pid����
		gimbal_control_loop();
		//���Ϳ��Ƶ���
		CanComm_SendControlPara(joint_control.p_set,joint_control.v_set,joint_control.kp_set,joint_control.kd_set,joint_control.torque_set,CAN_HT04_ID);
		FDCAN_cmd_gimbal(PID_pass_speed_out,PID_fang_speed_out,0,0);
		vTaskDelay(GIMBAL_CONTROL_TIME);
	}
}


 void gimbal_init()
{
/*********pid��ʼ��************/
	PID_init(&pid_pass_motor_speed,PID_POSITION,PID_Pass_SPEED,Pass_SPEED_MAX,Pass_SPEED_I_LIMIT);
	PID_init(&pid_pass_motor_angle,PID_POSITION,PID_Pass_ANGLE,Pass_ANGLE_MAX,Pass_ANGLE_I_LIMIT);
	PID_init(&pid_fang_motor_speed,PID_POSITION,PID_Fang_SPEED,Fang_SPEED_MAX,Fang_SPEED_I_LIMIT);
	PID_init(&pid_fang_motor_angle,PID_POSITION,PID_Fang_ANGLE,Fang_ANGLE_MAX,Fang_ANGLE_I_LIMIT);
	gimbal_3508_set_ecd=0;
	gimbal_fang_set_ecd=0;
	M3508_currentpos[8]=0;
	M3508_currentpos[9]=0;
/********��̩�����ʼ��**********/
	MotorControl_Start();
	joint_control.p_set        =   0;
	joint_control.v_set        =   1;
	joint_control.kp_set       =   5;
  joint_control.kd_set       =   2;
	joint_control.torque_set   =   0;	
	
//	joint_control.p_set        =   -1.86;
//	joint_control.v_set        =   1;
//	joint_control.kp_set       =   5;
//  joint_control.kd_set       =   1.7;
//	joint_control.torque_set   =   0;	

//		joint_control.p_set        =   -1.99;
//	joint_control.v_set        =   0.5;
//	joint_control.kp_set       =   25;
//  joint_control.kd_set       =   5.0;
//	joint_control.torque_set   =   0;	

/*********IO�ڳ�ʼ��***********/
HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);//��
HAL_GPIO_WritePin(pass_GPIO_Port,pass_Pin,0);//����

	
 
		
}
static void gimbal_control_loop()
{
	PID_pass_angle_out=PID_calc(&pid_pass_motor_angle, M3508_currentpos[8],gimbal_3508_set_ecd);
	PID_pass_speed_out=PID_calc(&pid_pass_motor_speed,motor_chassis[8].speed_rpm,PID_pass_angle_out);
	
	PID_fang_angle_out=PID_calc(&pid_fang_motor_angle, M3508_currentpos[9],gimbal_fang_set_ecd);
	PID_fang_speed_out=PID_calc(&pid_fang_motor_speed,motor_chassis[9].speed_rpm,PID_fang_angle_out);
}

//��̨�Զ����ƺ���
static void gimbal_auto_control()
{	 current_s1=Remote_Ctrl.rc.s[1];
	 current_s2=Remote_Ctrl.rc.s[0];
	 current_ch4=Remote_Ctrl.rc.ch[4];
//�󲦸�
		if(current_s1!=last_s1)
	{
		last_s1=current_s1;
		
		if(current_s1 ==1)//S1��
		{
			HAL_GPIO_WritePin(pass_GPIO_Port,pass_Pin,1);//����
			osDelay(200);
			HAL_GPIO_WritePin(pass_GPIO_Port,pass_Pin,0);
			
		return;
		}
		else if(current_s1 ==3)//S1��
		{
			

		return;
		}
	  else	if(current_s1 ==2 && current_s2!=2)//S1��
		{

			
			gimbal_pass_ready();

	
			
			return;
		}
		else
		{
		return;
		}
	}
	//�Ҳ���
		if(current_s2!=last_s2)
	{
		last_s2=current_s2;
		if(current_s2 ==1)//S2��
		{
			gimbal_pat();
		return;
		}
		else if(current_s2 ==3)//S2��
		{

		return;
		}
	  else	if( current_s2 ==2 && current_s1 !=2)//S2��
		{
			gimbal_dribble_ready();

	
			return;
		}
		
		else
		{
		return;
		}

	}
	
			//����ң��

			if(Remote_Ctrl.rc.ch[1]<=-650)//ҡ������
			{
			gimbal_fang_set_ecd=-15000;
			}
			 else if(Remote_Ctrl.rc.ch[1]>650)//ҡ������
			{
			gimbal_fang_set_ecd=-655555;
			
			}
}

static void gimbal_pat(void)
{
	
	
						/***���ӽ�������****/
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);//��

//�Ƴ�
HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);//����
osDelay(16);
	
//osDelay(15);
HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);//��


//�س�
osDelay(150);
HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
//HAL_Delay(473);1.0
//osDelay(420);2.0
	osDelay(330);
HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);//��
//		uint8_t current_s2=Remote_Ctrl.rc.s[0];
//	
//	if(current_s2!=last_s2_state)
//	{
//		last_s2_state=current_s2;
//		
//		if(current_s2 ==1)//S2��
//		{
//		
//		return;
//		}
//		else if(current_s2 ==3)//S2��
//		{

//		return;
//		}
//	  else	if(current_s2 ==2)//S2��
//		{

//			return;
//		}
//		else
//		{
//		return;
//		}
//	}
	

	
	
	
	
	
//	/***********��̧��****************/
//	HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);
//	HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);
	
	/**********��̧��*************/
//	HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);
//	osDelay(500);
//	HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);
//	HAL_GPIO_WritePin(lifting_GPIO_Port,lifting_Pin,0);
//	HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);
//	if(sensor_state==1)
//	{
//		HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);
//		osDelay(300);
//		HAL_GPIO_WritePin(lifting_GPIO_Port,lifting_Pin,1);
//		
//	}

}


// ������״̬��������
static void pass_ready_state_handler(void)
{
    switch(pass_ready_state) {
        case PASS_READY_SET_3508:
            // ����3508Ŀ��λ��
            gimbal_3508_set_ecd = 19 * 2207.0194;
            pass_ready_state = PASS_READY_WAIT_3508;  // ����ȴ�״̬
            break;
            
        case PASS_READY_WAIT_3508:  // ����״̬���ȴ�3508��λ
            // ���3508�Ƿ�λ������ڿɽ��ܷ�Χ�ڣ�
            if(fabs( M3508_currentpos[8] - gimbal_3508_set_ecd) < 50.0) {
                pass_ready_state = PASS_READY_RELEASE_BALL;
                release_ball_timestamp = osKernelSysTick();
                HAL_GPIO_WritePin(finger_GPIO_Port, finger_Pin, 1); // ��ʱ���ɿ���
            }
            break;
            
        case PASS_READY_RELEASE_BALL:
            // ����Ƿ�ﵽ��ʱҪ��
            if(osKernelSysTick() - release_ball_timestamp >= 1000) {
                HAL_GPIO_WritePin(finger_GPIO_Port, finger_Pin, 0); // ��ס
                pass_ready_state = PASS_READY_SET_HAITAI;
            }
            break;
            
        case PASS_READY_SET_HAITAI:
            // ���ú�̩����
            joint_control.p_set      = 0.02;
            joint_control.v_set      = 1;
            joint_control.kp_set     = 5;
            joint_control.kd_set     = 2;
            joint_control.torque_set = 0;
            pass_ready_state = PASS_READY_IDLE; // �������
            break;
            
        case PASS_READY_IDLE:
        default:
            // ����״̬�޲���
            break;
    }
}
//����׼��
static void gimbal_pass_ready(void)
{
	
    // ֻ����״̬������ִ��ʵ�ʲ���
    if(pass_ready_state == PASS_READY_IDLE) {
        pass_ready_state = PASS_READY_SET_3508;
        // ע�⣺���ﲻ�������ɿ���
    }

}

//����׼��
static void gimbal_dribble_ready(void)
{	//��̩׼��

//	//������λ��1.0
//	joint_control.p_set        =   -1.93;
//	joint_control.v_set        =   0.3;
//	joint_control.kp_set       =   25;
//  joint_control.kd_set       =   5.0;
//	joint_control.torque_set   =   0;	
	//������λ��2.0
	joint_control.p_set        =   -1.50;
	joint_control.v_set        =   0.3;
	joint_control.kp_set       =   25;
  joint_control.kd_set       =   5.0;
	joint_control.torque_set   =   0;	
	//3508׼��
	gimbal_3508_set_ecd=0;//19*2275.2778;
	//HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}


//void sensor_detection()
//{
//        if(exit_flag == 1)
//        {
//            exit_flag = 2;
//            if(rising_falling_flag == GPIO_PIN_RESET)
//            {
//                //debouce
//                //����
//                osDelay(20);
//                if(HAL_GPIO_ReadPin(sensor_GPIO_Port, sensor_Pin) == GPIO_PIN_RESET)
//                {
//                    sensor_state=0;
//                    exit_flag = 0;
//                }
//                else
//                {
//                    exit_flag = 0;
//                }
//            }
//            else if(rising_falling_flag == GPIO_PIN_SET)
//            {
//                //debouce
//                //����
//                osDelay(20);
//                if(HAL_GPIO_ReadPin(sensor_GPIO_Port, sensor_Pin) == GPIO_PIN_SET)
//                {
//                    sensor_state=1;
//                    exit_flag = 0;
//                }
//                else
//                {
//                    exit_flag = 0;
//                }
//            }

//        }
//  }
//static uint8_t last_s1_state=0;
//static uint8_t last_s2_state=0;
////��̨�ֶ����ƺ���
//void gimbal_manual_control()
//{
//  //gimbal_S1();
//	gimbal_S2();
//}


//static void gimbal_S1(void)
//{

//   uint8_t current_s1=Remote_Ctrl.rc.s[1];
//	
//	if(current_s1!=last_s1_state)
//	{
//		last_s1_state=current_s1;
//		
//		if(current_s1 ==1)//S1��
//		{
//				HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);
//			
//		return;
//		}
//		else if(current_s1 ==3)//S1��
//		{
//			

//		return;
//		}
//	  else	if(current_s1 ==2)//S1��
//		{
//			HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);
//			
//			
//			return;
//		}
//		else
//		{
//		return;
//		}
//	}
//}


//static void gimbal_S2(void)
//{
//		uint8_t current_s2=Remote_Ctrl.rc.s[0];
//	
//	if(current_s2!=last_s2_state)
//	{
//		last_s2_state=current_s2;
//		
//		if(current_s2 ==1)//S2��
//		{
//		HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);
//		return;
//		}
//		else if(current_s2 ==3)//S2��
//		{

//		return;
//		}
//	  else	if(current_s2 ==2)//S2��
//		{
//			HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);
//			return;
//		}
//		else
//		{
//		return;
//		}
//	}

//}
