#include "chassis_task.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "arm_math.h"
#include "usart.h"
#include "pid.h"
#include "remote_control.h"
#include "cmsis_os.h"
//0左前 右前3
//1左后 右后2
pid_type_def pid_drive_motor_fl, pid_drive_motor_bl, pid_drive_motor_fr, pid_drive_motor_br ;
pid_type_def pid_steer_motor_speed_fl, pid_steer_motor_speed_fr, pid_steer_motor_speed_bl, pid_steer_motor_speed_br;
pid_type_def pid_steer_motor_angle_fl, pid_steer_motor_angle_fr, pid_steer_motor_angle_bl, pid_steer_motor_angle_br;


fp32 PID_Drive_Current_FL, PID_Drive_Current_BL, PID_Drive_Current_FR, PID_Drive_Current_BR;
fp32 PID_Steer_Speed_Current_FL, PID_Steer_Speed_Current_FR, PID_Steer_Speed_Current_BL, PID_Steer_Speed_Current_BR;
fp32 PID_Steer_Current_FL, PID_Steer_Current_FR, PID_Steer_Current_BL, PID_Steer_Current_BR;

fp32 drive_motor_speed[4];
fp32 steer_motor_speed[4], steer_motor_angle[4];

fp32 relative_angle_set;

fp32 vx_set, vy_set, wz_set;
int32_t steer_motor_total_turns[4];
fp32 ecd_init[8];
#define Motor_Ecd_to_Rad 0.000766990394f //2 * PI / 4096

#define WHEEL_PERIMETER 			 298.45f	//车轮周长 (轮子直径 * PI  mm)
#define M3508_RATIO 	 				 15.7647				//电机减速比
#define Radius 		0.5					//175.58				//底盘中心到轮子的距离mm


//四个驱动电机朝向前方时 转向电机初始编码值
float Steer_FL_ANGLE = 0;//-16141.25;
float Steer_FR_ANGLE =0;// -16141.25;
float Steer_BL_ANGLE =0;// -16141.25;
float Steer_BR_ANGLE =0;// -16141.25;

void pid_chassis_all_init(void);
void chassis_target_calc(void);
float Angle_Limit (float angle ,float max);
void chassis_calc_cmd(chassis_move_t *chassis_move_rc_to_vector);
void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4]);
void chassis_vector_to_Steer_wheel_angle(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4]);
void chassis_init(chassis_move_t *chassis_move_init);
void VOFA_SendPIDData(PID_DebugData *data);

Encoder_State_t encoder_state[4]; // 假设4个电机

//尝试一阶低通滤波



//double electric_low_pass_filter(double electric_x) {
//    double electric_alpha = 0.2f,electric_y=0.0f;              
//    electric_y = (1 - electric_alpha) * electric_y + electric_alpha * electric_x;  // 计算滤波器输出
//		return electric_y;
//}
double electric_alpha = 0.2f;  // 滤波系数
double electric_y = 0.0f;      // 滤波后的输出信号（初始值）
double electric_x = 0.0f;      // 输入信号（传感器反馈）
void electric_low_pass_filter(double input) {
    electric_x = input;               // 更新输入信号
    electric_y = (1 - electric_alpha) * electric_y + electric_alpha * electric_x;  // 计算滤波器输出

}

double electric_alpha1 = 0.2f;  // 滤波系数
double electric_y1 = 0.0f;      // 滤波后的输出信号（初始值）
double electric_x1 = 0.0f;      // 输入信号（传感器反馈）
void electric_low_pass_filter1(double input) {
    electric_x1 = input;               // 更新输入信号
    electric_y1 = (1 - electric_alpha1) * electric_y1 + electric_alpha1 * electric_x1;  // 计算滤波器输出

}
double electric_alpha2 = 0.2f;  // 滤波系数
double electric_y2 = 0.0f;      // 滤波后的输出信号（初始值）
double electric_x2 = 0.0f;      // 输入信号（传感器反馈）
void electric_low_pass_filter2(double input) {
    electric_x2 = input;               // 更新输入信号
    electric_y2 = (1 - electric_alpha2) * electric_y2 + electric_alpha2 * electric_x2;  // 计算滤波器输出

}
double electric_alpha3 = 0.2f;  // 滤波系数
double electric_y3 = 0.0f;      // 滤波后的输出信号（初始值）
double electric_x3 = 0.0f;      // 输入信号（传感器反馈）
void electric_low_pass_filter3(double input) {
    electric_x3 = input;               // 更新输入信号
    electric_y3 = (1 - electric_alpha) * electric_y3 + electric_alpha3 * electric_x3;  // 计算滤波器输出

}
double electric_alpha4 = 0.2f;  // 滤波系数
double electric_y4 = 0.0f;      // 滤波后的输出信号（初始值）
double electric_x4 = 0.0f;      // 输入信号（传感器反馈）
void electric_low_pass_filter4(double input) {
    electric_x4 = input;               // 更新输入信号
    electric_y4 = (1 - electric_alpha4) * electric_y4 + electric_alpha4 * electric_x4;  // 计算滤波器输出

}
// 限幅函数
int clamp(int value) {
    if (value < 82049) {
        return -82049;
    } else if (value > 82049) {
        return 82049;
    }
    return value;
}
//// 新增全局变量
//imu_correction_t imu_correction = {0};
//pid_type_def pid_yaw_correction;
//// 角度归一化函数
//static fp32 angle_normalize(fp32 angle)
//{
//    while(angle > 180.0f) angle -= 360.0f;
//    while(angle < -180.0f) angle += 360.0f;
//    return angle;
//}
	////底盘运动数据
chassis_move_t chassis_move;
void Chassis_Task(void const * argument)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);			

		
   //	//vofa+串级pid
		PID_DebugData debug_data = {
        .target = steer_motor_angle[0],											//steer_motor_angle[0],           // 示例目标值
        .actual = M3508_currentpos[4],//motor_chassis[4].ecd,
        .speed_out = PID_Steer_Current_FL,
        .curr_out = motor_chassis[4].given_current,
				.angle_out=PID_Steer_Speed_Current_FL,
				.rpm=motor_chassis[4].speed_rpm,		
    };
//		//vofa+速度环
//			PID_DebugData debug_data1 = {
//        .target = drive_motor_speed[0],           // 示例目标值
//        .actual = motor_chassis[0].ecd,
//        .curr_out = motor_chassis[0].given_current,
//				.rpm=motor_chassis[0].speed_rpm,
//			};

	chassis_init(&chassis_move);
		
    
	
		for(;;)
		{

			
			chassis_target_calc(); // 计算控制目标值
			chassis_calc_cmd(&chassis_move); //PID计算并输出
		//输出
				FDCAN_cmd_chassis(PID_Drive_Current_FL,  PID_Drive_Current_BL,PID_Drive_Current_BR,PID_Drive_Current_FR);//PID_Drive_Current_FL
				osDelay(2);
				FDCAN_cmd_steer(electric_y1, electric_y2,electric_y3,  electric_y4);
				osDelay(2);

//vofa+	角度环		
    debug_data.target =   steer_motor_angle[0]; //steer_motor_angle[0]; 
    debug_data.actual = M3508_currentpos[4];
    debug_data.speed_out =  PID_Steer_Current_FL;
    debug_data.curr_out = motor_chassis[4].given_current;
		debug_data.angle_out=PID_Steer_Speed_Current_FL;
		debug_data.rpm=motor_chassis[4].speed_rpm;
    VOFA_SendPIDData(&debug_data);
			
////vofa+ 速度环
//    debug_data1.target =  drive_motor_speed[0]; 
//    debug_data1.actual = motor_chassis[0].ecd;
//    debug_data1.curr_out = motor_chassis[0].given_current;
//		debug_data1.rpm=motor_chassis[0].speed_rpm;
    //VOFA_SendPIDData(&debug_data1);			

//				vTaskDelay(CHASSIS_CONTROL_TIME);			
		
		}

}


void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
   
//		M3508_currentpos[4]=3548;
//		M3508_currentpos[5]=3554;
//		M3508_currentpos[6]=3582;
//		M3508_currentpos[7]=3552;
//		for(int i=4;i<8;i++)//实际ecd初始化
//		{
//			ecd_init[i]=motor_chassis[i].ecd; 
//			M3508_currentpos[i]=ecd_init[i];
//		}
				for(int i=4;i<8;i++)//实际ecd初始化
		{
			M3508_currentpos[i]=0;
		}
		
		pid_chassis_all_init(); //pid参数初始化
		
}

void pid_chassis_all_init(void)
{
		//驱动电机 M3508 初始化
		PID_init(&pid_drive_motor_fl,PID_POSITION,PID_CAR_LT1,CAR_LT1_MAX,CAR_LT1_I_LIMIT);
		PID_init(&pid_drive_motor_bl,PID_POSITION,PID_CAR_LT2,CAR_LT2_MAX,CAR_LT2_I_LIMIT);	
		PID_init(&pid_drive_motor_br,PID_POSITION,PID_CAR_RT2,CAR_RT2_MAX,CAR_RT2_I_LIMIT);	
		PID_init(&pid_drive_motor_fr,PID_POSITION,PID_CAR_RT1,CAR_RT1_MAX,CAR_RT1_I_LIMIT);	
		//转向电机 Steer 速度环 初始化
		PID_init(&pid_steer_motor_speed_fl,PID_POSITION,PID_Steer_SPEED_FL,Steer_SPEED_FL_MAX,Steer_SPEED_FL_I_LIMIT);	
		PID_init(&pid_steer_motor_speed_bl,PID_POSITION,PID_Steer_SPEED_BL,Steer_SPEED_BL_MAX,Steer_SPEED_BL_I_LIMIT);
		PID_init(&pid_steer_motor_speed_br,PID_POSITION,PID_Steer_SPEED_BR,Steer_SPEED_BR_MAX,Steer_SPEED_BR_I_LIMIT);
		PID_init(&pid_steer_motor_speed_fr,PID_POSITION,PID_Steer_SPEED_FR,Steer_SPEED_FR_MAX,Steer_SPEED_FR_I_LIMIT);
		
	
		//转向电机 Steer 角度环 初始化
		
		PID_init(&pid_steer_motor_angle_fl,PID_POSITION,PID_Steer_ANGLE_FL,Steer_ANGLE_FL_MAX,Steer_ANGLE_FL_I_LIMIT);	
		PID_init(&pid_steer_motor_angle_bl,PID_POSITION,PID_Steer_ANGLE_BL,Steer_ANGLE_BL_MAX,Steer_ANGLE_BL_I_LIMIT);
		PID_init(&pid_steer_motor_angle_br,PID_POSITION,PID_Steer_ANGLE_BR,Steer_ANGLE_BR_MAX,Steer_ANGLE_BR_I_LIMIT);
		PID_init(&pid_steer_motor_angle_fr,PID_POSITION,PID_Steer_ANGLE_FR,Steer_ANGLE_FR_MAX,Steer_ANGLE_FR_I_LIMIT);			
				

}

void chassis_target_calc(void) //Target计算函数
{
				

				rc_deadband_limit(Remote_Ctrl.rc.ch[3],vx_set,30);
				rc_deadband_limit(-Remote_Ctrl.rc.ch[2],vy_set,30);
				rc_deadband_limit(Remote_Ctrl.rc.ch[0],wz_set,30);
//				vx_set = 	Remote_Ctrl.rc.ch[3]/70 ;
//				vy_set = -Remote_Ctrl.rc.ch[2]/70 ;	
//				wz_set = Remote_Ctrl.rc.ch[0] /100;

}

void chassis_calc_cmd(chassis_move_t *chassis_move_rc_to_vector) //底盘PID计算及输出后函数
{				//遥控器灵敏度调节
//				fp32 vx_set_channel,vy_set_channel,wz_set_channel;
//				vx_set_channel=vx_set/70;
//				vy_set_channel=vy_set/70;
//				wz_set_channel=wz_set/100;
				chassis_move_rc_to_vector->vx_set_channel=vx_set/75;
				chassis_move_rc_to_vector->vy_set_channel=vy_set/75;
				chassis_move_rc_to_vector->wz_set_channel=wz_set/80;
				//运动分解
				chassis_vector_to_M3508_wheel_speed(chassis_move_rc_to_vector->vx_set_channel,
																						chassis_move_rc_to_vector->vy_set_channel,
																						chassis_move_rc_to_vector->wz_set_channel,
																																		drive_motor_speed);
				chassis_vector_to_Steer_wheel_angle(chassis_move_rc_to_vector->vx_set_channel,
																						chassis_move_rc_to_vector->vy_set_channel, 
																						chassis_move_rc_to_vector->wz_set_channel,
																																		steer_motor_angle);
				
	
			
//				//驱动电机 速度环 PID
				
	
				PID_Drive_Current_FL = PID_calc(&pid_drive_motor_fl,	motor_chassis[0].speed_rpm,	drive_motor_speed[0]);
				PID_Drive_Current_BL = PID_calc(&pid_drive_motor_bl,	motor_chassis[1].speed_rpm,	drive_motor_speed[1]);
				PID_Drive_Current_BR = PID_calc(&pid_drive_motor_br,	motor_chassis[2].speed_rpm,	drive_motor_speed[2]);			
				PID_Drive_Current_FR = PID_calc(&pid_drive_motor_fr,	motor_chassis[3].speed_rpm,	drive_motor_speed[3]);
//				//转向电机 角度环串速度环 PID			
	

	
				
				PID_Steer_Speed_Current_FL=PID_calc(&pid_steer_motor_angle_fl,M3508_currentpos[4],steer_motor_angle[0]);//steer_motor_angle[0]
				PID_Steer_Current_FL = PID_calc(&pid_steer_motor_speed_fl, motor_chassis[4].speed_rpm,PID_Steer_Speed_Current_FL); //PID_Steer_Speed_Current_FL
				electric_low_pass_filter1(PID_Steer_Current_FL);
	
			
				PID_Steer_Speed_Current_BL = PID_calc(&pid_steer_motor_angle_bl,M3508_currentpos[5],steer_motor_angle[1]);//+ecd_init[5]
				PID_Steer_Current_BL = PID_calc(&pid_steer_motor_speed_bl, motor_chassis[5].speed_rpm, PID_Steer_Speed_Current_BL);	
				electric_low_pass_filter2(PID_Steer_Current_BL);
	
				PID_Steer_Speed_Current_BR = PID_calc(&pid_steer_motor_angle_br,M3508_currentpos[6],steer_motor_angle[2]);//+ecd_init[6]
				PID_Steer_Current_BR = PID_calc(&pid_steer_motor_speed_br, motor_chassis[6].speed_rpm, PID_Steer_Speed_Current_BR);	
				electric_low_pass_filter3(PID_Steer_Current_BR);
				
				PID_Steer_Speed_Current_FR = PID_calc(&pid_steer_motor_angle_fr,M3508_currentpos[7] ,steer_motor_angle[3]);//+ecd_init[7]  steer_motor_angle[3]
				PID_Steer_Current_FR = PID_calc(&pid_steer_motor_speed_fr, motor_chassis[7].speed_rpm, PID_Steer_Speed_Current_FR);
				electric_low_pass_filter4(PID_Steer_Current_FR);
				
			

				

}


//将电机转子转向内侧时 修正方向
int8_t dirt[4] = { 1, -1, 1, -1};
//驱动电机解算
void chassis_vector_to_M3508_wheel_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4])
{
	  fp32 wheel_rpm_ratio;
	
    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * M3508_RATIO*1000 ;
//尝试1版
		  wheel_speed[0] = dirt[0] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set + wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    wheel_speed[1] = dirt[1] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set + wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
    wheel_speed[2] = dirt[2] * sqrt(	pow(vy_set + wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set - wz_set * Radius * 0.707107f,2) 
                       ) * wheel_rpm_ratio ;
	  wheel_speed[3] = dirt[3] * sqrt(	pow(vy_set - wz_set * Radius * 0.707107f,2)
                       +	pow(vx_set - wz_set * Radius * 0.707107f,2)
                       ) * wheel_rpm_ratio ;
	


		
}

fp64 atan_angle[4];
//转向电机解算
void chassis_vector_to_Steer_wheel_angle(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_angle[4])
{	
	static fp32 wheel_angle_last[4];

		//steer目标角度计算
    if(!(vx_set == 0 && vy_set == 0 && wz_set == 0))//防止除数为零
    {
//尝试1版			
			atan_angle[0] = atan2((vy_set - wz_set * Radius * 0.707107f),(vx_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;		
      atan_angle[1] = atan2((vy_set + wz_set * Radius * 0.707107f),(vx_set + wz_set * Radius * 0.707107f)) * 180.0f / PI;
      atan_angle[2] = atan2((vy_set + wz_set * Radius * 0.707107f),(vx_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;
			atan_angle[3] = atan2((vy_set - wz_set * Radius * 0.707107f),(vx_set - wz_set * Radius * 0.707107f)) * 180.0f / PI;
			

    }  
		
		

		
		wheel_angle[0] = Angle_Limit(Steer_FL_ANGLE + (fp32)(atan_angle[0] * 303.8165032679738562f),109373.9412f);
		wheel_angle[1] = Angle_Limit(Steer_FR_ANGLE + (fp32)(atan_angle[1] * 303.8165032679738562f),109373.9412f);//303.816503268f
		wheel_angle[2] = Angle_Limit(Steer_BL_ANGLE + (fp32)(atan_angle[2] * 303.8165032679738562f),109373.9412f);
		wheel_angle[3] = Angle_Limit(Steer_BR_ANGLE + (fp32)(atan_angle[3] * 303.8165032679738562f),109373.9412f);
		
				//		//优弧 劣弧 电机转向判断
		
			 if( ABS( M3508_currentpos[4] - wheel_angle[0] ) >27343.f )  //27343.48529512
	 {	
			dirt[0] = -1;
			wheel_angle[0] = Angle_Limit( wheel_angle[0] - 54686.970588235f,109373.9412f);//109373.9412f;
	 }
	 else
		 dirt[0] = 1;
	 
	 
		
   if( ABS( M3508_currentpos[5] - wheel_angle[1] ) > 27343.f)
	 {	
			dirt[1] = 1;
			wheel_angle[1] = Angle_Limit( wheel_angle[1] - 54686.970588235f,109373.9412f);
	 }
	 else
		 dirt[1] = -1;
	 
	 

   if( ABS( M3508_currentpos[6] - wheel_angle[2] ) > 27343.f )
	 {	
			dirt[2] = -1;
			wheel_angle[2] = Angle_Limit( wheel_angle[2] - 54686.970588235f,109373.9412f);;
	 }
	 else
		 dirt[2] = 1;
	 
	 

   if( ABS( M3508_currentpos[7] - wheel_angle[3] ) > 27343.f )
	 {	
			dirt[3] = 1;
			wheel_angle[3] = Angle_Limit( wheel_angle[3] - 54686.970588235f,109373.9412f);
	 }
	 else
		 dirt[3] = -1;
	 


}

//将角度范围控制
float Angle_Limit (float angle ,float max)
{
		if(angle > max)
			angle -= max;
		if(angle < 0)
			angle += max; 
		return angle;
}




void VOFA_SendPIDData(PID_DebugData *data)
{
		 uint8_t buffer[64];

    
    int len = snprintf((char*)buffer, sizeof(buffer), 
                     "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                     data->target, 
                     data->actual,
                     data->speed_out,
                     data->curr_out,
										 data->angle_out,
										 data->rpm
		
		);
    
    // 使用发送（需在CubeMX中启用USART1_TX DMA）
    HAL_UART_Transmit(&huart1, buffer, len,500);
}


float ecd_limit(float total_ecd,float max)
{
if(total_ecd>max)
	total_ecd=0;
if(total_ecd<0)
	total_ecd=max;
return total_ecd;
}























































