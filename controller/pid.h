/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
#include "lpf.h"

#define LIMIT(x,min,max) (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define ABS(x) (((x)>0)?(x):(-(x)))
#define PI 3.141592653589
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};
typedef enum
{
		PID_Type_None = 0x00U,         /*!< No Type */
		PID_POSITION2 = 0x01U,          /*!< position pid */
		PID_VELOCITY = 0x02U,          /*!< velocity pid */
    PID_TYPE_NUM,
}PID_Type_e;
typedef enum
{
    PID_ERROR_NONE = 0x00U,        /*!< No error */
    PID_FAILED_INIT = 0x01U,        /*!< Initialization failed */
		PID_CALC_NANINF = 0x02U,      /*!< Not-a-number (NaN) or infinity is generated */
    PID_Status_NUM,
}PID_Status_e;

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
		fp32 dead_zone;  // 新增：死区阈值
} pid_type_def;

typedef struct
{
fp32 kp;
fp32 ki;
fp32 kd;

fp32 set;
fp32 get;
fp32 err;

fp32 max_out;
fp32 max_iout;
	
fp32 Pout;
fp32 Iout;
fp32 Dout;

fp32 out;
	
}series_pid_type_def;
//dzk
/**
 * @brief typedef structure that contains the information for the pid controller.
 */
#ifndef PID_PARAMETER_NUM 
#define PID_PARAMETER_NUM 7							
#endif
typedef struct
{
    float KP;             //比例系数
    float KI;             //积分系数
    float KD;             //微分系数
    float Alpha;           //微分一阶滤波器系数
		float Deadband;       //死区 当误差绝对值小于死区 PID停止计算。
    float LimitIntegral;  //积分限幅
    float LimitOutput;    //总输出限幅
}PID_Parameter_Typedef;

typedef struct
{
    uint16_t ErrorCount;    /*!< Error status judgment count */
    PID_Status_e Status;    /*!< Error Status */
}PID_ErrorHandler_Typedef;


typedef struct _PID_TypeDef
{
		PID_Type_e Type;    //PID类型 位置式or增量式 通常使用位置式
	
		float Target;       //目标值
		float Measure;      //实际值
	
    float Err[3];       //误差 目标值-期望值 = 误差 当前以及过去两次的误差 
		float Integral;     //误差积分值 误差累加
    float Pout;         // KP * 误差值 比例输出
    float Iout;         // KI * 误差积分 积分输出
    float Dout;         // KD * 误差微分（差分）微分输出
    float Output;       //总输出 Pout + Iout + Dout = Output
	
	  LowPassFilter1p_Info_TypeDef Dout_LPF; //微分输出的一阶滤波器
	
	
		PID_Parameter_Typedef Param;            //PID参数结构体
    PID_ErrorHandler_Typedef ERRORHandler;  //PID错误处理结构体

    /**
     * @brief 初始化PID参数的函数指针，将PID参数装载至PID参数结构体中。
     * @param PID: 指向_pid_TypeDef结构的指针，包含PID控制器的信息。
     * @param Param: 指向PID参数的浮点型指针，包含PID参数。
     * @retval PID错误状态 返回PID是否初始化成功。
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *PID,float *Param);

    /**
     * @brief 清除pid计算函数的简短指针。
     * @param PID:指向_pid_TypeDef结构的指针，包含PID控制器的信息。
     * @retval 无.
     */
		void (*PID_Calc_Clear)(struct _PID_TypeDef *PID);
				
}PID_Info_TypeDef;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern fp32 PID_Calc_Ecd(pid_type_def *pid, fp32 ref, fp32 set, fp32 ecd_range);
/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

//dzk4.16
extern void PID_Init(PID_Info_TypeDef *PID,PID_Type_e Type,float Param[PID_PARAMETER_NUM]);
extern float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure);

static fp32 ecd_zero(uint16_t ecd, uint16_t offset_ecd, uint16_t ecd_range);
fp32 angle_zero(fp32 angle, fp32 offset_angle);
fp32 PID_calc_cascade(pid_type_def *pid, fp32 ref, fp32 set);
#endif
