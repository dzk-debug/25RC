/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�
		fp32 dead_zone;  // ������������ֵ
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
    float KP;             //����ϵ��
    float KI;             //����ϵ��
    float KD;             //΢��ϵ��
    float Alpha;           //΢��һ���˲���ϵ��
		float Deadband;       //���� ��������ֵС������ PIDֹͣ���㡣
    float LimitIntegral;  //�����޷�
    float LimitOutput;    //������޷�
}PID_Parameter_Typedef;

typedef struct
{
    uint16_t ErrorCount;    /*!< Error status judgment count */
    PID_Status_e Status;    /*!< Error Status */
}PID_ErrorHandler_Typedef;


typedef struct _PID_TypeDef
{
		PID_Type_e Type;    //PID���� λ��ʽor����ʽ ͨ��ʹ��λ��ʽ
	
		float Target;       //Ŀ��ֵ
		float Measure;      //ʵ��ֵ
	
    float Err[3];       //��� Ŀ��ֵ-����ֵ = ��� ��ǰ�Լ���ȥ���ε���� 
		float Integral;     //������ֵ ����ۼ�
    float Pout;         // KP * ���ֵ �������
    float Iout;         // KI * ������ �������
    float Dout;         // KD * ���΢�֣���֣�΢�����
    float Output;       //����� Pout + Iout + Dout = Output
	
	  LowPassFilter1p_Info_TypeDef Dout_LPF; //΢�������һ���˲���
	
	
		PID_Parameter_Typedef Param;            //PID�����ṹ��
    PID_ErrorHandler_Typedef ERRORHandler;  //PID������ṹ��

    /**
     * @brief ��ʼ��PID�����ĺ���ָ�룬��PID����װ����PID�����ṹ���С�
     * @param PID: ָ��_pid_TypeDef�ṹ��ָ�룬����PID����������Ϣ��
     * @param Param: ָ��PID�����ĸ�����ָ�룬����PID������
     * @retval PID����״̬ ����PID�Ƿ��ʼ���ɹ���
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *PID,float *Param);

    /**
     * @brief ���pid���㺯���ļ��ָ�롣
     * @param PID:ָ��_pid_TypeDef�ṹ��ָ�룬����PID����������Ϣ��
     * @retval ��.
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
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
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
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern fp32 PID_Calc_Ecd(pid_type_def *pid, fp32 ref, fp32 set, fp32 ecd_range);
/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
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
