#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "tim.h"
#include "gpio.h"

const RC_ctrl_t *gimbal_rc_ctrl;
extern void esc_calibration_sequence(void);
void gimbal_task(void const *pvParameters)
{	
	
		gimbal_rc_ctrl = get_remote_control_point();
		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		esc_calibration_sequence();
	
          while(1)
		    {

		
    if(switch_is_up(gimbal_rc_ctrl->rc.s[1]))  
			{
		
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1400);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1400);	
      }	
					
		else if(switch_is_mid(gimbal_rc_ctrl->rc.s[1]))
			{
		
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1378);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1378);	
       }			
					
		else if(switch_is_down(gimbal_rc_ctrl->rc.s[1]))
			{
		
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1100);
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1100);	
      }	
		
		 
        }


}

void esc_calibration_sequence(void)
{
	
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1940);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1940);
	
  osDelay(3000);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1100);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1100);

  osDelay(2000);
}
