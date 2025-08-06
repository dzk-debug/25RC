#include "measurement_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "remote_control.h"

const RC_ctrl_t *rod_rc_ctrl;

void measurement_task(void const *pvParameters)
{
	
				 rod_rc_ctrl = get_remote_control_point();
				 HAL_GPIO_WritePin(PUSH_PIN_GPIO_Port,PUSH_PIN_Pin ,GPIO_PIN_RESET);

    while (1)
    {

//     if (switch_is_up(rod_rc_ctrl->rc.s[0]))
//        {

//          HAL_GPIO_WritePin(PUSH_PIN_GPIO_Port,PUSH_PIN_Pin ,GPIO_PIN_SET);
//					
//        }
//     else  if (switch_is_mid(rod_rc_ctrl->rc.s[0]))
//        {
//					
//          HAL_GPIO_WritePin(PUSH_PIN_GPIO_Port,PUSH_PIN_Pin ,GPIO_PIN_RESET);

//        }    

    }
}
