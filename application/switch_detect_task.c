#include "main.h"
#include "switch_detect_task.h"


void Switch_Detect_Task(void const * argument)
{
    while (1) {
        uint8_t current_s2 = rc_ctrl.rc.s[1];
        
        // ��⵵λ�仯
        if (current_s2 != last_s2_state) {
            last_s2_state = current_s2;
            
            // �����¼���ִ������
            EventBits_t event = 0;
            switch (current_s2) {
                case 1: event = S2_UP_EVENT;   break;
                case 2: event = S2_DOWN_EVENT; break;
                case 3: event = S2_MID_EVENT;  break;
            }
            xEventGroupSetBits(g_gimbal_events, event);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 20ms�������
    }

}




