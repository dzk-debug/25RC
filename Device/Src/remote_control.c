
#include "remote_control.h"
	

Remote_Info_Typedef Remote_Ctrl={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};
int uu=0;
uint8_t SBUS_MultiRx_Buf[2][RC_FRAME_LENGTH];

void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl)
{
    if (sbus_buf == NULL || Remote_Ctrl == NULL) return;
uu=1;
    /* Channel 0, 1, 2, 3 */
    Remote_Ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    Remote_Ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    Remote_Ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    Remote_Ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    Remote_Ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ); //& 0x07ff;                 			      //!< Channel 4
uu=2;
    /* Switch left, right */
    Remote_Ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    Remote_Ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    Remote_Ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    Remote_Ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    Remote_Ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    Remote_Ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    Remote_Ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    Remote_Ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    Remote_Ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    Remote_Ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    Remote_Ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    Remote_Ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    Remote_Ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
		/* reset the online count */
		Remote_Ctrl->online_cnt = 0xFAU;
		
		/* reset the lost flag */
		Remote_Ctrl->rc_lost = false;
}

