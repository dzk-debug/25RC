#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
#include <string.h>



#define SBUS_RX_BUF_NUM		36u

#define RC_FRAME_LENGTH		18u

#define RC_CH_VALUE_OFFSET		1024U

extern uint8_t SBUS_MultiRx_Buf[2][RC_FRAME_LENGTH];



typedef  struct
{

	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	

	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

extern Remote_Info_Typedef Remote_Ctrl;

extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *Remote_Ctrl);

#endif 

