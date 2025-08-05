#ifndef PASS_TASK_H
#define PASS_TASK_H
#include "user_lib.h"
#include "remote_control.h"


typedef enum
{
    PASS_STOP = 0,
    PASS_DONE,
} pass_mode_e;

typedef struct
{
    pass_mode_e pass_mode;
    const RC_ctrl_t *pass_rc;
	



    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;



} pass_control_t;






#endif


