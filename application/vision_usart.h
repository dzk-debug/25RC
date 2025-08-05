/*


*/
#ifndef  VISION_USART_H
#define  VISION_USART_H

#include "struct_typedef.h"

#define VISION_TX_BUF_NUM 9u
#define VISION_RX_BUF_NUM 20u
#define VISION_FRAME_LENGTH 10u


typedef struct
{
	char start_flag;
	int16_t get_yaw;
	int16_t get_pitch;
	int16_t get_roll;
	int16_t get_shoot_delay;
	char end_flag;
	
}vision_recevise_t;

typedef struct
{
	fp32 curr_yaw;
	fp32 curr_pitch;
	uint8_t state;//a×ÔÃé sÐ¡·ù b´ó·ù
	uint8_t mark;
	uint8_t anti_top;
	uint8_t enemy_color;
	
	int16_t delta_x;
	int16_t delta_y;

}vision_send_t;




extern void vision_init(void);
extern void decode_vision(volatile const uint8_t *sbus_buf);
extern void  vision_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
extern void usart_to_vision(vision_send_t *vision_send_f);
extern void vision_date_update(void);


#endif
