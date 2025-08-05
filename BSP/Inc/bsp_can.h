#ifndef BSP_RC_H
#define BSP_RC_H

#include "main.h"
#include "stm32h7xx_hal.h"

// ң����ͨ��Э����ض��壨��DR16���ջ�ƥ�䣩

#define RC_PROTOCOL_HEADER     0xA1   // Э��ͷ

// ��ʼ����������
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

// ȫ�־������
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



/**
  * @brief  FDCAN�˲�����ʼ��
  * @note   ����˫CAN���ߵ��˲�������������CAN
  */
void fdcan_filter_init(void);

#endif
