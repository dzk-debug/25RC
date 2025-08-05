#ifndef BSP_RC_H
#define BSP_RC_H

#include "main.h"
#include "stm32h7xx_hal.h"

// 遥控器通信协议相关定义（与DR16接收机匹配）

#define RC_PROTOCOL_HEADER     0xA1   // 协议头

// 初始化函数声明
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

// 全局句柄引用
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



/**
  * @brief  FDCAN滤波器初始化
  * @note   配置双CAN总线的滤波器参数并启动CAN
  */
void fdcan_filter_init(void);

#endif
