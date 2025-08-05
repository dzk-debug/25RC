///*
//处理视觉相关信息
//整个系统通过UART与视觉模块进行通信，使用DMA实现高效的数据接收和发送。
//代码结构清晰，分为初始化、接收、解码和发送四个主要部分，能够有效视觉信息的传输。

//*/
//#include "vision_usart.h"
//#include "main.h"


//extern UART_HandleTypeDef huart1;
//extern DMA_HandleTypeDef hdma_usart1_rx;
//extern DMA_HandleTypeDef hdma_usart1_tx;

///*
//视觉协议解析

//*/
//uint8_t vision_rx_buf[2][VISION_RX_BUF_NUM];//双缓存区接收原始数据20字节，数据8字节防止溢出
//vision_recevise_t vision_recevise;//解码后信息，10字节

//uint8_t vision_tx_buf[VISION_TX_BUF_NUM];//发送缓存区9字节
//vision_send_t vision_send;//发送的信息，16字节

//void vision_init(void)
//{
//	vision_dma_init(vision_rx_buf[0],vision_rx_buf [1],VISION_RX_BUF_NUM );
//	
//	


//}

//void  vision_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
//{
//	//enable the DMA transfer for the receiver request
//    //使能DMA串口接收
//    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);//接收
//		
//    //enalbe idle interrupt
//    //使能空闲中断
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

//    //disable DMA
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    }

//    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);//
//    
//		//memory buffer 1
//    //内存缓冲区1
//    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
//    //memory buffer 2
//    //内存缓冲区2
//    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
//    //data length
//    //数据长度
//    hdma_usart1_rx.Instance->NDTR = dma_buf_num;//计数器在数据存储结束后递减，该计数器中包含仍需执行的事务数
//    //enable double memory buffer
//    //使能双缓冲区
//    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

//    //enable DMA
//    //使能DMA
//    __HAL_DMA_ENABLE(&hdma_usart1_rx);
//		///////发送初始化///
//		SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);//发送
//		//disable DMA
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart1_tx);

//    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    }

//    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);//设置dma外设地址
//    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
//    hdma_usart1_tx.Instance->NDTR = 0;


//}

////串口中断，接收视觉信息
//void USART1_IRQHandler(void)
//{
//    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);
//    }
//    else if(USART1->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */
//    
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = VISION_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = VISION_RX_BUF_NUM;

//            //set memory buffer 1
//            //设定缓冲区1
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == VISION_FRAME_LENGTH)
//            {
//               //调用解码函数
//							decode_vision(vision_rx_buf[1]);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = VISION_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = VISION_FRAME_LENGTH;

//            //set memory buffer 0
//            //设定缓冲区0
//            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == VISION_FRAME_LENGTH)
//            {
//               //调用解码函数
//							decode_vision(vision_rx_buf[0]);
//            }
//        }
//    }
//}

///*
//将接收到的视觉信息解码
//输入接收缓存区数组
//*/
//static void decode_vision(volatile const uint8_t *vision_buf)
//{
//	if (vision_buf == NULL )
//    {
//        return;
//    }
//		vision_recevise.start_flag =vision_buf[0];
//		vision_recevise.end_flag =vision_buf[9];
//		if(vision_recevise.start_flag=='s'&&vision_recevise.end_flag=='e')
//		{
//			vision_recevise.get_yaw =((vision_buf[1]<<8)|(vision_buf[2]<<0))&0xffff;
//			vision_recevise.get_pitch  =((vision_buf[3]<<8)|(vision_buf[4]<<0))&0xffff;
//			vision_recevise.get_roll =((vision_buf[5]<<8)|(vision_buf[6]<<0))&0xffff;
//			vision_recevise.get_shoot_delay  =((vision_buf[7]<<8)|(vision_buf[8]<<0))&0xffff;
//		}



//}
///*
//使能usart1 dma发送
//*/
//void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
//{

//    //disable DMA
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    }

//    //clear flag
//    //清除标志位
//    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
//    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7);

//    //set data address
//    //设置数据地址
//    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
//    //set data length
//    //设置数据长度
//    hdma_usart1_tx.Instance->NDTR = len;

//    //enable DMA
//    //使能DMA
//    __HAL_DMA_ENABLE(&hdma_usart1_tx);
//}

///*
//给视觉发送信编码并发送
//*/
//void usart_to_vision(vision_send_t *vision_send_f)
//{
////	uint32_t* c_yaw,*c_pitch;
////	c_yaw=(uint32_t *)&(vision_send_f->curr_yaw);
////	c_pitch=(uint32_t *)&(vision_send_f->curr_pitch);
////	
////	vision_tx_buf[0]=(*c_yaw>>24) &0xff;
////	vision_tx_buf[1]=(*c_yaw>>16) &0xff;
////	vision_tx_buf[3]=(*c_yaw>>8 ) &0xff;
////	vision_tx_buf[4]=(*c_yaw>>0 ) &0xff;
////	vision_tx_buf[5]=(*c_pitch>>24) &0xff;
////	vision_tx_buf[6]=(*c_pitch>>16) &0xff;
////	vision_tx_buf[7]=(*c_pitch>>8 ) &0xff;
////	vision_tx_buf[8]=(*c_pitch>>0 ) &0xff;
////	vision_tx_buf[9]=vision_send_f->state ;
////	vision_tx_buf[10]=vision_send_f->mark ;
////	vision_tx_buf[11]=vision_send_f->anti_top ;
////	vision_tx_buf[12]=vision_send_f->enemy_color ;
////	vision_tx_buf[12]=(vision_send_f->delta_x>>8)&0xff;
////	vision_tx_buf[13]=(vision_send_f->delta_x>>0)&0xff;
////	vision_tx_buf[14]=(vision_send_f->delta_y>>8)&0xff;
////	vision_tx_buf[15]=(vision_send_f->delta_y>>0)&0xff;
//	vision_tx_buf[0]=0x00;
//	vision_tx_buf[1]=0x00;
//	vision_tx_buf[2]=vision_send_f->state ;
//	vision_tx_buf[3]=vision_send_f->mark ;
//	vision_tx_buf[4]=vision_send_f->anti_top ;
//	vision_tx_buf[5]=vision_send_f->enemy_color ;
//	vision_tx_buf[6]=0x00;
//	vision_tx_buf[7]=0x00;
//	vision_tx_buf[8]=0x0A;

//	
//	
//	
//	usart1_tx_dma_enable(vision_tx_buf,VISION_TX_BUF_NUM);
//}

///*
//解码
//*/
//void vision_date_update()
//{
////	vision_send.curr_yaw =gimbal_control.gimbal_yaw_motor.absolute_angle ;
////	vision_send.curr_pitch =gimbal_control.gimbal_pitch_motor .absolute_angle ;
//	vision_send.state ='a';
//	vision_send.mark =0xff;
//	vision_send.anti_top =0xff;
//	vision_send.enemy_color =0;
//	vision_send.delta_x =0;
//	vision_send.delta_y =0;
//	

//}	
