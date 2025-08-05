///*
//�����Ӿ������Ϣ
//����ϵͳͨ��UART���Ӿ�ģ�����ͨ�ţ�ʹ��DMAʵ�ָ�Ч�����ݽ��պͷ��͡�
//����ṹ��������Ϊ��ʼ�������ա�����ͷ����ĸ���Ҫ���֣��ܹ���Ч�Ӿ���Ϣ�Ĵ��䡣

//*/
//#include "vision_usart.h"
//#include "main.h"


//extern UART_HandleTypeDef huart1;
//extern DMA_HandleTypeDef hdma_usart1_rx;
//extern DMA_HandleTypeDef hdma_usart1_tx;

///*
//�Ӿ�Э�����

//*/
//uint8_t vision_rx_buf[2][VISION_RX_BUF_NUM];//˫����������ԭʼ����20�ֽڣ�����8�ֽڷ�ֹ���
//vision_recevise_t vision_recevise;//�������Ϣ��10�ֽ�

//uint8_t vision_tx_buf[VISION_TX_BUF_NUM];//���ͻ�����9�ֽ�
//vision_send_t vision_send;//���͵���Ϣ��16�ֽ�

//void vision_init(void)
//{
//	vision_dma_init(vision_rx_buf[0],vision_rx_buf [1],VISION_RX_BUF_NUM );
//	
//	


//}

//void  vision_dma_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
//{
//	//enable the DMA transfer for the receiver request
//    //ʹ��DMA���ڽ���
//    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);//����
//		
//    //enalbe idle interrupt
//    //ʹ�ܿ����ж�
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

//    //disable DMA
//    //ʧЧDMA
//    __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_rx);
//    }

//    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);//
//    
//		//memory buffer 1
//    //�ڴ滺����1
//    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
//    //memory buffer 2
//    //�ڴ滺����2
//    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
//    //data length
//    //���ݳ���
//    hdma_usart1_rx.Instance->NDTR = dma_buf_num;//�����������ݴ洢������ݼ����ü������а�������ִ�е�������
//    //enable double memory buffer
//    //ʹ��˫������
//    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

//    //enable DMA
//    //ʹ��DMA
//    __HAL_DMA_ENABLE(&hdma_usart1_rx);
//		///////���ͳ�ʼ��///
//		SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);//����
//		//disable DMA
//    //ʧЧDMA
//    __HAL_DMA_DISABLE(&hdma_usart1_tx);

//    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    }

//    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);//����dma�����ַ
//    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
//    hdma_usart1_tx.Instance->NDTR = 0;


//}

////�����жϣ������Ӿ���Ϣ
//void USART1_IRQHandler(void)
//{
//    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
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
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = VISION_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart1_rx.Instance->NDTR = VISION_RX_BUF_NUM;

//            //set memory buffer 1
//            //�趨������1
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == VISION_FRAME_LENGTH)
//            {
//               //���ý��뺯��
//							decode_vision(vision_rx_buf[1]);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = VISION_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart1_rx.Instance->NDTR = VISION_FRAME_LENGTH;

//            //set memory buffer 0
//            //�趨������0
//            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == VISION_FRAME_LENGTH)
//            {
//               //���ý��뺯��
//							decode_vision(vision_rx_buf[0]);
//            }
//        }
//    }
//}

///*
//�����յ����Ӿ���Ϣ����
//������ջ���������
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
//ʹ��usart1 dma����
//*/
//void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
//{

//    //disable DMA
//    //ʧЧDMA
//    __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart1_tx);
//    }

//    //clear flag
//    //�����־λ
//    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
//    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_HTIF7);

//    //set data address
//    //�������ݵ�ַ
//    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
//    //set data length
//    //�������ݳ���
//    hdma_usart1_tx.Instance->NDTR = len;

//    //enable DMA
//    //ʹ��DMA
//    __HAL_DMA_ENABLE(&hdma_usart1_tx);
//}

///*
//���Ӿ������ű��벢����
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
//����
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
