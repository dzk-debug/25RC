#include "bsp_uart.h"
#include "usart.h"
#include "remote_control.h"

static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t );

static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){


 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxEventType = HAL_UART_RXEVENT_TC;

 huart->RxXferSize    = DataLength;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 

 HAL_DMAEx_MultiBufferStart(&hdma_uart5_rx,(uint32_t)SrcAddress,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);
	

}


void BSP_USART_Init(void){

	USART_RxDMA_MultiBufferStart(&huart5,(uint32_t *)&(huart5.Instance->RDR),(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],36);
		
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
	 if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
  {
			//Disable DMA 
			__HAL_DMA_DISABLE(huart->hdmarx);

			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

      if(Size == RC_FRAME_LENGTH)
      {
        SBUS_TO_RC(SBUS_MultiRx_Buf[0],&Remote_Ctrl);
      }
  }
  /* Current memory buffer used is Memory 1 */
  else
  {
			//Disable DMA 
			__HAL_DMA_DISABLE(huart->hdmarx);
		
       ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
		
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);

      if(Size == RC_FRAME_LENGTH)
      {
        SBUS_TO_RC(SBUS_MultiRx_Buf[1],&Remote_Ctrl);
      }
  }
	

  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	
	 huart->RxEventType = HAL_UART_RXEVENT_TC;

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	
  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}







