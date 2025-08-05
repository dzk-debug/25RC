/* bsp_fdcan.c */
#include "bsp_can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
void fdcan_filter_init(void)
{
    /* FDCAN1 ÂË²¨Æ÷ÅäÖÃ */
    FDCAN_FilterTypeDef fdcan_filter_st = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,//0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = 0x00000000,  // ±ê×¼IDÆ¥ÅäÖµ
        .FilterID2 = 0x00000000,   // 
        .RxBufferIndex = 0,
        .IsCalibrationMsg = 0
    };
    
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter_st);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, 
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* FDCAN2 ÂË²¨Æ÷ÅäÖÃ */
    fdcan_filter_st.FilterIndex = 21,//14
    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter_st);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2,
     FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		/* FDCAN3 ÂË²¨Æ÷ÅäÖÃ */
		fdcan_filter_st.FilterIndex = 14,//14
    HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter_st);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3,
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
