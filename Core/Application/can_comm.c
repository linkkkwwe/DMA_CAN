#include "can_comm.h"
#include "can.h"

HAL_StatusTypeDef can_comm_init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;   /* 全收 */
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;

    /* 三步都检查返回值：任一失败立即上报，避免"以为配好其实没配" */
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        return HAL_ERROR;
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        return HAL_ERROR;
    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
