#include "Controller_Send_Task.h"
#include "Usart_Send.h"
#include "CAN_receive.h"
#include "cmsis_os.h"

void Controller_Send_Task(void const *argument)
{
    (void)argument;
    for (;;)
    {
        DM_Motor_Request_Feedback();     /* 零力矩帧请求 4 个电机反馈 */
        Usart_send_cotrol_data_task();   /* 组帧 + DMA 发送 */
        osDelay(20);                     /* 50Hz */
    }
}
