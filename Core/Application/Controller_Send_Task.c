#include "Controller_Send_Task.h"
#include "Usart_Send.h"
#include "cmsis_os.h"

void Controller_Send_Task(void const *argument)
{
    (void)argument;
    for (;;)
    {
        Usart_send_cotrol_data_task();   /* 组帧 + DMA 发送 */
        osDelay(20);                     /* 50Hz */
    }
}
