#include "Controller_Send_Task.h"
#include "Usart_Send.h"
#include "CAN_receive.h"
#include "DM_motor.h"
#include "cmsis_os.h"

void Controller_Send_Task(void const *argument)
{
    (void)argument;
    uint16_t cnt = 0;
    for (;;)
    {
        DM_Motor_Request_Feedback();     /* 500Hz：使能维持 + 阻尼/重力补偿 MIT 帧 */
        if (++cnt >= 10u)                /* 50Hz：串口 39 字节帧。
                                            USART1=115200，一帧约 3.4ms，不能跟 CAN 一样快 */
        {
            cnt = 0;
            Usart_send_cotrol_data_task();
        }
        osDelay(2);
    }
}
