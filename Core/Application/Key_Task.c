#include "Key_Task.h"
#include "gpio.h"
#include "cmsis_os.h"

uint16_t key_mode = 0;

// 内部变量（仅本文件使用）
static uint16_t last_key_press = 0;       
static TickType_t debounce_tick = 0;      
static const TickType_t debounce_time = pdMS_TO_TICKS(10); // 消抖时间10ms（转为tick单位）比haldelay好


void Key_task(void)
{
  
    GPIO_PinState pin_level = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);//检测按键口的高低电平
    // 转换为逻辑状态：1=按下，0=未按
    uint16_t current_key = (pin_level == GPIO_PIN_RESET) ? 1 : 0;


    if ((xTaskGetTickCount() - debounce_tick) >= debounce_time)
    {
       //与上一次的按键状态取反
        if (current_key == 1 && last_key_press == 0)
        {
            // 状态翻转（0→1，1→0）
            key_mode = (key_mode == 0) ? 1 : 0;
        }

        // 4. 更新上一次按键状态和消抖时间戳
        last_key_press = current_key;
        debounce_tick = xTaskGetTickCount();
    }

}

void Key_Task(void const *argument)
{
    (void)argument;
    for (;;)
    {
        Key_task();      /* 扫一次、消抖 */
        osDelay(10);     /* 10ms 一次 */
    }
}

uint16_t Key_GetMode(void)
{
    return key_mode;
}
