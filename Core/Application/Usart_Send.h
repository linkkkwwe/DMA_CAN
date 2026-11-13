#ifndef __USART_SEND_H__
#define __USART_SEND_H__
#include <main.h>
#define FRAME_HEADER_LENGTH 5 // 帧头数据长度
#define CMD_ID_LENGTH 2       // 命令码ID数据长度
#define CONTR_DATA_LENGTH 30        // 数据段长度
#define FRAME_TAIL_LENGTH 2   // 帧尾数据长度

#define DATA_FRAME_LENGTH 39 // 整个数据帧的长度

#define CONTROLLER_CMD_ID 0x0302 // 自定义控制器命令码

#define Joint_1_Zero_angle 0.0f
#define Joint_2_Zero_angle 0.0f
#define Joint_3_Zero_angle 0.0f
#define Joint_4_Zero_angle 0.0f

typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[CONTR_DATA_LENGTH];    // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
} Controller_t;                   // 自定义控制器数据包

void Usart_send_cotrol_data_task(void);
#endif
