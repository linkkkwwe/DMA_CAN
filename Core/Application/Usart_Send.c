#include "Usart_Send.h"
#include "CAN_receive.h"
#include "CRC.h"
#include "usart.h"
#include "cmsis_os.h"
#include "Key_Task.h"
#include <string.h>
Controller_t tx_packet;

static void Data_Concatenation(uint8_t *data, uint16_t data_lenth);

void Usart_send_cotrol_data_task(void)
{
    uint16_t key_val = Key_GetMode(); 

    const  DMMotor_measure_t *m1 = get_DM_joint1_motor_measure_point();
    const  DMMotor_measure_t *m2 = get_DM_joint2_motor_measure_point();
    const  DMMotor_measure_t *m3 = get_DM_joint3_motor_measure_point();
    const  DMMotor_measure_t *m4 = get_DM_joint4_motor_measure_point();

    int32_t ecd1 = (int32_t)((m1->pos - Joint_1_Zero_angle) * 4096.0f / 6.2831853f);
    int32_t ecd2 = (int32_t)((m2->pos - Joint_2_Zero_angle) * 4096.0f / 6.2831853f);
    int32_t ecd3 = (int32_t)((m3->pos - Joint_3_Zero_angle) * 4096.0f / 6.2831853f);
    int32_t ecd4 = (int32_t)((m4->pos - Joint_4_Zero_angle) * 4096.0f / 6.2831853f);

    //电机反馈值（计算后）
    tx_packet.data[0] = (ecd1 >> 8) & 0xFF;
    tx_packet.data[1] = ecd1 & 0xFF;

    tx_packet.data[2] = (ecd2 >> 8) & 0xFF;
    tx_packet.data[3] = ecd2 & 0xFF;

    tx_packet.data[4] = (ecd3 >> 8) & 0xFF;
    tx_packet.data[5] = ecd3 & 0xFF;
 
    tx_packet.data[6] = (ecd4 >> 8) & 0xFF;
    tx_packet.data[7] = ecd4 & 0xFF;//低8位
		//按键值
	  tx_packet.data[8] = (key_val >> 8) & 0xFF;
    tx_packet.data[9] = key_val& 0xFF;//低8位
	  //补充剩下的字节
     for (uint8_t i = 10; i < CONTR_DATA_LENGTH; i++) {
        tx_packet.data[i] = 0x00;
    }
    //将所有数据拼在一起
		Data_Concatenation(tx_packet.data, CONTR_DATA_LENGTH);
		//按照八个字节的数据发出去整包发出
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)(&tx_packet), DATA_FRAME_LENGTH);
	
	}

//帧头5位+2命令帧+30位数据帧+2帧尾
/**
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// 帧头数据
    tx_packet.frame_header.sof = 0xA5;                              // 数据帧起始字节，固定值为 0xA5
    tx_packet.frame_header.data_length = data_lenth;                // 数据帧中数据段的长度
    tx_packet.frame_header.seq = seq++;                             // 包序号
    Append_CRC8_Check_Sum((uint8_t *)(&tx_packet.frame_header), 5); // 添加帧头 CRC8 校验位
    /// 命令码ID
    tx_packet.cmd_id = CONTROLLER_CMD_ID;
    /// 数据段
    memcpy(tx_packet.data, data, data_lenth);
    /// 帧尾CRC16，整包校验
    Append_CRC16_Check_Sum((uint8_t *)(&tx_packet), DATA_FRAME_LENGTH);
}
