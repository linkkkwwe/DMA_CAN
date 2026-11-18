#include "DM_motor.h"
#include "can.h"
#include <string.h>
#include "CAN_receive.h"
#include <math.h>

static void DM_Motor_SendMitFrame(uint16_t motor_id, const uint8_t data[8]);
static int float_to_uint(float x, float x_min, float x_max, int bits);
void DM_Motor_Request_Feedback(void);
void DM_Motor_MitCtrl(uint16_t motor_id, float kp, float kd, float tor_ff);


static int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DM_Motor_Request_Feedback(void)
{
    static const uint8_t enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    const  DMMotor_measure_t *m1 = get_DM_joint1_motor_measure_point();
    const  DMMotor_measure_t *m2 = get_DM_joint2_motor_measure_point();
    const  DMMotor_measure_t *m3 = get_DM_joint3_motor_measure_point();
    const  DMMotor_measure_t *m4 = get_DM_joint4_motor_measure_point();

    /* 失能的电机补发使能帧；随后按关节发“阻尼 + 重力补偿”MIT 帧。
       J1/J4 回转轴：只给阻尼；J2/J3：τ_ff = −A·cos(pos − PHI0) 前馈。 */
    for (uint16_t id = 1u; id <= 4u; id++)
    {
        const DMMotor_measure_t *m = (id == 1u) ? m1 :
                                     (id == 2u) ? m2 :
                                     (id == 3u) ? m3 : m4;
        if (m->err != 1u)
        {
            DM_Motor_SendMitFrame(id, enable);
        }

        if (id == 2u)
        {
            DM_Motor_MitCtrl(id, 0.0f, GRAV_KD, -GRAV_A2 * cosf(m->pos - GRAV_PHI0_2));
        }
        else if (id == 3u)
        {
            DM_Motor_MitCtrl(id, 0.0f, GRAV_KD, -GRAV_A3 * cosf(m->pos - GRAV_PHI0_3));
        }
        else    /* J1 / J4：回转轴无重力矩 */
        {
            DM_Motor_MitCtrl(id, 0.0f, GRAV_KD, 0.0f);
        }
    }
}

/* MIT 帧（kp=0 纯力矩用法）：只发 kd 阻尼 + t_ff 前馈，pos/vel 填中值无意义。
   重力补偿就走这里，每周期算出 τ_ff 传进来，量程自动查本关节参数表。 */
void DM_Motor_MitCtrl(uint16_t motor_id, float kp, float kd, float tor_ff)
{
    const DMMotor_Params_t *p = get_motor_params((uint8_t)motor_id);
    if (p == 0) return;

    uint8_t data[8];
    uint16_t pos  = 32767;                                        /* 16bit 中值 */
    uint16_t vel  = 2047;                                         /* 12bit 中值 */
    uint16_t kp_i = (uint16_t)float_to_uint(kp,     p->KP_MIN, p->KP_MAX, 12);
    uint16_t kd_i = (uint16_t)float_to_uint(kd,     p->KD_MIN, p->KD_MAX, 12);
    uint16_t t_i  = (uint16_t)float_to_uint(tor_ff, p->T_MIN,  p->T_MAX,  12);

    data[0] = (uint8_t)(pos >> 8);
    data[1] = (uint8_t)(pos & 0xFF);
    data[2] = (uint8_t)(vel >> 4);
    data[3] = (uint8_t)(((vel & 0xF) << 4) | (kp_i >> 8));
    data[4] = (uint8_t)(kp_i & 0xFF);
    data[5] = (uint8_t)(kd_i >> 4);
    data[6] = (uint8_t)(((kd_i & 0xF) << 4) | (t_i >> 8));
    data[7] = (uint8_t)(t_i & 0xFF);

    DM_Motor_SendMitFrame(motor_id, data);
}

/* 达妙电机反馈是“收到控制帧后的响应”，这里周期发 MIT 帧来请求反馈。 */
static void DM_Motor_SendMitFrame(uint16_t motor_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t tx_mailbox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u) return;

    tx_header.StdId = motor_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8u;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &tx_mailbox);
}
