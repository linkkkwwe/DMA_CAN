#include "can_comm.h"
#include "main.h"
#include "CAN_receive.h"
#include "can.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;

/* 电机变量声明 */
DMMotor_measure_t DMMotor_joint1,DMMotor_joint2,DMMotor_joint3,DMMotor_joint4,DMMotor_jiazhuan;
const DMMotor_Params_t* get_motor_params(uint8_t motor_id);

//达妙电机数据获取
//ptr 放入电机 rx_message放入接收数据的data
#define get_DMMotor_measure(ptr, rx_message)                                                       \
    {                                                                                              \
        (ptr)->id = (uint8_t)((rx_message)[0] & 0x0f);                                       \
        (ptr)->err = (uint8_t)((rx_message)[0] >> 4);                                      \
        (ptr)->pos_int = (uint16_t)((rx_message)[1] << 8 | (rx_message)[2]);           \
        (ptr)->vel_int = (uint16_t)((rx_message)[3] << 4 | (rx_message)[4] >> 4);      \
        (ptr)->tor_int = (uint16_t)(((rx_message)[4] & 0x0f) << 8 | (rx_message)[5]);  \
        (ptr)->T_Mos = (float)((rx_message)[6]);                                             \
        (ptr)->T_Rotor = (float)((rx_message)[7]);                                           \
    }

// -------------------------- 核心：电机参数表（修正语法格式，成员名完整） --------------------------
const DMMotor_Params_t dmmotor_params_table[] = {
[DM4310_ID] = {    // 4310电机（关节1）
				.P_MIN = -12.5f, 
				.P_MAX = 12.5f,  
				.V_MIN = -30.0f,
				.V_MAX = 30.0f,
				.KP_MIN = 0.0f,
				.KP_MAX = 500.0f,
				.KD_MIN = 0.0f,
				.KD_MAX = 5.0f,
				.T_MIN = -10.0f,
				.T_MAX = 10.0f,
		},
    [DM6220_ID] = {    // 6220电机（关节4）- 扭矩范围更大
    	  .P_MIN = -12.5f, 
				.P_MAX = 12.5f,  
				.V_MIN = -45.0f,
				.V_MAX = 45.0f,
				.KP_MIN = 0.0f,
				.KP_MAX = 500.0f,
				.KD_MIN = 0.0f,
				.KD_MAX = 5.0f,
				.T_MIN = -10.0f,
				.T_MAX = 10.0f,
    },
    [DM8009_ID] = {    // 8009电机（关节3）- 速度范围更大
     	  .P_MIN = -12.5f, 
				.P_MAX = 12.5f,  
				.V_MIN = -45.0f,
				.V_MAX = 45.0f,
				.KP_MIN = 0.0f,
				.KP_MAX = 500.0f,
				.KD_MIN = 0.0f,
				.KD_MAX = 5.0f,
				.T_MIN = -30.0f,
				.T_MAX = 30.0f,
    },
    [DM10010_ID] = {   // 10010电机（关节2）- 综合参数
  	    .P_MIN = -12.5f, 
				.P_MAX = 12.5f,  
				.V_MIN = -25.0f,
				.V_MAX = 25.0f,
				.KP_MIN = 0.0f,
				.KP_MAX = 500.0f,
				.KD_MIN = 0.0f,
				.KD_MAX = 5.0f,
				.T_MIN = -50.0f,//200
				.T_MAX = 50.0f,//200
    },
    [DMJIAZHUAN_ID] = { // 夹爪电机（4310）- 扭矩更小
    	  .P_MIN = -12.5f, 
				.P_MAX = 12.5f,  
				.V_MIN = -10.0f,
				.V_MAX = 10.0f,
				.KP_MIN = 0.0f,
				.KP_MAX = 500.0f,
				.KD_MIN = 0.0f,
				.KD_MAX = 5.0f,
				.T_MIN = -2.0f,
				.T_MAX =  2.0f,
    },
};

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/* 达妙电机反馈是“收到控制帧后的响应”，这里周期发零力矩 MIT 帧来请求反馈。
 * 数据段：pos=32767, vel=2047, kp=0, kd=0, tor=2047，电机不出力。 */
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

void DM_Motor_Request_Feedback(void)
{
    static const uint8_t zero_torque[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
    static const uint8_t enable[8]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static uint8_t enable_index = 0u;

    /* 上电后前 4 次调用依次给电机 1~4 发使能帧 */
    if (enable_index < 4u)
    {
        DM_Motor_SendMitFrame((uint16_t)(enable_index + 1u), enable);
        enable_index++;
    }

    for (uint16_t id = 1u; id <= 4u; id++)
    {
        DM_Motor_SendMitFrame(id, zero_torque);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    CAN_Frame_t frame = {0};

    /* 读取FIFO中的数据 */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        frame.hcan = hcan;
        frame.std_id = rx_header.StdId;
        frame.len = rx_header.DLC;
        memcpy(frame.data, rx_data, rx_header.DLC); 
        const DMMotor_Params_t* params;
         switch (rx_data[0] & 0x0F) {
            case CAN_joint1_MOTOR_ID:
                get_DMMotor_measure(&DMMotor_joint1, frame.data);
                params = get_motor_params(DM4310_ID);
                if (params) {
                    DMMotor_joint1.pos = uint_to_float(DMMotor_joint1.pos_int, params->P_MIN, params->P_MAX, 16);
                    DMMotor_joint1.vel = uint_to_float(DMMotor_joint1.vel_int, params->V_MIN, params->V_MAX, 12);
                    DMMotor_joint1.tor = uint_to_float(DMMotor_joint1.tor_int, params->T_MIN, params->T_MAX, 12);
                }
                break;
            case CAN_joint2_MOTOR_ID:
                get_DMMotor_measure(&DMMotor_joint2, frame.data);
                params = get_motor_params(DM10010_ID);
                if (params) {
                    DMMotor_joint2.pos = uint_to_float(DMMotor_joint2.pos_int, params->P_MIN, params->P_MAX, 16);
                    DMMotor_joint2.vel = uint_to_float(DMMotor_joint2.vel_int, params->V_MIN, params->V_MAX, 12);
                    DMMotor_joint2.tor = uint_to_float(DMMotor_joint2.tor_int, params->T_MIN, params->T_MAX, 12);
                }
                break;
            case CAN_joint3_MOTOR_ID:
                get_DMMotor_measure(&DMMotor_joint3, frame.data);
                params = get_motor_params(DM8009_ID);
                if (params) {
                    DMMotor_joint3.pos = uint_to_float(DMMotor_joint3.pos_int, params->P_MIN, params->P_MAX, 16);
                    DMMotor_joint3.vel = uint_to_float(DMMotor_joint3.vel_int, params->V_MIN, params->V_MAX, 12);
                    DMMotor_joint3.tor = uint_to_float(DMMotor_joint3.tor_int, params->T_MIN, params->T_MAX, 12);
                }
							
                break;
            case CAN_joint4_MOTOR_ID:
                get_DMMotor_measure(&DMMotor_joint4, frame.data);
                params = get_motor_params(DM6220_ID);
                if (params) {
                    DMMotor_joint4.pos = uint_to_float(DMMotor_joint4.pos_int, params->P_MIN, params->P_MAX, 16);
                    DMMotor_joint4.vel = uint_to_float(DMMotor_joint4.vel_int, params->V_MIN, params->V_MAX, 12);
                    DMMotor_joint4.tor = uint_to_float(DMMotor_joint4.tor_int, params->T_MIN, params->T_MAX, 12);
                }
								break;
            case CAN_jiazhua_MOTOR_ID:
                get_DMMotor_measure(&DMMotor_jiazhuan, frame.data);
                params = get_motor_params(DMJIAZHUAN_ID);
                if (params) {
                    DMMotor_jiazhuan.pos = uint_to_float(DMMotor_jiazhuan.pos_int, params->P_MIN, params->P_MAX, 16);
                    DMMotor_jiazhuan.vel = uint_to_float(DMMotor_jiazhuan.vel_int, params->V_MIN, params->V_MAX, 12);
                    DMMotor_jiazhuan.tor = uint_to_float(DMMotor_jiazhuan.tor_int, params->T_MIN, params->T_MAX, 12);
                }
                break;
            default: break;
        }
    }
}
const DMMotor_Params_t* get_motor_params(uint8_t motor_id) {
    
       
	  // 增加参数合法性检查，避免数组越界
    if (motor_id >= sizeof(dmmotor_params_table)/sizeof(dmmotor_params_table[0])) {
        return NULL; 
    }  
      return &dmmotor_params_table[motor_id];
}

const DMMotor_measure_t *get_DM_joint1_motor_measure_point() { return &DMMotor_joint1; }
const DMMotor_measure_t *get_DM_joint2_motor_measure_point() { return &DMMotor_joint2; }
const DMMotor_measure_t *get_DM_joint3_motor_measure_point() { return &DMMotor_joint3; }
const DMMotor_measure_t *get_DM_joint4_motor_measure_point() { return &DMMotor_joint4; }
const DMMotor_measure_t *get_DM_jiazhuan_motor_measure_point() { return &DMMotor_jiazhuan; }
