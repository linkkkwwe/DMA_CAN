#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "struct_typedef.h"

typedef struct {
    CAN_HandleTypeDef *hcan;    // CAN句柄（hcan1/hcan2）
    uint32_t std_id;            // 标准ID
    uint8_t data[8];            // 数据域
    uint8_t len;                // 数据长度（0~8）
} CAN_Frame_t;

typedef struct 
{
    uint8_t id;             // 控制器id
    uint8_t err;            // 故障类型  0失能 1使能 8超压 9欠压 A过流 等等还有B C D E情况
    uint16_t pos_int;    
    uint16_t vel_int;
    uint16_t tor_int;
    float pos;              // 位置信息
    float vel;              // 速度信息
    float tor;              // 扭矩信息
    float T_Mos;            // MOS管上的温度信息
    float T_Rotor;          // 电机线圈的温度信息
} DMMotor_measure_t;

typedef struct {
    float P_MIN;
    float P_MAX;
    float V_MIN;
    float V_MAX;
    float KP_MIN;
    float KP_MAX;
    float KD_MIN;
    float KD_MAX;
    float T_MIN;
    float T_MAX;
} DMMotor_Params_t;

typedef enum
{
    //DM_h3510 id
     CAN_joint1_MOTOR_ID = 0x01,
     CAN_joint2_MOTOR_ID = 0x02,
	 CAN_joint3_MOTOR_ID = 0x03,
	 CAN_joint4_MOTOR_ID = 0x04,
     CAN_jiazhua_MOTOR_ID = 0x05,

} DM_can_msg_id_e;

typedef enum
{
  CAN_joint1_REC_ID = 0x11,//DM4310 id
	CAN_joint2_REC_ID = 0x12,//DMj10010 id
	CAN_joint3_REC_ID = 0x13,//DM_8009 id
  CAN_joint4_REC_ID = 0x14,//DM_6220 id
	CAN_jiazhuan_REC_ID =0x15,
} DM_REC_msg_id_e;

// 控制ID（发送给电机的ID）
#define DM4310_ID        CAN_joint1_MOTOR_ID    // 4310电机（关节1）
#define DM10010_ID       CAN_joint2_MOTOR_ID    // 10010电机（关节2）
#define DM8009_ID        CAN_joint3_MOTOR_ID    // 8009电机（关节3）
#define DM6220_ID        CAN_joint4_MOTOR_ID    // 6220电机（关节4）
#define DMJIAZHUAN_ID    CAN_jiazhua_MOTOR_ID   // 夹爪电机（4310）

extern const DMMotor_Params_t dmmotor_params_table[];
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
const DMMotor_Params_t* get_motor_params(uint8_t motor_id);
const DMMotor_measure_t *get_DM_joint1_motor_measure_point(void);
const DMMotor_measure_t *get_DM_joint2_motor_measure_point(void);
const DMMotor_measure_t *get_DM_joint3_motor_measure_point(void);
const DMMotor_measure_t *get_DM_joint4_motor_measure_point(void);
const DMMotor_measure_t *get_DM_jiazhuan_motor_measure_point(void);

#endif /* CAN_RECEIVE_H */
