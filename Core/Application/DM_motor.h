#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "main.h"
#include "struct_typedef.h"
#include "CAN_receive.h"

/* ==================== 重力补偿标定区 ====================
 * J1(腰)/J4(腕) 是回转轴，重力矩恒为 0，只给阻尼防乱飘。
 * J2(肩)/J3(肘) 用 τ_ff = −A·cos(pos − PHI0) 前馈补偿：
 *   标定方法：连杆摆到水平位置，读电机 pos 填进 PHI0；
 *   A 从 0 起逐步加大，直到水平位置松手不下垂为止。
 *   H3510 额定 0.18Nm —— J2 的 A 不得超过 0.15（持续输出留余量，
 *   超额定会发热；J2 结构上让 J3/J4 电机贴近转轴安装可减小 A）。
 *   若上电后臂往反方向倒，把对应 A 改成负号（安装方向决定）。 */
#define GRAV_KD       0.2f    /* 各关节阻尼，大了手感发涩，0.05~0.3 之间试 */
#define GRAV_A2       0.0f    /* J2 重力补偿系数（标定后填，上限 0.15） */
#define GRAV_PHI0_2   0.0f    /* J2 连杆水平时的 pos（rad） */
#define GRAV_A3       0.0f    /* J3 重力补偿系数（标定后填） */
#define GRAV_PHI0_3   0.0f    /* J3 连杆水平时的 pos（rad） */

void DM_Motor_Request_Feedback(void);
void DM_Motor_MitCtrl(uint16_t motor_id, float kp, float kd, float tor_ff);

#endif /* DM_MOTOR_H */
