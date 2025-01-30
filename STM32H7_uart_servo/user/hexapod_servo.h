#ifndef __HAXEPOD_SERVO_H__
#define __HAXEPOD_SERVO_H__

#include "main.h"
/*
    L3           L2           L1
(ankle_L3)  (ankle_L2)   (ankle_L1)
    |            |            |
(knee_L3)   (knee_L2)     (knee_L1)
    |            |            |
(hip_L3)     (hip_L2)     (hip_L1)
    |            |            |
尾------------------------------ 头
    |            |            |
(hip_R3)     (hip_R2)     (hip_R1)
    |            |            |
(knee_R3)   (knee_R2)   (knee_R1)
    |            |            |
(ankle_R3)  (ankle_R2)   (ankle_R1)
    R3           R2           R1
 */

/*
-------  hip  ------  knee -------  ankle

hip   90°  前方为0°，后方为180°
knee  90° 上方为0°，下方为180°
ankle 30°  上方为0°，下方为180°
*/
#define MATRIX_SIZE 4
#define X 0
#define Y 1
#define Z 2

#define L1 0
#define L2 1
#define L3 2
#define R1 3
#define R2 4
#define R3 5

#define HIP 0
#define KNEE 1
#define ANKLE 2

#define HIP_Length 40.0f // 单位mm
#define KNEE_Length 90.0f
#define ANKLE_Length 150.0f
#define FRONT_LEG_CENTER_DISTANCE 58.71f // 前足中心距
#define MIDDLE_LEG_CENTER_DISTANCE 73.7f // 中足中心距
#define FRONT_BACK_LEG_DISTANCE 101.69f  // 前后足间距
#define ANKEL_ANGLE_OFFSET 30.0f         // 足三0°角度偏移量
#define KNEE_ANGLE_OFFSET 0.0f           // 足二0°角度偏移量

void hexapod_servo_init(void);
void hexapod_start_all_servo(void);
void hexapod_stop_all_servo(void);
void Set_servo_angle(uint8_t leg, uint8_t joint, float angle);

void init_leg_matrix(void);
void Set_servo_Local_position_IK(uint8_t leg, float position[3]);
void Set_servo_Global_position_IK(uint8_t leg, float position[3]);
void Set_all_leg_Global_position(float position[6][3]);
void MoveAndRotateBody(float leg_positions[6][3], float dx, float dy, float dz, float roll, float pitch, float yaw);
void smoother_to_init_position(float leg_position[6][3]);
void Set_Storage_Leg_Position(void);
void HexapodMoveStepGait(float x, float y, float z, float LeggedHeight, uint8_t gait, uint32_t i);

void HexapodMove(float x, float y, float z, float LeggedHeight, float speed, uint8_t gait, uint32_t Cycle_time);

void set_test_position(void);

void elrs_Control(void);
void joint_test(uint8_t joint_id);
void Set_servo_Local_position_IK_test(void);
void Set_servo_Global_position_IK_test(void);
void MoveAndRotateBody_test(void);
void HexapodMove_test(void);
void test(void);
#endif