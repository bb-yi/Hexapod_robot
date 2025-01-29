#include "hexapod_servo.h"
#include "tool.h"
#include "cmsis_os.h"
#include "bus_servo_c.h"

float get_leg_height(float x, float offset, float gait, float y_max)
{
    float x_new = int_mod(x - offset, 360);
    float output = x_new < 360 / gait ? ParabolicPath(x_new, 360 / gait, y_max) : 0;
    return output;
}
float get_leg_move(float x, float offset, float gait, uint8_t line_back)
{
    float x_new = degreesToRadians(int_mod(x - offset, 360));
    float output;
    if (line_back == 0)
    {
        output = x_new < 2 * PI / gait ? cos(x_new * gait / 2) : cos((x_new - (2 * PI / gait) + 2 * PI * (1 - (1 / gait))) * gait / (2 * (gait - 1)));
    }
    else
    {
        output = x_new < 2 * PI / gait ? cos(x_new * (gait / 2)) : (gait / (PI * (gait - 1))) * x_new - ((gait / (gait - 1)) * 2) + 1;
    }
    return output;
}

void hexapod_servo_init(void)
{
    uint8_t delay_time = 50;
    osDelay(delay_time);
    Set_Servo_angle_offset(1, -5);
    osDelay(delay_time);
    Set_Servo_angle_offset(2, -5);
    osDelay(delay_time);
    Set_Servo_angle_offset(3, 4);

    osDelay(delay_time);
    Set_Servo_angle_offset(4, 8);
    osDelay(delay_time);
    Set_Servo_angle_offset(5, -12);
    osDelay(delay_time);
    Set_Servo_angle_offset(6, -6);

    osDelay(delay_time);
    Set_Servo_angle_offset(7, -5);
    osDelay(delay_time);
    Set_Servo_angle_offset(8, 5);
    osDelay(delay_time);
    Set_Servo_angle_offset(9, 2);

    osDelay(delay_time);
    Set_Servo_angle_offset(10, 5);
    osDelay(delay_time);
    Set_Servo_angle_offset(11, 3);
    osDelay(delay_time);
    Set_Servo_angle_offset(12, 4);

    osDelay(delay_time);
    Set_Servo_angle_offset(13, -1);
    osDelay(delay_time);
    Set_Servo_angle_offset(14, 8);
    osDelay(delay_time);
    Set_Servo_angle_offset(15, -5);

    osDelay(delay_time);
    Set_Servo_angle_offset(16, -1);
    osDelay(delay_time);
    Set_Servo_angle_offset(17, 0);
    osDelay(delay_time);
    Set_Servo_angle_offset(18, 0);
}

void hexapod_start_all_servo(void)
{
}
void hexapod_stop_all_servo(void)
{
    for (uint8_t i = 0; i < 18; i++)
    {
        Get_Servo_Angle(i);
        osDelay(1);
    }
}

/*
舵机ID对应表
        L1  L2  L3 R1  R2  R3
HIP     1   4   7   10  13  16
KNEE    2   5   8   11  14  17
ANKLE   3   6   9   12  15  18
*/
void Set_servo_angle(uint8_t leg, uint8_t joint, float angle)
{
    uint8_t Control_id = leg * 3 + (joint + 1); // 舵机ID
    if (Control_id == 11 || Control_id == 14 || Control_id == 17)
    {
        angle = 180 - angle;
    }
    if (Control_id == 3 || Control_id == 6 || Control_id == 9)
    {
        angle = 180 - angle;
    }
    // if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
    if (1)
    {
        Set_UART_Servo_Angle(Control_id, angle, 0);
    }
    else
    {
        hexapod_stop_all_servo();
    }
}

// 变换位置的函数
void transform_position(float matrix[MATRIX_SIZE][MATRIX_SIZE], float position[3], float result[3])
{
    // 将 3D 位置扩展为 4D 齐次坐标 [x, y, z, 1]
    float homogenous_position[4] = {position[0], position[1], position[2], 1.0f};
    float transformed_position[4] = {0, 0, 0, 0};

    // 应用矩阵变换
    for (int i = 0; i < MATRIX_SIZE; i++)
    {
        for (int j = 0; j < MATRIX_SIZE; j++)
        {
            transformed_position[i] += matrix[i][j] * homogenous_position[j];
        }
    }

    // 输出新的 3D 位置，忽略第 4 个齐次坐标分量
    for (int i = 0; i < 3; i++)
    {
        result[i] = transformed_position[i];
    }
}
// 设置变换矩阵旋转
void SetRotationMatrix(float theta, float matrix[MATRIX_SIZE][MATRIX_SIZE])
{
    theta = degreesToRadians(theta);
    // 旋转部分
    matrix[0][0] = cos(theta);
    matrix[0][1] = -sin(theta);
    matrix[0][2] = 0;
    matrix[0][3] = 0; // 平移x

    matrix[1][0] = sin(theta);
    matrix[1][1] = cos(theta);
    matrix[1][2] = 0;
    matrix[1][3] = 0; // 平移y

    matrix[2][0] = 0;
    matrix[2][1] = 0;
    matrix[2][2] = 1;
    matrix[2][3] = 0; // 平移z

    // 齐次坐标部分
    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
}
float Leg_matrix[6][MATRIX_SIZE][MATRIX_SIZE];

void init_leg_matrix(void)
{
    SetRotationMatrix(-30, Leg_matrix[L1]);
    SetRotationMatrix(-90, Leg_matrix[L2]);
    SetRotationMatrix(-150, Leg_matrix[L3]);
    SetRotationMatrix(30, Leg_matrix[R1]);
    SetRotationMatrix(90, Leg_matrix[R2]);
    SetRotationMatrix(150, Leg_matrix[R3]);
}

// 逆运动学 计算出三个角度
/**
 * @brief
 *
 * @param leg
 * @param x 最大150.25
 * @param y 最大150.25
 * @param z 最大123
 */
void Set_servo_Local_position_IK(uint8_t leg, float position[3])
{
    float x = position[0];
    float y = position[1];
    float z = position[2];
    float theta1, theta2, theta3;
    float r = sqrt(x * x + y * y) - HIP_Length;               // L1端点和目标点的水平距离
    float s = sqrt(r * r + z * z);                            // L1端点和目标点的距离
    float a1 = acos(Cosinelaw(ANKLE_Length, KNEE_Length, s)); // L2和s的夹角
    float a2 = atan2(r, -z);                                  // s和z的夹角
    float a3 = acos(Cosinelaw(s, KNEE_Length, ANKLE_Length)); // L2和L3的夹角
    theta1 = radiansToDegrees(atan2(y, x));
    theta2 = 180.0f - radiansToDegrees(a1) - radiansToDegrees(a2) + KNEE_ANGLE_OFFSET;
    theta3 = 180.0f - radiansToDegrees(a3) + ANKEL_ANGLE_OFFSET;

    Set_servo_angle(leg, HIP, theta1);
    Set_servo_angle(leg, KNEE, theta2);
    // osDelay(1);
    Set_servo_angle(leg, ANKLE, theta3);
    // printf("theta1:%f, theta2:%f, theta3:%f\n", theta1, theta2, theta3);
}

void Set_servo_Global_position_IK(uint8_t leg, float position[3])
{
    float transformed_position[3];
    transform_position(Leg_matrix[leg], position, transformed_position);
    Set_servo_Local_position_IK(leg, transformed_position);
}

void Set_all_leg_Global_position(float position[6][3])
{
    for (int i = 0; i < 6; i++)
    {
        Set_servo_Global_position_IK(i, position[i]);
    }
}

// 混合位置
void MixLegPositions(float leg_position[6][3], float init_position[6][3], float output_position[6][3], float weight)
{
    // 遍历每个位置和坐标，并按权重混合
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            output_position[i][j] = weight * init_position[i][j] + (1.0f - weight) * leg_position[i][j];
        }
    }
}
// 足起点向量
float body_base_vector[6][3] = {
    {-FRONT_LEG_CENTER_DISTANCE / 2, FRONT_BACK_LEG_DISTANCE / 2, 0},
    {-MIDDLE_LEG_CENTER_DISTANCE / 2, 0, 0},
    {-FRONT_LEG_CENTER_DISTANCE / 2, -FRONT_BACK_LEG_DISTANCE / 2, 0},
    {FRONT_LEG_CENTER_DISTANCE / 2, FRONT_BACK_LEG_DISTANCE / 2, 0},
    {MIDDLE_LEG_CENTER_DISTANCE / 2, 0, 0},
    {FRONT_LEG_CENTER_DISTANCE / 2, -FRONT_BACK_LEG_DISTANCE / 2, 0}};
// 初始位置矩阵
#define base_x 110
#define base_y 100
#define base_z 110
float init_position[6][3] = {
    {-base_x, base_y, -base_z},
    {-base_x - 40, 0, -base_z},
    {-base_x, -base_y, -base_z},
    {base_x, base_y, -base_z},
    {base_x + 40, 0, -base_z},
    {base_x, -base_y, -base_z}};
float leg_position[6][3];

void MoveAndRotateBody(float leg_positions[6][3], float dx, float dy, float dz, float roll, float pitch, float yaw)
{
    // 相对地面坐标系平移旋转相反
    float transformed_matrix[MATRIX_SIZE][MATRIX_SIZE];
    roll = -degreesToRadians(roll);
    pitch = -degreesToRadians(pitch);
    yaw = -degreesToRadians(yaw);
    // 计算旋转的三角函数值
    float cosRoll = cos(roll), sinRoll = sin(roll);
    float cosPitch = cos(pitch), sinPitch = sin(pitch);
    float cosYaw = cos(yaw), sinYaw = sin(yaw);

    // 填充旋转矩阵 (结合 roll, pitch, yaw)
    transformed_matrix[0][0] = cosYaw * cosPitch;
    transformed_matrix[0][1] = cosYaw * sinPitch * sinRoll - sinYaw * cosRoll;
    transformed_matrix[0][2] = cosYaw * sinPitch * cosRoll + sinYaw * sinRoll;
    transformed_matrix[0][3] = -dx; // X 平移

    transformed_matrix[1][0] = sinYaw * cosPitch;
    transformed_matrix[1][1] = sinYaw * sinPitch * sinRoll + cosYaw * cosRoll;
    transformed_matrix[1][2] = sinYaw * sinPitch * cosRoll - cosYaw * sinRoll;
    transformed_matrix[1][3] = -dy; // Y 平移

    transformed_matrix[2][0] = -sinPitch;
    transformed_matrix[2][1] = cosPitch * sinRoll;
    transformed_matrix[2][2] = cosPitch * cosRoll;
    transformed_matrix[2][3] = -dz; // Z 平移

    transformed_matrix[3][0] = 0.0f;
    transformed_matrix[3][1] = 0.0f;
    transformed_matrix[3][2] = 0.0f;
    transformed_matrix[3][3] = 1.0f;

    float center2leg[6][3];                    // 初始姿态身体中心到足端点矢量
    float center2leg_vector_transformed[6][3]; // 旋转平移变换后身体中心到足端点矢量
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            center2leg[i][j] = init_position[i][j] + body_base_vector[i][j]; // 将对应项相加
        }
    }
    for (int i = 0; i < 6; i++)
    {
        transform_position(transformed_matrix, center2leg[i], center2leg_vector_transformed[i]);
    }
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            leg_positions[i][j] = center2leg_vector_transformed[i][j] - body_base_vector[i][j]; // 将对应项相加 数组对象作为指针传入,可以直接修改全局变量
        }
    }
}

void smoother_target_position_to_init_position(float leg_position[6][3])
{
    uint8_t cycle = 5;
    float output[6][3];
    for (uint8_t i = 0; i < cycle; i++)
    {
        MixLegPositions(leg_position, init_position, output, (float)(i) / (cycle - 1));
        Set_all_leg_Global_position(output);
        // osDelay(1);
    }
}
void smoother_init_position_to_target_position(float target_position[6][3])
{
    uint8_t cycle = 5;
    float output[6][3];
    for (uint8_t i = 0; i < cycle; i++)
    {
        MixLegPositions(init_position, leg_position, output, (float)(i) / (cycle - 1));
        Set_all_leg_Global_position(output);
        // osDelay(1);
    }
}

void Set_Storage_Leg_Position(void)
{
    uint8_t pos_angle = 60;
    Set_servo_angle(0, HIP, 90 + pos_angle);
    Set_servo_angle(1, HIP, 90);
    Set_servo_angle(2, HIP, 90 - pos_angle);
    Set_servo_angle(3, HIP, 90 - pos_angle);
    Set_servo_angle(4, HIP, 90);
    Set_servo_angle(5, HIP, 90 + pos_angle);
    for (uint8_t i = 0; i < 6; i++)
    {
        Set_servo_angle(i, KNEE, 2);
        Set_servo_angle(i, ANKLE, 160);
    }
}

float leg_group_move_position[6][6][3];
/**
 * @brief 六足运动步态函数 单步
 * @param x x方向步幅 速度
 * @param y y方向步幅 速度
 * @param z z旋转步幅 速度 顺时针为负
 * @param LeggedHeight  抬腿最高点
 * @param gait  步态类型 2,3,6
 * @param i
 */
void HexapodMoveStepGait(float x, float y, float z, float LeggedHeight, uint8_t gait, uint32_t i)
{
    int gait2_leg_map[6];
    float x_move[6], y_move[6], z_move[6];
    float hight[6];
    if (gait != 2 && gait != 3 && gait != 6)
    {
        gait = 2;
    }
    uint32_t time = int_mod(i, 360);
    switch (gait)
    {
    case 2: // 二步态 三角步态 分为两组，组0{0,2,4}，组1{1,3,5}

        for (uint8_t i = 0; i < 2; i++)
        {
            x_move[i] = x * get_leg_move(time, 360 / gait * i, gait, 0);
            y_move[i] = y * get_leg_move(time, 360 / gait * i, gait, 0);
            z_move[i] = z * get_leg_move(time, 360 / gait * i, gait, 0);
            hight[i] = -get_leg_height(time, 360 / gait * i, gait, LeggedHeight);
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            MoveAndRotateBody(leg_group_move_position[i], x_move[i], y_move[i], hight[i], 0, 0, z_move[i]);
        }
        gait2_leg_map[0] = 0;
        gait2_leg_map[1] = 1;
        gait2_leg_map[2] = 0;
        gait2_leg_map[3] = 1;
        gait2_leg_map[4] = 0;
        gait2_leg_map[5] = 1;

        break;
    case 3: // 三步态 四角步态 分为三组，组0{0,5}，组1{1,4},组2{2,3}
        gait2_leg_map[0] = 0;
        gait2_leg_map[1] = 2;
        gait2_leg_map[2] = 1;
        gait2_leg_map[3] = 1;
        gait2_leg_map[4] = 2;
        gait2_leg_map[5] = 0;
        for (uint8_t i = 0; i < 3; i++)
        {
            x_move[i] = x * get_leg_move(time, 360 / gait * i, gait, 0);
            y_move[i] = y * get_leg_move(time, 360 / gait * i, gait, 0);
            z_move[i] = z * get_leg_move(time, 360 / gait * i, gait, 0);
            hight[i] = -get_leg_height(time, 360 / gait * i, gait, LeggedHeight);
        }
        for (uint8_t i = 0; i < 3; i++)
        {
            MoveAndRotateBody(leg_group_move_position[i], x_move[i], y_move[i], hight[i], 0, 0, z_move[i]);
        }

        break;
    case 6: // 六步态 六角步态 分为六组，每组1个足 组0{3}，组1{1}，组2{4}，组3{2}，组4{5}，组5{0}
        gait2_leg_map[0] = 3;
        gait2_leg_map[1] = 1;
        gait2_leg_map[2] = 4;
        gait2_leg_map[3] = 2;
        gait2_leg_map[4] = 5;
        gait2_leg_map[5] = 0;
        for (uint8_t i = 0; i < 6; i++)
        {
            x_move[i] = x * get_leg_move(time, 360 / gait * i, gait, 0);
            y_move[i] = y * get_leg_move(time, 360 / gait * i, gait, 0);
            z_move[i] = z * get_leg_move(time, 360 / gait * i, gait, 0);
            hight[i] = -get_leg_height(time, 360 / gait * i, gait, LeggedHeight);
        }
        for (uint8_t i = 0; i < 6; i++)
        {
            MoveAndRotateBody(leg_group_move_position[i], x_move[i], y_move[i], hight[i], 0, 0, z_move[i]);
        }
        break;
    }
    for (uint8_t i = 0; i < 6; i++)
    {
        memcpy(leg_position[i], leg_group_move_position[gait2_leg_map[i]][i], sizeof(leg_position[i]));
    }
    Set_all_leg_Global_position(leg_position);
    if (i == 0)
    {
        smoother_init_position_to_target_position(leg_position);
    }
    else
    {
        Set_all_leg_Global_position(leg_position);
    }
}

/**
 * @brief 六足运动步态函数 循环
 *
 * @param x x方向步幅 速度
 * @param y y方向步幅 速度
 * @param z z旋转步幅 速度 顺时针为负
 * @param LeggedHeight 抬腿高度
 * @param speed 速度
 * @param gait 步态类型 2,3,6
 * @param Cycle_time 运行周期
 */
void HexapodMove(float x, float y, float z, float LeggedHeight, float speed, uint8_t gait, uint32_t Cycle_time)
{
    // 一个周期360度走gait个z
    uint32_t i = 0;
    for (;;)
    {
        if ((i <= Cycle_time * 360) || (Cycle_time == 0))
        {
            HexapodMoveStepGait(x, y, z, LeggedHeight, gait, i);
        }
        else
        {
            smoother_target_position_to_init_position(leg_position);
            break;
        }
        i = i + speed;
        // osDelay(1);
    }
}
void joint_test(uint8_t joint_id)
{
    uint8_t delay_time = 20;
    for (uint16_t i = 0; i < 90; i++)
    {
        Set_UART_Servo_Angle(joint_id, clamp(i * 2, 45, 135), 0);
        osDelay(delay_time);
    }
    for (uint16_t i = 0; i < 90; i++)
    {
        Set_UART_Servo_Angle(joint_id, clamp(180 - i * 2, 45, 135), 0);
        osDelay(delay_time);
    }
}

void set_test_position(void)
{
    for (int i = 0; i < 6; i++)
    {
        Set_servo_angle(i, HIP, 90);
        Set_servo_angle(i, KNEE, 90);
        Set_servo_angle(i, ANKLE, 30);
    }
}

void Set_servo_Local_position_IK_test(void)
{
    float position[3] = {100, 200, -100};
    for (uint8_t i = 0; i < 180; i++)
    {
        position[2] = 80 * sin(degreesToRadians(i * 2));
        Set_servo_Local_position_IK(0, position);
        osDelay(5);
    }
}

void Set_servo_Global_position_IK_test(void)
{
    float position[3] = {180, 120, -80};
    for (uint8_t i = 0; i < 180; i++)
    {
        position[1] = 0 + 80 * sin(degreesToRadians(i * 2));
        Set_servo_Global_position_IK(4, position);
        osDelay(5);
    }
}

void MoveAndRotateBody_test(void)
{
    uint16_t speed = 2;
    for (uint16_t i = 0; i < 360 / speed; i++)
    {
        uint16_t time = i * speed;
        MoveAndRotateBody(leg_position, 0, 0, 0 * sin(degreesToRadians(time)), 15 * sin(degreesToRadians(time)), 15 * cos(degreesToRadians(time)), 30 * sin(degreesToRadians(time)));
        Set_all_leg_Global_position(leg_position);
    }
}

void HexapodMove_test(void)
{
    float speed = 45;
    float leg_height = 15;
    uint8_t gait = 2;
    float move_size = 30;
    float rot_size = 12;
    uint8_t cycle = 4;
    // osDelay(1000);
    // HexapodMove(0, move_size, 0, leg_height, speed, gait, 3);
    // osDelay(1000);
    // HexapodMove(0, -move_size, 0, leg_height, speed, gait, 3);
    // osDelay(1000);
    // HexapodMove(-0, 0, rot_size, leg_height, speed, gait, 3);
    // osDelay(1000);
    // HexapodMove(0, -0, -rot_size, leg_height, speed, gait, 3);
    // osDelay(1000);
    // gait = 3;
    HexapodMove(move_size / 2, move_size, 0, leg_height, speed, gait, cycle);
    osDelay(1000);
    HexapodMove(-move_size / 2, -move_size, 0, leg_height, speed, gait, cycle);
    osDelay(1000);
    HexapodMove(-0, 0, rot_size, leg_height, speed, gait, cycle);
    osDelay(1000);
    HexapodMove(0, -0, -rot_size, leg_height, speed, gait, cycle);
    osDelay(1000);
    // gait = 6;
    // HexapodMove(0, move_size, 0, leg_height, speed, gait, 3);
    // osDelay(1000);
    // HexapodMove(0, -move_size, 0, leg_height, speed, gait, 3);
    // osDelay(1000);
    // HexapodMove(-0, 0, rot_size, leg_height, speed, gait, 3);
    // osDelay(1000);
    // HexapodMove(0, -0, -rot_size, leg_height, speed, gait, 3);
    // osDelay(1000);
}
void test(void)
{
    for (uint16_t i = 0; i < 360; i++)
    {
        MoveAndRotateBody(leg_position, 0, 0, 30 * sin(degreesToRadians(i)) + 10, 0, 0, 0);
        Set_all_leg_Global_position(leg_position);
        osDelay(5);
    }
    osDelay(1000);
    smoother_target_position_to_init_position(leg_position);
    for (uint16_t i = 0; i < 720; i++)
    {
        MoveAndRotateBody(leg_position, 0, 0, 0, 10 * sin(degreesToRadians(i)), 10 * cos(degreesToRadians(i)), 30 * sin(degreesToRadians(i)));
        Set_all_leg_Global_position(leg_position);
        osDelay(5);
    }
    osDelay(1000);
    smoother_target_position_to_init_position(leg_position);
    for (uint8_t i = 0; i < 180; i++)
    {
        MoveAndRotateBody(leg_position, 20 * sin(degreesToRadians(2 * i)), 20 * cos(degreesToRadians(2 * i)), 0, 0, 0, 0);
        Set_all_leg_Global_position(leg_position);
        osDelay(5);
    }
    osDelay(1000);
    for (uint8_t i = 0; i < 180; i++)
    {
        MoveAndRotateBody(leg_position, 20 * sin(degreesToRadians(-2 * i)), 20 * cos(degreesToRadians(-2 * i)), 0, 0, 0, 0);
        Set_all_leg_Global_position(leg_position);
        osDelay(5);
    }
    osDelay(500);
    smoother_target_position_to_init_position(leg_position);
    osDelay(1000);
    HexapodMove(-10, 15, 0, 10, 2, 2, 5);
    osDelay(1000);
    HexapodMove(10, 15, 0, 10, 2, 2, 5);
    osDelay(1000);
    HexapodMove(0, -15, 0, 10, 2, 2, 5);
    osDelay(1000);
    smoother_target_position_to_init_position(leg_position);
    HexapodMove(0, 0, -5, 15, 2, 2, 4);
    osDelay(1000);
    HexapodMove(0, 0, 5, 15, 2, 2, 4);
    osDelay(1000);
    smoother_target_position_to_init_position(leg_position);
    osDelay(1000);
}