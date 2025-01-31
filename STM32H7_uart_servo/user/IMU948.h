#ifndef __IMU948_H__
#define __IMU948_H__

#include "main.h"

// 传输时转换比例--------------
#define scaleAccel 0.00478515625f         // 加速度 [-16g~+16g]    9.8*16/32768
#define scaleQuat 0.000030517578125f      // 四元数 [-1~+1]         1/32768
#define scaleAngle 0.0054931640625f       // 角度   [-180~+180]     180/32768
#define scaleAngleSpeed 0.06103515625f    // 角速度 [-2000~+2000]    2000/32768
#define scaleMag 0.15106201171875f        // 磁场 [-4950~+4950]   4950/32768
#define scaleTemperature 0.01f            // 温度
#define scaleAirPressure 0.0002384185791f // 气压 [-2000~+2000]    2000/8388608
#define scaleHeight 0.0010728836f         // 高度 [-9000~+9000]    9000/8388608

#define CmdPacket_Begin 0x49     // 起始码
#define CmdPacket_End 0x4D       // 结束码
#define CmdPacketMaxDatSizeRx 73 // 模块发来的   数据包的数据体最大长度
#define CmdPacketMaxDatSizeTx 31 // 发送给模块的 数据包的数据体最大长度

typedef signed short S16;
typedef signed long S32;
typedef unsigned long U32;

typedef struct
{
    uint32_t timestamp; // 时间戳

    // 无重力加速度
    float accel_X;
    float accel_Y;
    float accel_Z;

    // 含重力加速度
    float accel_with_gravity_X;
    float accel_with_gravity_Y;
    float accel_with_gravity_Z;

    // 角速度
    float gyro_X;
    float gyro_Y;
    float gyro_Z;

    // 磁场数据
    float mag_X;
    float mag_Y;
    float mag_Z;

    // 温度值
    float temperature;

    // 气压值
    float airPressure;

    // 高度值
    float height;

    // 四元数
    float quaternion_w;
    float quaternion_x;
    float quaternion_y;
    float quaternion_z;

    // 欧拉角
    float euler_X;
    float euler_Y;
    float euler_Z;

    // 三维位置
    float pos_X;
    float pos_Y;
    float pos_Z;

    // 导航系加速度
    float nav_accel_X;
    float nav_accel_Y;
    float nav_accel_Z;

    // 电压
    float voltage;
    uint32_t last_update_time;

} IMU948_Data;
void pos_to_zero(void);
void AccelerateSimpleCalibration(void);
void IMU_restar(void);
void IMU948_Init(void);
void IMU948_UART_Init(void);
void IMU948_RX_Callback(uint16_t size);
#endif
