#include "IMU948.h"
#include "usart.h"

#define IMU948_HANDLE huart3
extern DMA_HandleTypeDef hdma_usart3_rx;
uint8_t targetDeviceAddress = 255; // 通信地址，设为0-254指定则设备地址，设为255则不指定设备(即广播), 当需要使用485总线形式通信时通过该参数选中要操作的设备，若仅仅是串口1对1通信设为广播地址255即可

// 计算数据包的校验和
uint8_t CalcSum1(uint8_t *Buf, int Len)
{
    uint8_t Sum = 0;
    while (Len-- > 0)
    {
        Sum += Buf[Len];
    }
    return Sum;
}
void Cmd_Write(uint8_t *pBuf, int Len)
{
    // 通过UART_Write函数发送通信数据流，由用户针对底层硬件实现UART_Write函数把buf指针指向的Len字节数据发送出去即可
    HAL_UART_Transmit(&IMU948_HANDLE, pBuf, Len, 1000);
}
/**
 * 发送CMD命令
 *
 * @param pDat 要发送的数据体
 * @param DLen 数据体的长度
 *
 * @return int 0=成功, -1=失败
 */
int Cmd_PackAndTx(uint8_t *pDat, uint8_t DLen)
{
    uint8_t buf[50 + 5 + CmdPacketMaxDatSizeTx] =
        {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff}; // 发送包缓存 开头50字节是前导码，用于唤醒可能处于睡眠状态的模块

    if ((DLen == 0) || (DLen > CmdPacketMaxDatSizeTx) || (pDat == NULL))
    { // 非法参数
        return -1;
    }

    buf[50] = CmdPacket_Begin;                     // 起始码
    buf[51] = targetDeviceAddress;                 // 目前设备地址码
    buf[52] = DLen;                                // 长度
    memcpy(&buf[53], pDat, DLen);                  // 数据体
    buf[53 + DLen] = CalcSum1(&buf[51], DLen + 2); // CS 从 地址码开始算到数据体结束
    buf[54 + DLen] = CmdPacket_End;                // 结束码

    Cmd_Write(buf, DLen + 55);
    return 0;
}
// 唤醒传感器
void Cmd_03(void)
{
    uint8_t buf[1] = {0x03};
    Cmd_PackAndTx(buf, 1);
}
/**
 * 设置设备参数
 * @param accStill    惯导-静止状态加速度阀值 单位dm/s?
 * @param stillToZero 惯导-静止归零速度(单位cm/s) 0:不归零 255:立即归零
 * @param moveToZero  惯导-动态归零速度(单位cm/s) 0:不归零
 * @param isCompassOn 1=需融合磁场 0=不融合磁场
 * @param barometerFilter 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
 * @param reportHz 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
 * @param gyroFilter    陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
 * @param accFilter     加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
 * @param compassFilter 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
 * @param Cmd_ReportTag 功能订阅标识
 */
void Cmd_12(uint8_t accStill, uint8_t stillToZero, uint8_t moveToZero, uint8_t isCompassOn, uint8_t barometerFilter, uint8_t reportHz, uint8_t gyroFilter, uint8_t accFilter, uint8_t compassFilter, uint16_t Cmd_ReportTag)
{
    uint8_t buf[11] = {0x12};
    buf[1] = accStill;
    buf[2] = stillToZero;
    buf[3] = moveToZero;
    buf[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1); // bit[2-1]: BMP280的滤波等级[取值0-3]   bit[0]: 1=已开启磁场 0=已关闭磁场
    buf[5] = reportHz;
    buf[6] = gyroFilter;
    buf[7] = accFilter;
    buf[8] = compassFilter;
    buf[9] = Cmd_ReportTag & 0xff;
    buf[10] = (Cmd_ReportTag >> 8) & 0xff;
    Cmd_PackAndTx(buf, 11);
}
// 加速计简易校准 模块静止在水平面时，发送该指令并收到回复后等待9秒即可
void AccelerateSimpleCalibration(void)
{
    uint8_t buf[1] = {0x07};
    Cmd_PackAndTx(buf, 1);
}
// 恢复默认的自身坐标系Z轴指向及恢复默认的世界坐标系
void Cmd_08(void)
{
    uint8_t buf[1] = {0x08};
    Cmd_PackAndTx(buf, 1);
}
// 计步数清零
void Cmd_16(void)
{
    uint8_t buf[1] = {0x16};
    Cmd_PackAndTx(buf, 1);
}
// 开启数据主动上报
void Cmd_19(void)
{
    uint8_t buf[1] = {0x19};
    Cmd_PackAndTx(buf, 1);
}
// 惯导三维空间位置清零
void pos_to_zero(void)
{
    uint8_t buf[1] = {0x13};
    Cmd_PackAndTx(buf, 1);
}

// 设备重启
void IMU_restar(void)
{
    uint8_t buf[1] = {0x2A};
    Cmd_PackAndTx(buf, 1);
}
IMU948_Data imu948_Data;
void IMU948_Init(void)
{
    IMU948_UART_Init();
    int i = 10000;
    while (i--)
        ; // 延时一下让传感器上电准备完毕，传感器上电后需要初始化完毕后才会接收指令的
    imu948_Data.last_update_time = 0;
    Cmd_03();
    Cmd_12(1, 0, 0, 0, 3, 50, 2, 5, 5, 0xFFF); // 2 设置设备参数(内容1)
    Cmd_19();                                  // 3 开启数据主动上报
    pos_to_zero();                             // 位置归零
    Cmd_16();                                  // 计步数清零
}

uint8_t IMU948_RX_BUF[78];
void IMU948_UART_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&IMU948_HANDLE, IMU948_RX_BUF, sizeof(IMU948_RX_BUF)); // 启用空闲中断接收
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);                                   // 关闭DMA传输过半中断
}

void print_uint8_array(uint8_t arr[], int length)
{
    for (int i = 0; i < length; i++)
    {
        printf("%02X ", arr[i]); // %u 用于打印无符号整数
    }
    printf("\n");
}

void IMU948_RX_Callback(uint16_t Size)
{
    uint16_t ctl;
    uint8_t L;
    U32 tmpU32;
    IMU948_UART_Init();
    // HAL_UART_Transmit(&huart1, IMU948_RX_BUF, Size, HAL_MAX_DELAY);
    // print_uint8_array(IMU948_RX_BUF, Size);//print导致接收中断
    imu948_Data.last_update_time = HAL_GetTick();
    if (IMU948_RX_BUF[0] == CmdPacket_Begin)
    {
        switch (IMU948_RX_BUF[3])
        {
        case 0x11:
            ctl = ((uint16_t)IMU948_RX_BUF[5] << 8) | IMU948_RX_BUF[4]; // 字节[2-1] 为功能订阅标识，指示当前订阅了哪些功能
            imu948_Data.timestamp = (uint32_t)(((uint32_t)IMU948_RX_BUF[9] << 24) | ((uint32_t)IMU948_RX_BUF[8] << 16) | ((uint32_t)IMU948_RX_BUF[7] << 8) | ((uint32_t)IMU948_RX_BUF[6] << 0));
            L = 10; // 从第7字节开始根据 订阅标识tag来解析剩下的数据
            if ((ctl & 0x0001) != 0)
            { // 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
                imu948_Data.accel_X = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAccel;
                L += 2;
                imu948_Data.accel_Y = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAccel;
                L += 2;
                imu948_Data.accel_Z = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAccel;
                L += 2;
            }
            if ((ctl & 0x0002) != 0)
            { // 加速度xyz 包含了重力 使用时需*scaleAccel m/s
                imu948_Data.accel_with_gravity_X = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAccel;
                L += 2;
                imu948_Data.accel_with_gravity_Y = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAccel;
                L += 2;
                imu948_Data.accel_with_gravity_Z = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAccel;
                L += 2;
            }
            if ((ctl & 0x0004) != 0)
            { // 角速度xyz 使用时需*scaleAngleSpeed °/s
                imu948_Data.gyro_X = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAngleSpeed;
                L += 2;
                imu948_Data.gyro_Y = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAngleSpeed;
                L += 2;
                imu948_Data.gyro_Z = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAngleSpeed;
                L += 2;
            }
            if ((ctl & 0x0008) != 0)
            { // 磁场xyz 使用时需*scaleMag uT
                imu948_Data.mag_X = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleMag;
                L += 2;
                imu948_Data.mag_Y = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleMag;
                L += 2;
                imu948_Data.mag_Z = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleMag;
                L += 2;
            }
            if ((ctl & 0x0010) != 0)
            { // 温度 气压 高度
                imu948_Data.temperature = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleTemperature;
                L += 2;
                tmpU32 = (U32)(((U32)IMU948_RX_BUF[L + 2] << 16) | ((U32)IMU948_RX_BUF[L + 1] << 8) | (U32)IMU948_RX_BUF[L]);
                tmpU32 = ((tmpU32 & 0x800000) == 0x800000) ? (tmpU32 | 0xff000000) : tmpU32; // 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                imu948_Data.airPressure = (S32)tmpU32 * scaleAirPressure;
                L += 3;

                tmpU32 = (U32)(((U32)IMU948_RX_BUF[L + 2] << 16) | ((U32)IMU948_RX_BUF[L + 1] << 8) | (U32)IMU948_RX_BUF[L]);
                tmpU32 = ((tmpU32 & 0x800000) == 0x800000) ? (tmpU32 | 0xff000000) : tmpU32; // 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                imu948_Data.height = (S32)tmpU32 * scaleHeight;
                L += 3;
            }
            if ((ctl & 0x0020) != 0)
            { // 四元素 wxyz 使用时需*scaleQuat
                imu948_Data.quaternion_w = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleQuat;
                L += 2;
                imu948_Data.quaternion_x = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleQuat;
                L += 2;
                imu948_Data.quaternion_y = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleQuat;
                L += 2;
                imu948_Data.quaternion_z = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleQuat;
                L += 2;
            }
            if ((ctl & 0x0040) != 0)
            { // 欧拉角xyz 使用时需*scaleAngle
                imu948_Data.euler_X = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAngle;
                L += 2;
                imu948_Data.euler_Y = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAngle;
                L += 2;
                imu948_Data.euler_Z = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]) * scaleAngle;
                L += 2;
                imu948_Data.euler_X_Correct = imu948_Data.euler_X - imu948_Data.euler_X_offset;
                imu948_Data.euler_Y_Correct = imu948_Data.euler_Y - imu948_Data.euler_Y_offset;
                imu948_Data.euler_Z_Correct = imu948_Data.euler_Z - imu948_Data.euler_Z_offset;
            }
            if ((ctl & 0x0080) != 0)
            { // xyz 空间位移 单位mm
                imu948_Data.pos_X = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]);
                L += 2;
                imu948_Data.pos_Y = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]);
                L += 2;
                imu948_Data.pos_Z = (S16)(((S16)IMU948_RX_BUF[L + 1] << 8) | IMU948_RX_BUF[L]);
                L += 2;
            }
            if ((ctl & 0x0100) != 0)
            { // 活动检测数据
                L += 4;
                L += 1;
            }
            if ((ctl & 0x0200) != 0)
            { // 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
                L += 2;
                L += 2;
                L += 2;
            }
            if ((ctl & 0x0400) != 0)
            { // ADC的值
                imu948_Data.voltage = (float)(uint16_t)(((uint16_t)IMU948_RX_BUF[L + 1] << 8) | ((uint16_t)IMU948_RX_BUF[L] << 0));
                L += 2;
            }
            if ((ctl & 0x0800) != 0)
            { // GPIO1的值
                L += 1;
            }
            break;
        default:
            break;
        }
        memset(IMU948_RX_BUF, 0, sizeof(IMU948_RX_BUF));
        // printf("euler X=%.2f,euler Y=%.2f,euler Z=%.2f,aX=%.2f,aY=%.2f,aZ=%.2f,X=%.1f,Y=%.1f,Z=%.1f,wendu=%.2f,timestamp=%d\r\n", imu948_Data.euler_X, imu948_Data.euler_Y, imu948_Data.euler_Z, imu948_Data.accel_X, imu948_Data.accel_Y, imu948_Data.accel_Z, imu948_Data.pos_X, imu948_Data.pos_Y, imu948_Data.pos_Z, imu948_Data.temperature, imu948_Data.timestamp);
    }
}
void IMU948_euler_to_zero(void)
{
    imu948_Data.euler_X_offset = imu948_Data.euler_X;
    imu948_Data.euler_Y_offset = imu948_Data.euler_Y;
    imu948_Data.euler_Z_offset = imu948_Data.euler_Z;
}
