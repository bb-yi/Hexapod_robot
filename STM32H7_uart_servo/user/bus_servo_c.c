#include "bus_servo_c.h"
#include "cmsis_os.h"
#include "tool.h"
uint8_t WaitingForReplyTime = 5;

uint8_t servo_uart_rx_data, servo_uart_tx_data;
uint8_t long_rx_data_flag = 0;
uint8_t long_rx_data_num = 0;
uint8_t rxBuffer[10]; // 接收数据的缓冲区

void delay_us_user(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;           // LOAD的值
    ticks = nus * (SystemCoreClock / 1000000); // 需要的节拍数
    tcnt = 0;
    vTaskSuspendAll();   // 阻止OS调度，防止打断us延时
    told = SysTick->VAL; // 刚进入时的计数器值
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; // 这里注意一下SYSTICK是一个递减的计数器就可以了.
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // 时间超过/等于要延迟的时间,则退出.
        }
    };
    xTaskResumeAll(); // 恢复OS调度
}

void bus_servo_delay_ms(uint32_t ms)
{
    osDelay(ms);
}

void UART_Servo_Receive_Enable(void)
{
    if (UART_MODE == 0)
    {
        HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_1);
        HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_1, &servo_uart_rx_data, 1);
    }
    else if (UART_MODE == 1)
    {
        HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_1);
        HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_2);
        HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_3);
        HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_1, &servo_uart_rx_data, 1);
        HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_2, &servo_uart_rx_data, 1);
        HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_3, &servo_uart_rx_data, 1);
    }
}
void delay_2bit(uint16_t a1) // 延时出来2个回复位的时间，约20us
{
    // uint16_t a1 = 128;
    for (;;)
    {
        a1--;
        if (a1 == 0)
            break;
    }
}
void servo_send_data(uint8_t *send_data, uint8_t servo_ID)
{
    if (UART_MODE == 0)
    {
        HAL_HalfDuplex_EnableTransmitter(&BUS_SERVO_UART_HANDLE_1);
        HAL_UART_Transmit(&BUS_SERVO_UART_HANDLE_1, send_data, 10, HAL_MAX_DELAY);
        HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_1);
        HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_1, &servo_uart_rx_data, 1);
    }
    else if (UART_MODE == 1)
    {
        if (servo_ID == 1 || servo_ID == 2 || servo_ID == 3 || servo_ID == 10 || servo_ID == 11 || servo_ID == 12)
        {
            HAL_HalfDuplex_EnableTransmitter(&BUS_SERVO_UART_HANDLE_1);
            HAL_UART_Transmit(&BUS_SERVO_UART_HANDLE_1, send_data, 10, HAL_MAX_DELAY);
            HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_1);
            HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_1, &servo_uart_rx_data, 1);
            // printf("send data to servo 1\r\n");
        }
        else if (servo_ID == 4 || servo_ID == 5 || servo_ID == 6 || servo_ID == 13 || servo_ID == 14 || servo_ID == 15)
        {
            HAL_HalfDuplex_EnableTransmitter(&BUS_SERVO_UART_HANDLE_2);
            HAL_UART_Transmit(&BUS_SERVO_UART_HANDLE_2, send_data, 10, HAL_MAX_DELAY);
            HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_2);
            HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_2, &servo_uart_rx_data, 1);
        }
        else if (servo_ID == 7 || servo_ID == 8 || servo_ID == 9 || servo_ID == 16 || servo_ID == 17 || servo_ID == 18)
        {
            HAL_HalfDuplex_EnableTransmitter(&BUS_SERVO_UART_HANDLE_3);
            HAL_UART_Transmit(&BUS_SERVO_UART_HANDLE_3, send_data, 10, HAL_MAX_DELAY);
            HAL_HalfDuplex_EnableReceiver(&BUS_SERVO_UART_HANDLE_3);
            HAL_UART_Receive_IT(&BUS_SERVO_UART_HANDLE_3, &servo_uart_rx_data, 1);
        }
        else
        {
            HAL_HalfDuplex_EnableTransmitter(&BUS_SERVO_UART_HANDLE_1);
            HAL_UART_Transmit(&BUS_SERVO_UART_HANDLE_1, send_data, 10, HAL_MAX_DELAY);
            HAL_HalfDuplex_EnableTransmitter(&BUS_SERVO_UART_HANDLE_2);
            HAL_UART_Transmit(&BUS_SERVO_UART_HANDLE_2, send_data, 10, HAL_MAX_DELAY);
            HAL_HalfDuplex_EnableTransmitter(&BUS_SERVO_UART_HANDLE_3);
            HAL_UART_Transmit(&BUS_SERVO_UART_HANDLE_3, send_data, 10, HAL_MAX_DELAY);
        }
    }
    // UART_Servo_Receive_Enable();

    // delay_2bit();
}

/**
 * @brief UART接收回调函数
 *
 */
void UART_Servo_Rx_Callback(void)
{
    UART_Servo_Receive_Enable();
    // HAL_UART_Transmit(&huart1, &servo_uart_rx_data, 1, HAL_MAX_DELAY);
    if (servo_uart_rx_data == 0xFA || servo_uart_rx_data == 0xFC)
    {
        long_rx_data_flag = 1;
        rxBuffer[0] = servo_uart_rx_data;
        long_rx_data_num = 1;
    }
    else
    {
        if (long_rx_data_flag == 1)
        {
            rxBuffer[long_rx_data_num] = servo_uart_rx_data;
            long_rx_data_num++;
            if (long_rx_data_num == 10)
            {
                long_rx_data_flag = 0;
                long_rx_data_flag = 0;
                // printf("0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\r\n", rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3], rxBuffer[4], rxBuffer[5], rxBuffer[6], rxBuffer[7], rxBuffer[8], rxBuffer[9]);
            }
        }
    }
}

uint8_t checksum(uint8_t temp[10]) // 计算校验和
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 6; i++)
        sum += temp[i + 2];
    return (uint8_t)(sum & 0xff);
}

/**
 * @brief 获取版本号
 *
 * @param ID
 * @return uint8_t
 */
uint8_t GetVersion(uint8_t ID)
{
    uint8_t send_data[10] = {0xFC, 0XCF, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0XED};
    send_data[2] = ID;                  // 舵机ID
    send_data[8] = checksum(send_data); // 校验和
    memset(rxBuffer, 0, sizeof(rxBuffer));
    servo_send_data(send_data, ID);
    bus_servo_delay_ms(WaitingForReplyTime);

    if ((rxBuffer[4] + rxBuffer[5] + rxBuffer[6] + rxBuffer[7]) != 0 && rxBuffer[0] == 0xFC)
    {
        return (rxBuffer[4] + rxBuffer[5] + rxBuffer[6] + rxBuffer[7]);
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 设置舵机角度 0-240
 *
 * @param ID 舵机ID 0 广播
 * @param angle 目标角度
 * @param time 动作时间 0全速  单位20ms
 */
void Set_UART_Servo_Angle(uint8_t ID, uint8_t angle, uint8_t time) // 舵机控制
{
    uint8_t send_data[10] = {0xFA, 0XAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0XED};
    send_data[2] = ID;                  // 舵机ID
    send_data[3] = 0x01;                // 运动控制命令
    send_data[4] = angle;               // 目标角度
    send_data[5] = time;                // 运动时间
    send_data[6] = 0;                   // 运动完成后多久不接受指令
    send_data[7] = 0;                   // 运动完成后多久不接受指令
    send_data[8] = checksum(send_data); // 校验和
    servo_send_data(send_data, ID);
    // delay_2bit(5000);

    // osDelay(3);
    delay_us_user(300);
    // HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
}

/**-
 * @brief 控制LED
 *
 * @param ID 舵机ID 0 广播
 * @param enable  1打开 0关闭
 */
void Set_LED_Enable(uint8_t ID, uint8_t enable)
{
    uint8_t send_data[10] = {0xFA, 0XAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0XED};
    send_data[2] = ID;   // 舵机ID
    send_data[3] = 0x04; // LED控制命令
    if (enable == 1)
    {
        send_data[4] = 0x00;
    }
    else
    {
        send_data[4] = 0x01;
    }
    send_data[8] = checksum(send_data); // 校验和
    servo_send_data(send_data, ID);
}

/**
 * @brief 获取舵机角度
 *
 * @param ID
 * @return uint8_t 舵机角度 范围0-180
 */
uint8_t Get_Servo_Angle(uint8_t ID)
{
    uint8_t send_data[10] = {0xFA, 0XAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0XED};
    send_data[2] = ID;                  // 舵机ID
    send_data[3] = 0x02;                // 角度查询命令
    send_data[8] = checksum(send_data); // 校验和
    memset(rxBuffer, 0, sizeof(rxBuffer));
    servo_send_data(send_data, ID);
    bus_servo_delay_ms(WaitingForReplyTime);
    if ((rxBuffer[4] + rxBuffer[5] + rxBuffer[6] + rxBuffer[7]) != 0 && rxBuffer[0] == 0xFA)
    {
        return rxBuffer[7];
    }
    else
    {
        return 0;
    }
}

void Set_Servo_ID(uint8_t ID, uint8_t new_ID)
{
    uint8_t send_data[10] = {0xFA, 0XAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0XED};
    send_data[2] = ID;                  // 舵机ID
    send_data[3] = 0xCD;                // 舵机ID修改命令
    send_data[5] = new_ID;              // 新舵机ID
    send_data[8] = checksum(send_data); // 校验和
    servo_send_data(send_data, ID);
}

/**
 * @brief 读取舵机角度偏移
 *
 * @param ID
 * @return float 舵机角度偏移 范围-30~30
 */
float Read_Servo_Angle_Offset(uint8_t ID)
{
    uint8_t send_data[10] = {0xFA, 0XAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0XED};
    send_data[2] = ID;                  // 舵机ID
    send_data[3] = 0xD4;                // 读取角度偏移命令
    send_data[8] = checksum(send_data); // 校验和
    servo_send_data(send_data, ID);
    bus_servo_delay_ms(WaitingForReplyTime);
    if (rxBuffer[3] == 0xAA && rxBuffer[0] == 0xFA)
    {
        int16_t result = (rxBuffer[6] << 8) | rxBuffer[7];
        return (float)result / 3.0f;
    }
    else
    {
        return -100.0f;
    }
}

/**
 * @brief 设置舵机角度偏移
 *
 * @param ID
 * @param angle_offset 范围-30~30
 */
void Set_Servo_angle_offset(uint8_t ID, float angle_offset)
{
    uint8_t send_data[10] = {0xFA, 0XAF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0XED};
    int16_t angle_offset_int = (int16_t)(angle_offset * 3.0f);
    send_data[2] = ID;                             // 舵机ID
    send_data[3] = 0xD2;                           // 设置角度偏移命令
    send_data[6] = (angle_offset_int >> 8) & 0xff; // 偏移高八位
    send_data[7] = angle_offset_int & 0xff;        // 偏移低八位
    send_data[8] = checksum(send_data);            // 校验和
    servo_send_data(send_data, ID);
}
