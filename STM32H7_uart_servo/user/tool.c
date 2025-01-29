#include "tool.h"

float loop_now_time;  // 开机时间 毫秒
float loop_last_time; // 上一次循环时间 毫秒
float loop_dt;        // 循环时间间隔 毫秒

#define CPU_FREQUENCY_MHZ 72 // STM32时钟主频
/**
 * @brief微秒级延时函数
 *
 * @param delay
 */
void delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            } while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            } while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}

/**
 * @brief 浮点类型映射范围函数
 *
 * @param input_value
 * @param input_min
 * @param input_max
 * @param output_min
 * @param output_max
 * @return float
 */
float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}

/**
 * @brief 整数类型映射范围函数
 *
 * @param input_value
 * @param input_min
 * @param input_max
 * @param output_min
 * @param output_max
 * @return int
 */
int int_Map(int input_value, int input_min, int input_max, int output_min, int output_max)
{
    int output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}

/**
 * @brief 浮点类型映射范围函数，带有中值分割
 *
 * @param input_value
 * @param input_min
 * @param input_max
 * @param median
 * @param output_min
 * @param output_max
 * @return float
 */
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_value;
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_value < median)
    {
        output_value = float_Map(input_value, input_min, median, output_min, output_median);
        return output_value;
    }
    else
    {
        output_value = float_Map(input_value, median, input_max, output_median, output_max);
        return output_value;
    }
}

/**
 * @brief 钳制范围函数
 *
 * @param value
 * @param min
 * @param max
 * @return float
 */
float clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}

/**
 * @brief 求绝对值函数
 * @param a
 * @return float
 */
float Abs(float a)
{
    if (a >= 0)
        return a;
    else
        return (-a);
}

int int_mod(int a, int b)
{
    int temp = a % b;
    if (temp < 0)
    {
        temp += b;
    }
    return temp;
}

// 角度转弧度的函数
float degreesToRadians(float degrees)
{
    return degrees * (PI / 180.0f);
}

float radiansToDegrees(float radians)
{
    return radians * (180.0f / PI);
}
/**
 * @brief 余弦定理函数
 *
 * @param l 对边
 * @param s1 邻边1
 * @param s2 邻边2
 * @return float 余弦值
 */
float Cosinelaw(float l, float s1, float s2)
{
    float cos_val = (s1 * s1 + s2 * s2 - l * l) / (2 * s1 * s2);
    if (cos_val > 1)
    {
        cos_val = 1;
    }
    else if (cos_val < -1)
    {
        cos_val = -1;
    }
    return cos_val;
}

/**
 * @brief 余弦值转正弦值
 *
 * @param cos_val
 * @return float
 */
float cos2sin(float cos_val)
{
    return sqrt(1 - cos_val * cos_val);
}
#if 1
/**
 * @brief 通用接口，主频80MHz，预分频值为80-1，设置PWM的脉冲频率freq(0.16-10kHz)、占空比参数 pulse (0-100)
 *
 * @param htim pwm时钟句柄
 * @param Channel 通道号
 * @param freq PWM频率
 * @param duty PWM占空比
 */
void set_pwm_param(TIM_HandleTypeDef htim, uint32_t Channel, uint32_t freq, float duty)
{
    uint16_t prescaler = 0;
    uint64_t tim_clk_freq = 168000000;
    // 计算PWM频率，所对应的自动重装载值   ---> ARR = 主频 / (预分频+1) / 预期PWM频率(Hz) - 1
    float pwm_freq_arr = (tim_clk_freq * 1.0) / (prescaler + 1) / freq * 1.0 - 1;
    // 计算PWM占空比，所对应比较寄存器的值 ---> CCR = 预期占空比 * (自动重装载值+1)
    // 占空比则由捕获/比较寄存器（TIMx_CRx）寄存器决定。占空比:duty = Pluse / (ARR+1)
    float pwm_duty_pulse = duty * 1.0f / 100.0f * (pwm_freq_arr + 1.0f);

    // 配置PSC预分频值
    __HAL_TIM_SET_PRESCALER(&htim, prescaler);
    // 配置PWM频率 ARR
    __HAL_TIM_SetAutoreload(&htim, (uint16_t)pwm_freq_arr);
    // 配置PWM占空比
    __HAL_TIM_SetCompare(&htim, Channel, (uint16_t)pwm_duty_pulse);
}
#endif

extern UART_HandleTypeDef huart1;
char dma_printf_buf[DMA_PRINT_MAX];
void dma_print(char *fmt, ...)
{
    if ((&huart1)->gState == HAL_UART_STATE_READY)
    {
        va_list arg_ptr;
        va_start(arg_ptr, fmt);
        vsnprintf(dma_printf_buf, DMA_PRINT_MAX + 1, fmt, arg_ptr);
        va_end(arg_ptr);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&dma_printf_buf, strlen(dma_printf_buf));
    }
}

// DMA发送中断回调函数：DMA发送数据完成之后会进入此函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断哪个串口发送完成
    if (huart == &huart1)
    {
        memset(dma_printf_buf, 0, DMA_PRINT_MAX); // 防止程序跑得过快，导致Data中的数据还没有被DMA发送完成就被memset，一部分数据变成乱码被发送
        // 拉低485使能引脚，继续接收数据
    }
}
/**
 * @brief 输入
 *
 * @param val 0-x
 * @param max x
 * @param scale 缩放
 * @return float 输出-x - x - -x
 */
float value_cycle(float val, float max, float scale)
{
    float temp;
    if (val > max / 2)
    {
        temp = max - val;
    }
    else
    {
        temp = val;
    }
    temp = (temp - (max / 4)) * scale * 4;
    return temp;
}

/**
 * @brief 二次函数路径
 *
 * @param x
 * @param x_max 第二个零点
 * @param output_max 最高点
 * @return float
 */
float ParabolicPath(float x, float x_max, float output_max)
{
    float temp = (-(x * x) + x_max * x) * output_max / x_max / x_max * 4;
    return temp;
}
