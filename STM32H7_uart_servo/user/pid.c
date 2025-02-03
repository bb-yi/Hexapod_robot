#include "pid.h"
float PID_Control(pid *pid_ctrl, float Angle_Err, float i_max)
{
    float PID_Out;
    // 更新当前误差
    pid_ctrl->err = Angle_Err;
    // 低通滤波器
    // 更新积分部分
    pid_ctrl->integral += pid_ctrl->err;
    // printf("integral=%f\n", pid_ctrl->integral);
    // 计算PID输出
    PID_Out = pid_ctrl->Kp * pid_ctrl->err +
              pid_ctrl->Ki * pid_ctrl->integral +
              pid_ctrl->Kd * (pid_ctrl->err - pid_ctrl->err_last);

    // 防止积分项过大（积分饱和）
    if (pid_ctrl->integral > i_max)
    {
        pid_ctrl->integral = i_max;
    }
    else if (pid_ctrl->integral < -i_max)
    {
        pid_ctrl->integral = -i_max;
    }

    // 更新上一次误差
    pid_ctrl->err_last = pid_ctrl->err;

    return PID_Out;
}
void PID_Init(pid *pid_ctrl, float Kp, float Ki, float Kd)
{
    pid_ctrl->Kp = Kp;
    pid_ctrl->Ki = Ki;
    pid_ctrl->Kd = Kd;
    pid_ctrl->err = 0.0;
    pid_ctrl->err_last = 0.0;
    pid_ctrl->integral = 0.0;
}
