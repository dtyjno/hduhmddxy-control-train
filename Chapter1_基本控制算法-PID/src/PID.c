#include "PID.h"
// 首先在头文件定义PID结构体用于存放一个PID的数据

// 用于初始化pid参数的函数
void PID_Init(PID *pid, float p, float maxOut)
{
    pid->kp = p;
    pid->maxOutput = maxOut;
}

// 进行一次pid计算
// 计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid, float reference, float feedback)
{
    pid->output = pid->kp * (reference - feedback);
    // 取限幅
    if (pid->output > pid->maxOutput)
    {
        pid->output = pid->maxOutput;
    }
    else if (pid->output < -pid->maxOutput)
    {
        pid->output = -pid->maxOutput;
    }
}

void PID_SetGains(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    (void)ki;
    (void)kd;
    // pid->ki = ki;
    // pid->kd = kd;
}