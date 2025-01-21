#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;                // 比例系数
    float output, maxOutput; // 输出、输出限幅
    // float ki, kd;
    // ···
} PID;

// 用于初始化pid参数的函数
void PID_Init(PID *pid, float p, float maxOut);

// 进行一次pid计算
void PID_Calc(PID *pid, float reference, float feedback);

// 设置PID参数
void PID_SetGains(PID *pid, float kp, float ki, float kd);

#endif // PID_H