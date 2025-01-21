// extern "C" // C++中调用C语言函数
// {
#include "PID.h"
// }

#include "pso.h"
#include "FuzzyPID.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

// 根据是否定义，选择是否启用功能
#define TEST // 启用
// #define PSO_DEBUG_PRINT // 是否打印调试信息
//// #define PSO_SETRULES // 不使用优化算法优化模糊控制器
#define PSO_SETPID // 是否优化算法优化PID控制器
// #define PSO_OPTIMIZE_DIFF             // 是否启用优化算法
#define PSO_OPTIMIZE_UPDATE_PID_GAINS // 是否更新PID参数

// 定义物体运动参数，尝试自行调整
#define PSO_MAX_ACCELERATION 5.0 // 最大加速度(无单位)
#define PSO_MAX_VELOCITY 2.0     // 最大速度（m/s）
#define PSO_DELTA_T 0.3          // 时间间隔（s）
#define PSO_ADD_ACC 1.5          // 对物体施加的额外加速度
#define TARGET 10                // 目标位置

// 定义PSO参数
#define PSO_MAX_ITERATIONS 100 // 最大迭代次数
#define PSO_MAX_PARTICLES 500  // 最大粒子数

using namespace std;

// 定义系统状态
struct PSOState
{
    double position;
    double velocity;
    double acceleration;
};

//-----------------------------------PID-----------------------------------
// 运行程序，输出位置在TARGET+0.2来回运动，找出无法稳定在TARGET位置的原因
// 在首次到达目标后，位置11.1，出现了超调，如何解决超调问题
// 给出解决方案：更改示例pid.c与pid.h文件，引入积分与微分项，
// 试着调整PID参数，使得位置在TARGET+0.1来回运动，最终稳定在TARGET位置
// 还有没有更佳的解决方案？

// 定义PID初始化
void Get_PID_Init(PID *pid)
{
    // 定义初始PID参数
    // 比例系数kp = 5，最大输出为1
    PID_Init(pid, 5, 1);
}

// 定义PID计算
PSOState Get_PID_Calc(PID *pid, float reference, float feedback)
{
    PSOState state;
    PID_Calc(pid, reference, feedback);
    // 将返回的速度输出赋给state（state.velocity = 。。。)
    state.velocity = pid->output;
    return state;
}

// 设置PID参数
void Set_PID_Gains(PID *pid, float kp, float ki, float kd)
{
    // 设置PID参数
    PID_SetGains(pid, kp, ki, kd);
}

// 打印PID信息
void Print_PID_Update_Info(PID *pid)
{
    printf("kp: %f\n", pid->kp);
}

//-----------------------------------PIDEND-----------------------------------

// 定义系统动力学
PSOState systemDynamics(const PSOState &state, PSOState control_input, double dt)
{
    PSOState new_state;
    new_state.acceleration = (control_input.velocity - state.velocity) / dt + PSO_ADD_ACC;
    new_state.velocity = state.velocity + new_state.acceleration * dt;
    new_state.position = state.position + state.velocity * dt;
    return new_state;
}

// 定义适应度函数
// 可以尝试更改的目标性能指标：ITAE = ∫t|e(t)|dt
double fitnessFunction(FuzzyPID &fuzzypidcontroller, PID &pid, const double &measurement, const double &setpoint, double dt)
{
    const double max_acceleration = PSO_MAX_ACCELERATION; // 最大加速度 =
    PSOState state = {measurement, 0.0, 0.0};
    double total_error = 0.0;
    // float vel_max_num = (setpoint - measurement) / PSO_MAX_VELOCITY / dt;
    bool vel_max = false;
    int reach_count = 0;
    bool direction = (setpoint - measurement) > 0;
    // controller.set_pid_info();

    float kp = 0, ki = 0, kd = 0;
    for (int i = 0; i < setpoint / PSO_MAX_VELOCITY / dt * 6; ++i)
    {
#ifndef TEST
        PSOState controller_output = {0, 0, 0};
        controller.get_pid(kp, ki, kd);                                                     // 获取内置PID参数
        fuzzypidcontroller.fuzzy_pid_control(state.position, setpoint, 0, kp, ki, kd, 2.0); // 模糊PID控制，更新模糊控制器kp，ki，kd输出
        controller.set_gains(kp, ki, kd);                                                   // 设置PID参数，用于计算输出
        controller_output.velocity = controller.update_all(state.position, setpoint, dt, PSO_MAX_VELOCITY);
#ifdef PSO_DEBUG_PRINT
        controller.print_update_info();
#endif
#else
        PSOState controller_output = {0, 0, 0};
        controller_output = Get_PID_Calc(&pid, setpoint, state.position);
#endif
        // 更新系统状态
        state = systemDynamics(state, controller_output, dt);
        // vel_max: 速度是否逼近最大
        if (controller_output.velocity >= PSO_MAX_VELOCITY * 0.7)
        {
            vel_max = true;
        }
        else
        {
            vel_max = false;
        }
        // 判断是否到达目标，如果连续20次到达目标，则停止
        if (abs(setpoint - state.position) < 0.001)
        {
            reach_count++;
            // total_error -= 10;
            if (reach_count > 30)
            {
                total_error -= 1000;
                break;
            }
        }
        else
        {
            reach_count = 0;
        }
        // 计算适应度
        total_error += pow(
            (setpoint - state.position) *
                (vel_max ? 1 : 2) *                                              // 接近目标时，权值增大
                (direction == (abs(setpoint - state.position) < 0.05) ? 1 : 2) * // 超调时，权值增大
                (direction == (abs(setpoint - state.position) < 0.1) ? 1 : 2) *  // 超调时，权值增大
                (direction == (abs(setpoint - state.position) < 0.2) ? 1 : 2),   // 超调时，权值增大
            2);
        if (max_acceleration < abs(state.acceleration) && !vel_max) // 超过最大加速度且未到达最大速度，适应度减小
        {
            total_error += 100000; // 超过最大加速度，适应度减小
            // break;
        }
        // total_error += pow(setpoint - state.position, 2);

        // printf("setpoint - state.position = %lf\n", setpoint - state.position);
        // printf("result = %lf\n", pow((setpoint - state.position) * (vel_max ? 0.06 : 1), 2));

        // printf("result1 = %lf\n", (setpoint - state.position) * (i < 25 ? 0.1 : 1.0) * (setpoint - state.position) * (i < 25 ? 0.1 : 1.0));
    }
    return total_error;
}

// 粒子群优化算法
void particleSwarmOptimization(FuzzyPID &controller, PID &pid, float start_position, float desired_position, double dt)
{
    // PSO 参数
    const int num_parameters = qf_default * qf_default;
    const int num_rules = 3;
    const int num_particles = PSO_MAX_PARTICLES;
    const int num_iterations = PSO_MAX_ITERATIONS;
    const double w = 0.2;  // 惯性权重
    const double c1 = 0.6; // 个体学习因子
    const double c2 = 0.6; // 群体学习因子

    // 初始化粒子
    vector<vector<double>> particles(num_particles, vector<double>(num_parameters * num_rules + 3, 0.0));
    vector<vector<double>> velocities(num_particles, vector<double>(num_parameters * num_rules + 3, 0.0));
    vector<vector<double>> personal_best_positions = particles;
    vector<double> personal_best_scores(num_particles, numeric_limits<double>::max());
    vector<double> global_best_position(num_parameters * num_rules, 0.0);
    double global_best_score = numeric_limits<double>::max();

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(-1.0, 1.0);

    for (auto &particle : particles)
    {
        for (auto &value : particle)
        {
            value = dis(gen);
        }
        particle[num_parameters * num_rules] = abs(dis(gen));
        particle[num_parameters * num_rules + 1] = abs(dis(gen));
        particle[num_parameters * num_rules + 2] = abs(dis(gen));
    }

    // PSO 迭代
    for (int iter = 0; iter < num_iterations; ++iter)
    {
        for (int i = 0; i < num_particles; ++i)
        {
            // 设置模糊控制器参数
            for (int j = 0; j < num_rules; ++j)
            {
                for (int k = 0; k < num_parameters; ++k)
                {
#ifdef PSO_SETRULES
                    controller.set_rule_base(j, k, particles[i][j * num_parameters + k]);
#endif
                }
            }
#ifdef PSO_SETPID
#ifndef TEST
            pid.set_pid(particles[i][num_parameters * num_rules], particles[i][num_parameters * num_rules + 1], particles[i][num_parameters * num_rules + 2]);
#else
#ifdef PSO_OPTIMIZE_UPDATE_PID_GAINS
            Set_PID_Gains(&pid, particles[i][num_parameters * num_rules], particles[i][num_parameters * num_rules + 1], particles[i][num_parameters * num_rules + 2]);
#endif
#endif
#endif
            // 计算适应度

            double score = fitnessFunction(controller, pid, start_position, desired_position, dt);
            score += fitnessFunction(controller, pid, start_position, -desired_position, dt);

            // 更新个体最优
            if (score < personal_best_scores[i])
            {
                personal_best_scores[i] = score;
                personal_best_positions[i] = particles[i];
            }

            // 更新全局最优
            if (score < global_best_score)
            {
                global_best_score = score;
                global_best_position = particles[i];
            }
#ifdef PSO_DEBUG_PRINT
            if (iter % 100 == 0)
                printf("i: %d, score: %f \n", i, score);
#endif
        }

        // 更新粒子速度和位置
        for (int i = 0; i < num_particles; ++i)
        {
            for (int j = 0; j < num_parameters * num_rules + 3; ++j)
            {
                velocities[i][j] = w * velocities[i][j] +
                                   c1 * dis(gen) * (personal_best_positions[i][j] - particles[i][j]) +
                                   c2 * dis(gen) * (global_best_position[j] - particles[i][j]);
                particles[i][j] += velocities[i][j];
            }
        }
        if (iter % 100 == 0)
            printf("iter: %d, score: %f\n", iter + 1, global_best_score);
    }

    // 使用全局最优参数更新模糊控制器
    for (int i = 0; i < num_rules; ++i)
    {
        for (int j = 0; j < num_parameters; ++j)
        {
#ifdef PSO_SETRULES
            controller.set_rule_base(i, j, global_best_position[i * num_parameters + j]);
#endif
        }
    }
#ifdef PSO_SETPID
#ifndef TEST
    pid.set_pid(global_best_position[num_parameters * num_rules], global_best_position[num_parameters * num_rules + 1], global_best_position[num_parameters * num_rules + 2]);
    float kp = global_best_position[num_parameters * num_rules], ki = global_best_position[num_parameters * num_rules + 1], kd = global_best_position[num_parameters * num_rules + 2];
    controller.fuzzy_pid_control(0, 0, 0, kp, ki, kd, 2.0);
#else
#ifdef PSO_OPTIMIZE_UPDATE_PID_GAINS
    Set_PID_Gains(&pid, global_best_position[num_parameters * num_rules], global_best_position[num_parameters * num_rules + 1], global_best_position[num_parameters * num_rules + 2]);
#endif
#endif
#endif
}

int main()
{
    // PID 控制器
    PID pid;
    Get_PID_Init(&pid);

    // 默认的模糊规则库
    float rule_base[][qf_default] = {
        // delta kp 规则库
        {PB, PB, PM, PM, PS, ZO, ZO},
        {PB, PB, PM, PS, PS, ZO, NS},
        {PM, PM, PM, PS, ZO, NS, NS},
        {PM, PM, PS, ZO, NS, NM, NM},
        {PS, PS, ZO, NS, NS, NM, NM},
        {PS, ZO, NS, NM, NM, NM, NB},
        {ZO, ZO, NM, NM, NM, NB, NB},
        // delta ki 规则库
        {NB, NB, NM, NM, NS, ZO, ZO},
        {NB, NB, NM, NS, NS, ZO, ZO},
        {NB, NM, NS, NS, ZO, PS, PS},
        {NM, NM, NS, ZO, PS, PM, PM},
        {NM, NS, ZO, PS, PS, PM, PB},
        {ZO, ZO, PS, PS, PM, PB, PB},
        {ZO, ZO, PS, PM, PM, PB, PB},
        // delta kd 规则库
        {PS, NS, NB, NB, NB, NM, PS},
        {PS, NS, NB, NM, NM, NS, ZO},
        {ZO, NS, NM, NM, NS, NS, ZO},
        {ZO, NS, NS, NS, NS, NS, ZO},
        {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
        {PB, PS, PS, PS, PS, PS, PB},
        {PB, PM, PM, PM, PS, PS, PB}};

    // 默认的模糊函数参数（membership function parameters）
    float mf_params[4 * qf_default] = {-3, -3, -2, 0,
                                       -3, -2, -1, 0,
                                       -2, -1, 0, 0,
                                       -1, 0, 1, 0,
                                       0, 1, 2, 0,
                                       1, 2, 3, 0,
                                       2, 3, 3, 0};

    struct FuzzyPID::Fuzzy_params fuzzy_params[8] = {
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
        {4, 1, 0, mf_params, rule_base, PSO_MAX_VELOCITY, PSO_MAX_VELOCITY, 8},
    };

    // 初始化模糊控制器
    FuzzyPID controller(fuzzy_params);

// 使用粒子群优化算法优化模糊控制器
#ifdef PSO_OPTIMIZE_DIFF
    particleSwarmOptimization(controller, pid, 0.0, 10, PSO_DELTA_T);
#endif

    // 输出优化后的模糊控制器参数
    // controller.showFuzzyPIDInfo();

    Print_PID_Update_Info(&pid);

    PSOState state = {0.0, 0.0, 0.0};
    float kp = 0, ki = 0, kd = 0;
    float score = 0;
    // pid.set_pid_info();
    for (int i = 0; i < 100; ++i)
    {
#ifndef TEST
        PSOState controller_output = {0, 0, 0};
        pid.get_pid(kp, ki, kd);
        controller.fuzzy_pid_control(state.position, 5, 0, kp, ki, kd, 2.0);
        pid.set_gains(kp, ki, kd);
        controller_output.velocity = pid.update_all(state.position, 5, PSO_DELTA_T, PSO_MAX_VELOCITY);
#else
        PSOState controller_output = {0, 0, 0};
        controller_output = Get_PID_Calc(&pid, TARGET, state.position);
#endif
        state = systemDynamics(state, controller_output, PSO_DELTA_T);
        score += pow(5 - state.position, 2);
        printf("time: %3.2f, position: %f, control: %f, error: %f\n", i * PSO_DELTA_T, state.position, controller_output.velocity, 5 - state.position);
    }
    printf("score: %f\n", score);

    return 0;
}