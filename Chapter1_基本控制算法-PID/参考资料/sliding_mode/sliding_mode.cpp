#include <iostream>
#include <cmath>

using namespace std;

// 定义系统参数
const double K = 1.0;      // 控制增益
const double lambda = 1.0; // 滑模面参数
const double eta = 1.0;    // 控制增益

// 定义系统状态
struct State
{
    double position;
    double velocity;
};
// 这里以一个三阶系统为例。
// 设有一个标准三阶系统
// ⎧ x ˙ 1 = x 2
// ⎪ x ˙ 2 = x 3
// ⎩ x ˙ 3 = f ( x ) + g ( x ) u
// 其中g(x)是u 前面的系数公式，大多情况下是常数表达式，也存在一些g(x)为x的函数的情况。下文中把f(x)和g(x)略写为f和g。
// 假设x 1 的期望值为x 1d 。那么可以写出各个x i 的误差e i
// ⎧ e 1 = x 1 − x 1d
// ⎪ e 2 = e ˙ 1 = x ˙ 1 - x ˙ 1d = x 2 - x ˙ 1d
// ⎩ e 3 = e ˙ 2 = x ˙ 2 - x ˙ 2d = x 3 - x ˙ 2d
// 设计滑模面s
// s = c 1 e 1 + c 2 e 2 + c 3 e 3 = 0
// 其中c 1 、c 2 、c 3 是滑模面的系数，一般取为1。
// 滑模面s 的导数为
// s ˙ = c 1 e ˙ 1 + c 2 e ˙ 2 + c 3 e ˙ 3
// = c 1 e ˙ 2 + c 2 e ˙ 3 + x 3 - x 3d
// = c 1 e ˙ 2 + c 2 e ˙ 3 + f + g u - x 3d
// =​ Γ + f + g u - x 3d
// 设计lyapunov函数
// V = 1 / 2 s ^ 2
// V ˙ = s s ˙ = s (Γ + f + g u - x 3d)
// 将控制量u 单独提出来
// = s g u + s (Γ + f - x 3d)
// = s g ((Γ + f - x 3d)/ g + u)
// u = -K sign(s)
// 其中K 是控制增益，sign(s) 是符号函数。
// V ˙ = s (Γ + f - x 3d) - Kg | s | <= | s | (|Γ + f - x 3d| - Kg) <= 0
// 只要V ˙ < 0，系统就是稳定的。
// k 只要满足 k > |Γ + f - x 3d| / g 就可以了。
// 由Lyapunov 稳定性定理可知，当V ˙ <= 0 时，系统是稳定的。
// 由于V ˙ = 0 是一个不等式，所以系统是一个稳定的滑模控制系统。

// 定义符号函数
double sign(double x)
{
    return x > 0 ? 1 : -1;
}

// 定义滑模控制器
double slidingModeControl(const State &state, double desired_position)
{
    // 计算误差
    double error = desired_position - state.position;
    double error_dot = -state.velocity;

    // 计算滑模面
    double s = lambda * error + error_dot;

    // 计算控制输入
    double control_input = K * s + eta * sign(s);
    // eta * sign(s);
    //  K * s + eta * sign(s);

    return control_input;
}

// 定义系统动力学
State systemDynamics(const State &state, double control_input, double dt)
{
    State new_state;
    new_state.velocity = state.velocity + control_input * dt;
    new_state.position = state.position + state.velocity * dt;
    return new_state;
}

int main()
{
    // 初始化系统状态
    State state = {0.0, 0.0};
    double desired_position = 1.0; // 目标位置
    double dt = 0.01;              // 时间步长
    int steps = 1000;              // 仿真步数

    // 仿真
    for (int i = 0; i < steps; ++i)
    {
        // 计算控制输入
        double control_input = slidingModeControl(state, desired_position);

        // 更新系统状态
        state = systemDynamics(state, control_input, dt);

        // 输出当前状态
        cout << "Time: " << i * dt << " Position: " << state.position << " Velocity: " << state.velocity << " Control Input: " << control_input << endl;
    }

    return 0;
}