#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

using namespace std;

// 定义系统状态
struct State
{
    double position;
    double velocity;
};

// 定义模糊规则
struct FuzzyRule
{
    double a;      // 输入1的隶属度
    double b;      // 输入2的隶属度
    double output; // 输出
};

// 定义高斯隶属函数
double gaussianMembership(double x, double c, double alpha)
{
    return exp(-pow((x - c) / alpha, 2));
}

// 定义模糊控制器
class TakagiSugenoFuzzyController
{
public:
    TakagiSugenoFuzzyController()
    {
        // 初始化模糊规则
        rules.push_back({1.0, 1.0, 1.0});
        rules.push_back({1.0, -1.0, -1.0});
        rules.push_back({-1.0, 1.0, -1.0});
        rules.push_back({-1.0, -1.0, 1.0});
    }

    double computeControl(const State &state, double desired_position) const
    {
        double error = desired_position - state.position;
        double error_dot = -state.velocity;

        double numerator = 0.0;
        double denominator = 0.0;

        for (const auto &rule : rules)
        {
            double weight = gaussianMembership(error, rule.a, 1.0) * gaussianMembership(error_dot, rule.b, 1.0);
            numerator += weight * rule.output;
            denominator += weight;
        }

        return numerator / denominator;
    }

    void setRule(int index, double a, double b, double output)
    {
        if (index >= 0 && index < rules.size())
        {
            rules[index].a = a;
            rules[index].b = b;
            rules[index].output = output;
        }
    }

private:
    vector<FuzzyRule> rules;
};

// 定义系统动力学
State systemDynamics(const State &state, double control_input, double dt)
{
    State new_state;
    new_state.velocity = state.velocity + control_input * dt;
    new_state.position = state.position + state.velocity * dt;
    return new_state;
}

// 定义三角观测器
class TriangularObserver
{
public:
    TriangularObserver(double gain) : gain(gain) {}

    State estimate(const State &measured_state, const State &previous_estimate, double control_input, double dt)
    {
        State estimate;
        estimate.velocity = previous_estimate.velocity + gain * (measured_state.velocity - previous_estimate.velocity) + control_input * dt;
        estimate.position = previous_estimate.position + gain * (measured_state.position - previous_estimate.position) + estimate.velocity * dt;
        return estimate;
    }

private:
    double gain;
};

// 适应度函数
double fitnessFunction(const TakagiSugenoFuzzyController &controller, const vector<State> &states, double desired_position, double dt)
{
    double total_error = 0.0;
    State state = states[0];

    for (const auto &s : states)
    {
        double control_input = controller.computeControl(state, desired_position);
        state = systemDynamics(state, control_input, dt);
        total_error += pow(desired_position - state.position, 2);
    }

    return total_error;
}

// 粒子群优化算法
void particleSwarmOptimization(TakagiSugenoFuzzyController &controller, const vector<State> &states, double desired_position, double dt)
{
    // PSO 参数
    const int num_parameters = 4;
    const int num_rules = 3;
    const int num_particles = 30;
    const int num_iterations = 100;
    const double w = 0.5;  // 惯性权重
    const double c1 = 1.5; // 个体学习因子
    const double c2 = 1.5; // 社会学习因子

    // 初始化粒子
    vector<vector<double>> particles(num_particles, vector<double>(num_parameters * num_rules, 0.0));
    vector<vector<double>> velocities(num_particles, vector<double>(num_parameters * num_rules, 0.0));
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
    }

    // PSO 迭代
    for (int iter = 0; iter < num_iterations; ++iter)
    {
        for (int i = 0; i < num_particles; ++i)
        {
            // 设置模糊控制器参数
            for (int j = 0; j < num_parameters; ++j)
            {
                controller.setRule(j, particles[i][j * 3], particles[i][j * 3 + 1], particles[i][j * 3 + 2]);
            }

            // 计算适应度
            double score = fitnessFunction(controller, states, desired_position, dt);

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
        }

        // 更新粒子速度和位置
        for (int i = 0; i < num_particles; ++i)
        {
            for (int j = 0; j < num_parameters * num_rules; ++j)
            {
                velocities[i][j] = w * velocities[i][j] +
                                   c1 * dis(gen) * (personal_best_positions[i][j] - particles[i][j]) +
                                   c2 * dis(gen) * (global_best_position[j] - particles[i][j]);
                particles[i][j] += velocities[i][j];
            }
        }
    }

    // 使用全局最优参数更新模糊控制器
    for (int i = 0; i < num_parameters; ++i)
    {
        controller.setRule(i, global_best_position[i * 3], global_best_position[i * 3 + 1], global_best_position[i * 3 + 2]);
    }
}

int main()
{
    // 初始化系统状态
    State state = {0.0, 0.0};
    double desired_position = 1.0; // 目标位置
    double dt = 0.01;              // 时间步长
    int steps = 1000;              // 仿真步数

    // 初始化模糊控制器
    TakagiSugenoFuzzyController controller;

    // 初始化状态向量
    vector<State> states(steps, state);

    // 使用粒子群优化算法优化模糊控制器
    particleSwarmOptimization(controller, states, desired_position, dt);

    // 初始化三角观测器
    TriangularObserver observer(0.1); // 观测器增益

    // 仿真
    State estimated_state = state;
    for (int i = 0; i < steps; ++i)
    {
        // 计算控制输入
        double control_input = controller.computeControl(estimated_state, desired_position);

        // 更新系统状态
        state = systemDynamics(state, control_input, dt);

        // 更新估计状态
        estimated_state = observer.estimate(state, estimated_state, control_input, dt);

        // 输出当前状态
        cout << "Time: " << i * dt << " Position: " << state.position << " Velocity: " << state.velocity << " Estimated Position: " << estimated_state.position << " Estimated Velocity: " << estimated_state.velocity << " Control Input: " << control_input << endl;
    }

    return 0;
}