#include "fuzzypid.h"

int limit(int value, int max_limit, int min_limit)
{
    if (value > max_limit)
        return max_limit;
    if (value < min_limit)
        return min_limit;
    return value;
}

#define inverse(parameter) 1.0f / (float)parameter

// 高斯型隶属度函数（gaussian membership function）
float gaussmf(float x, float sigma, float c)
{
    return expf(-powf(((x - c) / sigma), 2.0f));
}

// 广义钟形隶属度函数（generalized bell-shaped membership function）
float gbellmf(float x, float a, float b, float c)
{
    return inverse(1.0f + powf(fabsf((x - c) / a), 2.0f * b));
}

// s型隶属度函数（sigmoidal membership function）
float sigmf(float x, float a, float c)
{
    return inverse(1.0f + expf(a * (c - x)));
}

// 梯形隶属度函数（trapezoidal membership function）
float trapmf(float x, float a, float b, float c, float d)
{
    if (x >= a && x < b)
        return (x - a) / (b - a);
    else if (x >= b && x < c)
        return 1.0f;
    else if (x >= c && x <= d)
        return (d - x) / (d - c);
    else
        return 0.0f;
}

// 三角隶属度函数（triangular membership function）
float trimf(float x, float a, float b, float c)
{
    return trapmf(x, a, b, b, c);
}

// z型隶属度函数（z-shaped membership function）
float zmf(float x, float a, float b)
{
    if (x <= a)
        return 1.0f;
    else if (x >= a && x <= (a + b) / 2.0f)
        return 1.0f - 2.0f * powf((x - a) / (b - a), 2.0f);
    else if (x >= (a + b) / 2.0f && x < b)
        return 2.0f * powf((x - b) / (b - a), 2.0f);
    else
        return 0;
}

// 隶属度函数（membership function）
// x: 输入
// mf_type: 隶属度函数类型
// params: 隶属度函数参数
float mf(float x, unsigned int mf_type, int *params)
{
    switch (mf_type)
    {
    case 0:
        return gaussmf(x, params[0], params[1]);
    case 1:
        return gbellmf(x, params[0], params[1], params[2]);
    case 2:
        return sigmf(x, params[0], params[2]);
    case 3:
        return trapmf(x, params[0], params[1], params[2], params[3]);
    case 5:
        return zmf(x, params[0], params[1]);
    default: // set triangular as default membership function
        return trimf(x, params[0], params[1], params[2]);
    }
}

// Union operator
float _or(float a, float b, unsigned int type)
{
    if (type == 1)
    { // algebraic sum
        return a + b - a * b;
    }
    else if (type == 2)
    { // bounded sum
        return fminf(1, a + b);
    }
    else
    { // fuzzy union
        return fmaxf(a, b);
    }
}

// Intersection operator
float _and(float a, float b, unsigned int type)
{
    if (type == 1)
    { // algebraic product
        return a * b;
    }
    else if (type == 2)
    { // bounded product
        return fmaxf(0, a + b - 1);
    }
    else
    { // fuzzy intersection
        return fminf(a, b);
    }
}

// Equilibrium operator
float _equilibrium(float a, float b, float params)
{
    return powf(a * b, 1 - params) * powf(1 - (1 - a) * (1 - b), params);
}

// 模糊数量域的操作（fuzzy quantity fields operation）
float fo(float a, float b, unsigned int type)
{
    if (type < 3)
    {
        return _and(a, b, type);
    }
    else if (type < 6)
    {
        return _or(a, b, type - 3);
    }
    else
    {
        return _equilibrium(a, b, 0.5f);
    }
}

// 中心去模糊化的平均值，只适用于两个输入的多重指标
// Mean of centers defuzzifier, only for two input multiple index
void moc(const float *joint_membership, const unsigned int *index, const unsigned int *count, struct fuzzy *fuzzy_struct)
{

    float denominator_count = 0;
    float numerator_count[fuzzy_struct->output_num];
    for (unsigned int l = 0; l < fuzzy_struct->output_num; ++l)
    {
        numerator_count[l] = 0;
    }

    for (int i = 0; i < count[0]; ++i)
    {
        for (int j = 0; j < count[1]; ++j)
        {
            denominator_count += joint_membership[i * count[1] + j];
        }
    }

    for (unsigned int k = 0; k < fuzzy_struct->output_num; ++k)
    {
        for (unsigned int i = 0; i < count[0]; ++i)
        {
            for (unsigned int j = 0; j < count[1]; ++j)
            {
                numerator_count[k] += joint_membership[i * count[1] + j] *
                                      fuzzy_struct->rule_base[k * qf_default * qf_default + index[i] * qf_default +
                                                              index[count[0] + j]];
            }
        }
    }

#ifdef fuzzy_pid_debug_print
    printf("output:\n");
#endif
    for (unsigned int l = 0; l < fuzzy_struct->output_num; ++l)
    {
        fuzzy_struct->output[l] = numerator_count[l] / denominator_count;
#ifdef fuzzy_pid_debug_print
        printf("%f / %f = %f\n", numerator_count[l], denominator_count, fuzzy_struct->output[l]);
#endif
    }
}

// 去模糊化器（defuzzifier）
void df(const float *joint_membership, const unsigned int *output, const unsigned int *count, struct fuzzy *fuzzy_struct,
        int df_type)
{
    if (df_type == 0)
        moc(joint_membership, output, count, fuzzy_struct);
    else
    {
        printf("Waring: No such of defuzzifier!\n");
        moc(joint_membership, output, count, fuzzy_struct);
    }
}

struct PID **fuzzy_pid_vector_init(struct PID_params *params, float delta_k, struct Fuzzy_params *fuzzy_params, unsigned int count)
{
    struct PID **pid = (struct PID **)malloc(sizeof(struct PID *) * count);
    for (unsigned int i = 0; i < count; ++i)
    {
        pid[i] = fuzzy_pid_init(params[i], delta_k, fuzzy_params[i]);
    }
    return pid;
}

// struct PID_params params = {kp, ki, kd, integral_limit, dead_zonefeed_forward, linear_adaptive_kp};
// delta_k: the ratio of kp, ki, kd to delta kp, delta ki, delta kd
struct PID *fuzzy_pid_init(struct PID_params params, float delta_k, struct Fuzzy_params fuzzy_params)
{
    return raw_fuzzy_pid_init(params.kp, params.ki, params.kd, params.integral_limit, params.dead_zone,
                              params.feed_forward, max_error, max_delta_error,
                              params.kp / delta_k, params.ki / delta_k, params.kd / delta_k,
                              fuzzy_params.mf_type, fuzzy_params.fo_type, fuzzy_params.df_type,
                              fuzzy_params.mf_params, fuzzy_params.rule_base,
                              params.min_output, params.middle_output, params.max_output);
}

struct PID *raw_fuzzy_pid_init(float kp, float ki, float kd, float integral_limit, float dead_zone,
                               float feed_forward, float error_max, float delta_error_max, float delta_kp_max,
                               float delta_ki_max, float delta_kd_max, unsigned int mf_type, unsigned int fo_type,
                               unsigned int df_type, int mf_params[], int rule_base[][qf_default],
                               int output_min_value, int output_middle_value, int output_max_value)
{
    struct PID *pid = (struct PID *)malloc(sizeof(struct PID));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->delta_kp_max = delta_kp_max;
    pid->delta_ki_max = delta_ki_max;
    pid->delta_kd_max = delta_kd_max;

    pid->delta_kp = 0;
    pid->delta_ki = 0;
    pid->delta_kd = 0;

    pid->error_max = error_max;
    pid->delta_error_max = delta_error_max;

    // 根据ki和kd是否为0来确定输出的数量
    int output_count = 1;
    if (ki > 1e-4)
    {
        output_count += 1;
        if (kd > 1e-4)
            output_count += 1;
    }

    pid->fuzzy_struct = fuzzy_init(2, output_count);
    fuzzy_params_init(pid->fuzzy_struct, mf_type, fo_type, df_type, mf_params, rule_base);

    pid->last_error = 0;
    pid->current_error = 0;

    pid->intergral = 0;
    pid->intergral_limit = integral_limit;

    pid->dead_zone = dead_zone;
    pid->feed_forward = feed_forward;

    pid->output_max_value = output_max_value;
    pid->output_middle_value = output_middle_value;
    pid->output_min_value = output_min_value;

    return pid;
}

//
struct fuzzy *fuzzy_init(unsigned int input_num, unsigned int output_num)
{
    struct fuzzy *fuzzy_struct = (struct fuzzy *)malloc(sizeof(struct fuzzy));
    fuzzy_struct->input_num = input_num;
    fuzzy_struct->output_num = output_num;
    fuzzy_struct->mf_type = (unsigned int *)malloc((input_num + output_num) * sizeof(unsigned int));
#ifdef fuzzy_pid_rule_base_deep_copy
    fuzzy_struct->mf_params = (int *)malloc(4 * qf_default * sizeof(int));
    fuzzy_struct->rule_base = (int *)malloc(output_num * qf_default * qf_default * sizeof(int));
#endif
    fuzzy_struct->output = (float *)malloc(output_num * sizeof(float));
    return fuzzy_struct;
}

void fuzzy_params_init(struct fuzzy *fuzzy_struct, unsigned int mf_type, unsigned int fo_type, unsigned int df_type,
                       int mf_params[], int rule_base[][qf_default])
{
    for (unsigned int i = 0; i < fuzzy_struct->input_num + fuzzy_struct->output_num; ++i)
    {
        fuzzy_struct->mf_type[i] = mf_type;
    }

    for (unsigned int i = 0; i < fuzzy_struct->output_num; ++i)
    {
        fuzzy_struct->output[i] = 0;
    }

#ifdef fuzzy_pid_rule_base_deep_copy
    for (unsigned int j = 0; j < 4 * qf_default; ++j)
    {
        fuzzy_struct->mf_params[j] = mf_params[j];
    }

    for (unsigned int k = 0; k < fuzzy_struct->output_num * qf_default; ++k)
    {
        for (unsigned int i = 0; i < qf_default; ++i)
        {
            fuzzy_struct->rule_base[k * 7 + i] = rule_base[k][i];
        }
    }
#else
    fuzzy_struct->mf_params = mf_params;
    fuzzy_struct->rule_base = (int *)rule_base;
#endif

    fuzzy_struct->fo_type = fo_type;
    fuzzy_struct->df_type = df_type;
}

// 模糊控制器（fuzzy controller）
void fuzzy_control(float e, float de, struct fuzzy *fuzzy_struct)
{
    float membership[qf_default * 2];   // Store membership
    unsigned int index[qf_default * 2]; // Store the index of each membership
    unsigned int count[2] = {0, 0};

    // 通过隶属度函数计算隶属度
    {
        int j = 0;
        for (int i = 0; i < qf_default; ++i)
        {
            float temp = mf(e, fuzzy_struct->mf_type[0], fuzzy_struct->mf_params + 4 * i);
            if (temp > 1e-4)
            {
                membership[j] = temp;
                index[j++] = i;
            }
        }

        count[0] = j;

        for (int i = 0; i < qf_default; ++i)
        {
            float temp = mf(de, fuzzy_struct->mf_type[1], fuzzy_struct->mf_params + 4 * i);
            if (temp > 1e-4)
            {
                membership[j] = temp;
                index[j++] = i;
            }
        }

        count[1] = j - count[0];
    }

#ifdef fuzzy_pid_debug_print
    {
        int j = count[0] + count[1];
        printf("membership:\n");
        for (unsigned int k = 0; k < j; ++k)
        {
            printf("%f\n", membership[k]);
        }

        printf("index:\n");
        for (unsigned int k = 0; k < j; ++k)
        {
            printf("%d\n", index[k]);
        }

        printf("count:\n");
        for (unsigned int k = 0; k < 2; ++k)
        {
            printf("%d\n", count[k]);
        }
    }
#endif

    if (count[0] == 0 || count[1] == 0)
    {
        for (unsigned int l = 0; l < fuzzy_struct->output_num; ++l)
        {
            fuzzy_struct->output[l] = 0;
        }
        return;
    }

    // 计算联合隶属度
    // Joint membership
    float joint_membership[count[0] * count[1]];

    for (int i = 0; i < count[0]; ++i)
    {
        for (int j = 0; j < count[1]; ++j)
        {
            joint_membership[i * count[1] + j] = fo(membership[i], membership[count[0] + j], fuzzy_struct->fo_type);
        }
    }

    df(joint_membership, index, count, fuzzy_struct, 0);
}

float fuzzy_pid_control(float real, float idea, struct PID *pid)
{
    pid->last_error = pid->current_error;
    pid->current_error = idea - real;
    float delta_error = pid->current_error - pid->last_error;
    ///
    pid->differential = pid->current_error - pid->last_error;
    limit(pid->differential, pid->delta_error_max, -pid->delta_error_max);
    limit(pid->current_error, pid->error_max, -pid->error_max);
///
#ifdef fuzzy_pid_dead_zone
    if (pid->current_error < pid->dead_zone && pid->current_error > -pid->dead_zone)
    {
        pid->current_error = 0;
    }
    else
    {
        if (pid->current_error > pid->dead_zone)
            pid->current_error = pid->current_error - pid->dead_zone;
        else
        {
            if (pid->current_error < -pid->dead_zone)
                pid->current_error = pid->current_error + pid->dead_zone;
        }
    }
#endif
    fuzzy_control(pid->current_error / pid->error_max * 3.0f, delta_error / pid->delta_error_max * 3.0f,
                  pid->fuzzy_struct);

    pid->delta_kp = pid->fuzzy_struct->output[0] / 3.0f * pid->delta_kp_max + pid->kp;

    if (pid->fuzzy_struct->output_num >= 2)
        pid->delta_ki = pid->fuzzy_struct->output[1] / 3.0f * pid->delta_ki_max;
    else
        pid->delta_ki = 0;

    if (pid->fuzzy_struct->output_num >= 3)
        pid->delta_kd = pid->fuzzy_struct->output[2] / 3.0f * pid->delta_kd_max;
    else
        pid->delta_ki = 0;

#ifdef fuzzy_pid_debug_print
    printf("kp : %f, ki : %f, kd : %f\n", pid->kp, pid->ki, pid->kd);
    printf("delta kp : %f, delta ki : %f, delta kd : %f\n", pid->delta_kp, pid->delta_ki, pid->delta_kd);
    printf("kp + delta kp : %f, %f, %f\n", pid->kp + pid->delta_kp, pid->ki + pid->delta_ki, pid->kd + pid->delta_kd);
    printf("output : %f\n", pid->output);
#endif

    pid->intergral += (pid->ki + pid->delta_ki) * pid->current_error;
#ifdef fuzzy_pid_integral_limit
    if (pid->intergral > pid->intergral_limit)
        pid->intergral = pid->intergral_limit;
    else
    {
        if (pid->intergral < -pid->intergral_limit)
            pid->intergral = -pid->intergral_limit;
    }
#endif
    pid->output = (pid->kp + pid->delta_kp) * pid->current_error + pid->intergral +
                  (pid->kd + pid->delta_kd) * pid->differential;
    pid->output += pid->feed_forward * (float)idea;
    return pid->output;
}

int fuzzy_pid_motor_pwd_output(float real, float idea, bool direct, struct PID *pid)
{
    return limit(fuzzy_pid_control(real, idea, pid), pid->output_max_value, pid->output_min_value);
}

void delete_fuzzy(struct fuzzy *fuzzy_struct)
{
    free(fuzzy_struct->mf_type);
    free(fuzzy_struct->output);
    free(fuzzy_struct);
}

void delete_pid(struct PID *pid)
{
    if (pid->fuzzy_struct != NULL)
    {
        delete_fuzzy(pid->fuzzy_struct);
    }
    free(pid);
}

void delete_pid_vector(struct PID **pid_vector, unsigned int count)
{
    for (unsigned int i = 0; i < count; ++i)
    {
        delete_pid(pid_vector[i]);
    }
    free(pid_vector);
}

// dof: degree of freedom
#define DOF 6

int main()
{
    // 默认的模糊规则库
    int rule_base[][qf_default] = {
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
    int mf_params[4 * qf_default] = {-3, -3, -2, 0,
                                     -3, -2, -1, 0,
                                     -2, -1, 0, 0,
                                     -1, 0, 1, 0,
                                     0, 1, 2, 0,
                                     1, 2, 3, 0,
                                     2, 3, 3, 0};
    // 默认的 PID 控制器参数
    struct PID_params pid_params[DOF] = {{0.65f, 0, 0, 0, 0, 0, -3, 0, 3, 1},
                                         {-0.34f, 0, 0, 0, 0, 0, -3, 0, 3, 1},
                                         {-1.1f, 0, 0, 0, 0, 0, -3, 0, 3, 1},
                                         {-2.4f, 0, 0, 0, 0, 0, -3, 0, 3, 1},
                                         {1.2f, 0, 0, 0, 0, 0, -3, 0, 3, 1},
                                         {1.2f, 0.05f, 0.1f, 0, 0, 0, -3, 0, 3, 1}};
    struct Fuzzy_params fuzzy_params[DOF] = {{4, 1, 0, mf_params, rule_base},
                                             {4, 1, 0, mf_params, rule_base},
                                             {4, 1, 0, mf_params, rule_base},
                                             {4, 1, 0, mf_params, rule_base},
                                             {4, 1, 0, mf_params, rule_base},
                                             {4, 1, 0, mf_params, rule_base}};
    // 根据参数获取 PID 控制器向量
    struct PID **pid_vector = fuzzy_pid_vector_init(pid_params, 2.0f, fuzzy_params, DOF);

    printf("output:\n");
    int control_id = 5;
    float real = 0;
    float idea = max_error * 0.9f;
    printf("idea value: %f\n", idea);
    bool direct[DOF] = {true, false, false, false, true, true};
    for (int j = 0; j < 1000; ++j)
    {
        real += fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
        printf("%f,%f\n", idea, real);
    }

    delete_pid_vector(pid_vector, DOF);
    return 0;
}