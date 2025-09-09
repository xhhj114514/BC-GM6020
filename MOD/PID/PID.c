#include "PID.h"



/**
 * @brief 初始化PID控制器，设置PID参数、限幅、输入输出指针和控制器类型。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 * @param kp 比例增益，float类型
 * @param ki 积分增益，float类型（已隐含采样周期）
 * @param kd 微分增益，float类型（已隐含采样周期）
 * @param integral_limit 积分限幅，float类型，非负值
 * @param output_limit 输出限幅，float类型，非负值
 * @param setpoint 指向目标值的指针，float*类型
 * @param feedback 指向反馈值的指针，float*类型
 * @param output 指向输出值的指针，float*类型
 * @param type PID控制器类型，PIDType_t枚举值（PID_POSITIONAL或PID_INCREMENTAL）
 */
void PID_Init(PIDController_t *pid, float kp, float ki, float kd, 
              float integral_limit, float output_limit, float deadband, 
              float *setpoint, float *feedback, float *output,
              PIDType_t type)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->DeadBand = deadband;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_output = 0.0f;
    pid->setpoint = setpoint;
    pid->feedback = feedback;
    pid->output = output;
    pid->type = type;
}

/**
 * @brief 更新PID控制器的比例、积分和微分增益。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 * @param kp 新的比例增益，float类型
 * @param ki 新的积分增益，float类型（已隐含采样周期）
 * @param kd 新的微分增益，float类型（已隐含采样周期）
 */
void PID_SetParams(PIDController_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置PID控制器的积分限幅。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 * @param integral_limit 新的积分限幅值，float类型，非负值
 */
void PID_SetIntegralLimit(PIDController_t *pid, float integral_limit)
{
    pid->integral_limit = integral_limit;
}

/**
 * @brief 设置PID控制器的输出限幅。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 * @param output_limit 新的输出限幅值，float类型，非负值
 */
void PID_SetOutputLimit(PIDController_t *pid, float output_limit)
{
    pid->output_limit = output_limit;
}

/**
 * @brief 计算位置型PID输出，公式：output = kp * error + ki * integral + kd * (error - last_error)。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 */
static void PID_PositionalCompute(PIDController_t *pid)
{
    // 计算误差
    float error = *(pid->setpoint) - *(pid->feedback);

    // 比例项
    float p_term = pid->kp * error;

    // 积分项
    pid->integral += pid->ki * error;

    // 积分限幅
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    // 微分项
    float d_term = pid->kd * (error - pid->last_error);

    // 更新误差
    pid->last_error = error;

    // 计算总输出
    float output = p_term + pid->integral + d_term;

    // 输出限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    output = output > 0 ? output+(pid->DeadBand)*(pid->output_limit) : output-(pid->DeadBand)*(pid->output_limit);
    // 更新输出
    *(pid->output) = output + (*pid->FF)*(*(pid->setpoint));
}

/**
 * @brief 计算增量型PID输出，公式：delta_output = kp * (error - last_error) + ki * error + kd * (error - 2 * last_error + prev_error)。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 */
static void PID_IncrementalCompute(PIDController_t *pid)
{
    // 计算误差
    float error = *(pid->setpoint) - *(pid->feedback);

    // 比例项增量
    float p_term = pid->kp * (error - pid->last_error);

    // 积分项增量
    float i_term = pid->ki * error;

    // 微分项增量
    float d_term = pid->kd * (error - 2.0f * pid->last_error + pid->prev_error);

    // 更新误差
    pid->prev_error = pid->last_error;
    pid->last_error = error;

    // 计算输出增量
    float delta_output = p_term + i_term + d_term;

    // 计算总输出
    float output = pid->last_output + delta_output;

    // 输出限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    output = output > 0 ? output+(pid->DeadBand)*(pid->output_limit) : output-(pid->DeadBand)*(pid->output_limit);
    // 更新输出和上一次输出
    *(pid->output) = output;
    pid->last_output = output;
}

/**
 * @brief 执行PID计算，根据控制器类型选择位置型或增量型PID算法。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 */
void PID_Compute(PIDController_t *pid)
{
    if (pid->type == PID_POSITIONAL) {
        PID_PositionalCompute(pid);
    } else {
        PID_IncrementalCompute(pid);
    }
}

/**
 * @brief 重置PID控制器，清除积分项、误差和上一次输出。
 * @param pid 指向PID控制器结构体的指针，PIDController_t*类型
 */
void PID_Reset(PIDController_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_output = 0.0f;
}