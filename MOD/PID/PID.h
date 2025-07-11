#ifndef PID_h
#define PID_h

#include <stdint.h>
#include "main.h" // 假设使用STM32F4系列，需根据实际MCU型号调整

// PID控制器类型
typedef enum {
    PID_POSITIONAL, // 位置型PID
    PID_INCREMENTAL // 增量型PID
} PIDType_t;

// 低通滤波器结构体
typedef struct {
    float alpha;      // 滤波系数 (0 < alpha < 1)
    float last_output; // 上一次滤波输出
} LowPassFilter_t;

// PID控制器结构体
typedef struct {
    float kp;               // 比例增益
    float ki;               // 积分增益
    float kd;               // 微分增益
    float integral_limit;    // 积分限幅
    float output_limit;     // 输出限幅
    float integral;         // 积分项
    float last_error;       // 上一次误差
    float prev_error;       // 上上一次误差（用于增量型PID）
    float last_output;      // 上一次输出（用于增量型PID）
    float *setpoint;        // 目标值指针
    float *feedback;        // 反馈值指针
    float *output;          // 输出值指针
    PIDType_t type;         // PID类型（位置型或增量型）
    uint32_t last_tick;     // 上一次调用HAL_GetTick的时间（毫秒）
} PIDController_t;


void LPF_Init(LowPassFilter_t *lpf, float alpha);
float LPF_Compute(LowPassFilter_t *lpf, float input);

void PID_Init(PIDController_t *pid, float kp, float ki, float kd, 
              float integral_limit, float output_limit, 
              float *setpoint, float *feedback, float *output,
              PIDType_t type);

void PID_SetParams(PIDController_t *pid, float kp, float ki, float kd);
void PID_SetIntegralLimit(PIDController_t *pid, float integral_limit);
void PID_SetOutputLimit(PIDController_t *pid, float output_limit);
void PID_Compute(PIDController_t *pid, LowPassFilter_t *lpf, uint32_t current_tick);
void PID_Reset(PIDController_t *pid);



#endif // !PID_h
