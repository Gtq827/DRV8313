#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

typedef struct
{
    float P, I, D;
    float output_ramp;   // 输出变化限速
    float limit;         // 输出幅值限制

    float error_prev;
    float output_prev;
    float integral_prev;
    uint32_t ts_prev;
} PIDController;

void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit);
void PID_Reset(PIDController *pid);
float PID_Calc(PIDController *pid, float error);

#endif
