#ifndef __FOC_H
#define __FOC_H

#include "stm32f10x.h"
#include "AS5600.h"
#include "InlineCurrent.h"
#include "pid.h"
#include "lowpass_filter.h"

#define PI   3.14159265358979f
#define _2PI 6.28318530718f

typedef struct
{
    float vbus;         // 母线电压(例如 12V)
    int   pp;           // 极对数
    int   dir;          // 方向 +1/-1

    AS5600_t *enc;      // 编码器对象

    float zero;         // 电角度零点（对齐后）
    float zero_mech;    // ?机械零点（你指定要加的）

    CurrSense_t *cs;    // 电流传感器

    PIDController vel_pid;  // 速度环 PID
    PIDController ang_pid;  // 位置环 PID
    PIDController iq_pid;   // 电流环 PID

    LowPassFilter vel_lpf;  // 速度低通
    LowPassFilter iq_lpf;   // Iq 测量低通
    LowPassFilter ang_lpf;  // 角度低通（用于位置控制）
} Motor_t;

// ========== 全局两个电机 ==========
extern Motor_t M0;
extern Motor_t M1;

// 使能 MOS
void FOC_Enable(uint8_t M0_en, uint8_t M1_en);

// 初始化、绑定传感器
void FOC_Init(Motor_t *m, AS5600_t *enc, float vbus, int pp, int dir);
void FOC_AttachCurrentSensor(Motor_t *m, CurrSense_t *cs);

// PID 设置接口
void FOC_SetCurrentPID(Motor_t *m, float kp, float ki, float kd, float ramp, float limit);
void FOC_SetVelPID    (Motor_t *m, float kp, float ki, float kd, float ramp, float limit);
void FOC_SetAngPID    (Motor_t *m, float kp, float ki, float kd, float ramp, float limit);

// 开环 / 电流闭环
void FOC_setTorqueAngle(Motor_t *m, float Uq, float angle_el);
void FOC_setTorque     (Motor_t *m, float Uq);
void FOC_setIq         (Motor_t *m, float iq_ref);

// 闭环接口
void FOC_setVelocity(Motor_t *m, float vel);
void FOC_setAngle   (Motor_t *m, float ang);
void FOC_setForce   (Motor_t *m, float ang);

// 状态获取
float FOC_Angle(Motor_t *m);             // 无限圈机械角(rad)
float FOC_Vel(Motor_t *m);               // rad/s
float FOC_GetIq(Motor_t *m);             // q轴电流(A)
float FOC_GetElectricalAngle(Motor_t *m);

#endif
