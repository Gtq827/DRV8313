#include "FOC.h"
#include "PWM.h"
#include "Delay.h"
#include "math.h"

#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef _2PI
#define _2PI 6.28318530718f
#endif

#ifndef _constrain
#define _constrain(x,low,high) ((x)<(low)?(low):((x)>(high)?(high):(x)))
#endif

#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

// ======= 小死区避免乱动 =======
#define VEL_TARGET_EPS      0.05f
#define VEL_FEEDBACK_EPS    0.10f
#define ANG_ERROR_EPS       0.002f
#define IQ_REF_EPS          0.02f
#define IQ_MEAS_EPS         0.05f

Motor_t M0 = {0};
Motor_t M1 = {0};


// ======================================================
// 工具函数
// ======================================================
static float normalize_angle(float a)
{
    while (a < 0)       a += _2PI;
    while (a >= _2PI)   a -= _2PI;
    return a;
}

static float wrap_pm_pi(float a)
{
    while (a >  PI) a -= _2PI;
    while (a < -PI) a += _2PI;
    return a;
}

static float electrical_angle(Motor_t *m)
{
    float mech = AS5600_GetAngle(m->enc);
    float elec = mech * (float)m->pp * (float)m->dir - m->zero;
    return normalize_angle(elec);
}


// ======================================================
// PWM 输出
// ======================================================
static void set_pwm_abc(Motor_t *m, float Ua, float Ub, float Uc)
{
    float da = _constrain(Ua / m->vbus, 0, 1);
    float db = _constrain(Ub / m->vbus, 0, 1);
    float dc = _constrain(Uc / m->vbus, 0, 1);

    if (m == &M0) {
        TIM_SetCompare1(TIM3, (uint16_t)(da * 255));
        TIM_SetCompare2(TIM3, (uint16_t)(db * 255));
        TIM_SetCompare3(TIM3, (uint16_t)(dc * 255));
    } else {
        TIM_SetCompare4(TIM3, (uint16_t)(da * 255));
        TIM_SetCompare3(TIM4, (uint16_t)(db * 255));
        TIM_SetCompare4(TIM4, (uint16_t)(dc * 255));
    }
}


// ======================================================
// 力矩控制
// ======================================================
void FOC_setTorqueAngle(Motor_t *m, float Uq, float angle_el)
{
    Uq = _constrain(Uq, -m->vbus*0.5f, m->vbus*0.5f);
    angle_el = normalize_angle(angle_el);

    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    float Ua = Ualpha + m->vbus * 0.5f;
    float Ub = (sqrtf(3)*Ubeta - Ualpha)/2 + m->vbus * 0.5f;
    float Uc = (-Ualpha - sqrtf(3)*Ubeta)/2 + m->vbus * 0.5f;

    set_pwm_abc(m, Ua, Ub, Uc);
}

void FOC_setTorque(Motor_t *m, float Uq)
{
    FOC_setTorqueAngle(m, Uq, electrical_angle(m));
}


// ======================================================
// 传感器对齐（你原来习惯的 3π/2 法）
// ======================================================
static void align_sensor(Motor_t *m)
{
    const float align_angle = 1.5f * PI;
    const float Uq_align = 3.0f;

    FOC_setTorqueAngle(m, Uq_align, align_angle);
    Delay_ms(1500);

    float mech = AS5600_GetAngle(m->enc);

    m->zero = normalize_angle(mech * m->pp * m->dir);

    // ?记录机械零点（用于无限圈角度）
    m->zero_mech = mech;

    FOC_setTorqueAngle(m, 0, align_angle);
}


// ======================================================
// Iq 测量
// ======================================================
static float measure_Iq(Motor_t *m)
{
    if (m->cs == 0) return 0;

    CurrSense_GetPhaseCurrents(m->cs);

    float ia = m->cs->ia;
    float ib = m->cs->ib;
    float ic = m->cs->ic;

    float Ialpha = ia;
    float Ibeta  = _1_SQRT3 * ia + _2_SQRT3 * ib;

    float ang = electrical_angle(m);
    float ct = cosf(ang);
    float st = sinf(ang);

    float Iq = Ibeta * ct - Ialpha * st;
    
    return LowPassFilter_Calc(&m->iq_lpf, Iq);

}


float FOC_GetIq(Motor_t *m)
{
    return measure_Iq(m);
}


// ======================================================
// 电流闭环
// ======================================================
void FOC_setIq(Motor_t *m, float iq_ref)
{
    float iq_meas = measure_Iq(m);

//    if (fabsf(iq_ref) < IQ_REF_EPS && fabsf(iq_meas) < IQ_MEAS_EPS) {
//        PID_Reset(&m->iq_pid);
//        FOC_setTorque(m, 0);
//        return;
//    }

    float uq = PID_Calc(&m->iq_pid, iq_ref - iq_meas);
    uq = _constrain(uq, -m->vbus*0.5f, m->vbus*0.5f);

    FOC_setTorque(m, uq);
}


// ======================================================
// PID 配置
// ======================================================
void FOC_SetCurrentPID(Motor_t *m, float kp, float ki, float kd, float ramp, float limit)
{
    m->iq_pid.P = kp;
    m->iq_pid.I = ki;
    m->iq_pid.D = kd;
    m->iq_pid.output_ramp = ramp;
    m->iq_pid.limit = limit;
}

void FOC_SetVelPID(Motor_t *m, float kp, float ki, float kd, float ramp, float limit)
{
    m->vel_pid.P = kp;
    m->vel_pid.I = ki;
    m->vel_pid.D = kd;
    m->vel_pid.output_ramp = ramp;
    m->vel_pid.limit = limit;
}

void FOC_SetAngPID(Motor_t *m, float kp, float ki, float kd, float ramp, float limit)
{
    m->ang_pid.P = kp;
    m->ang_pid.I = ki;
    m->ang_pid.D = kd;
    m->ang_pid.output_ramp = ramp;
    m->ang_pid.limit = limit;
}


static void FOC_EnableGPIO_Init(void)
{
    static uint8_t inited = 0;
    if (inited) return;
    inited = 1;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;   // M0_EN, M1_EN
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
}

void FOC_Enable(uint8_t M0_en, uint8_t M1_en)
{
    FOC_EnableGPIO_Init();

    GPIO_WriteBit(GPIOB, GPIO_Pin_12, M0_en ? Bit_SET : Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_13, M1_en ? Bit_SET : Bit_RESET);
}

// ======================================================
// 初始化
// ======================================================
void FOC_Init(Motor_t *m, AS5600_t *enc, float vbus, int pp, int dir)
{
    m->vbus = vbus;
    m->pp   = pp;
    m->dir  = dir;
    m->enc  = enc;

    m->zero = 0;
    m->zero_mech = 0;

    m->cs = 0;

    LowPassFilter_Init(&m->vel_lpf, 0.05f);
    LowPassFilter_Init(&m->iq_lpf,  0.05f);
    LowPassFilter_Init(&m->ang_lpf, 0.1f);

    align_sensor(m);
}

void FOC_AttachCurrentSensor(Motor_t *m, CurrSense_t *cs)
{
    m->cs = cs;
}


// ======================================================
// 状态获取
// ======================================================
float FOC_Angle(Motor_t *m)
{
    float mech = AS5600_GetFullRot(m->enc);
    float val = (mech - m->zero_mech) * m->dir;
    return LowPassFilter_Calc(&m->ang_lpf, val);
}

float FOC_Vel(Motor_t *m)
{
    float raw = AS5600_GetVelocity(m->enc);
    return LowPassFilter_Calc(&m->vel_lpf, raw * m->dir);
}

float FOC_GetElectricalAngle(Motor_t *m)
{
    return electrical_angle(m);
}


// ======================================================
// 三级环
// ======================================================
void FOC_setVelocity(Motor_t *m, float vel_target)
{
    float vel_now = FOC_Vel(m);
    float err = vel_target - vel_now;

    // ===== 速度死区：小误差当 0 =====
    if (fabsf(vel_target) < VEL_TARGET_EPS && fabsf(vel_now) < VEL_FEEDBACK_EPS)
    {
        PID_Reset(&m->vel_pid);
        FOC_setIq(m, 0);
        return;
    }

    // 新增：避免“轻微自动旋转”
    if (fabsf(err) < 0.02f)   // 0.02 rad/s 你可以自己调
        err = 0.0f;

    float iq_ref = PID_Calc(&m->vel_pid, err);
    FOC_setIq(m, iq_ref);
}


void FOC_setAngle(Motor_t *m, float ang_target)
{
    float ang_now = FOC_Angle(m);
    float err = ang_target - ang_now;

    if (fabsf(err) < ANG_ERROR_EPS) {
        PID_Reset(&m->ang_pid);
        FOC_setVelocity(m, 0);
        return;
    }

    float vel_ref = PID_Calc(&m->ang_pid, err);
    FOC_setVelocity(m, vel_ref);
}

void FOC_setForce(Motor_t *m, float ang_target)
{
    float ang_now = FOC_Angle(m);
    float err = ang_target - ang_now;

    float iq_ref = -PID_Calc(&m->ang_pid, err);
    FOC_setIq(m, iq_ref);
}
