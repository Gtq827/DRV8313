#include "PID.h"
#include "timer.h"

#define _constrain(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))

void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit       = limit;

    pid->error_prev    = 0.0f;
    pid->output_prev   = 0.0f;
    pid->integral_prev = 0.0f;
    pid->ts_prev       = micros();
}

void PID_Reset(PIDController *pid)
{
    pid->error_prev    = 0.0f;
    pid->output_prev   = 0.0f;
    pid->integral_prev = 0.0f;
    pid->ts_prev       = micros();
}

float PID_Calc(PIDController *pid, float error)
{
    uint32_t now = micros();
    float Ts = (now - pid->ts_prev) * 1e-6f;
    if (Ts <= 0.0f || Ts > 0.5f) Ts = 1e-3f;

    // P
    float p = pid->P * error;

    // I (Tustin)
    float integ = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    integ = _constrain(integ, -pid->limit, pid->limit);

    // D
    float d = pid->D * (error - pid->error_prev) / Ts;

    float out = p + integ + d;
    out = _constrain(out, -pid->limit, pid->limit);

    // Êä³öÏÞËÙ
    if (pid->output_ramp > 0.0f)
    {
        float rate = (out - pid->output_prev) / Ts;
        if (rate >  pid->output_ramp) out = pid->output_prev + pid->output_ramp * Ts;
        if (rate < -pid->output_ramp) out = pid->output_prev - pid->output_ramp * Ts;
    }

    pid->integral_prev = integ;
    pid->output_prev   = out;
    pid->error_prev    = error;
    pid->ts_prev       = now;

    return out;
}
