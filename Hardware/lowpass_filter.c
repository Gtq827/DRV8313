#include "lowpass_filter.h"
#include "timer.h"

void LowPassFilter_Init(LowPassFilter *f, float Tf)
{
    f->Tf      = Tf;
    f->y_prev  = 0.0f;
    f->ts_prev = micros();
}

float LowPassFilter_Calc(LowPassFilter *f, float x)
{
    uint32_t now = micros();
    float dt = (now - f->ts_prev) * 1e-6f;

    if (dt <= 0.0f) dt = 1e-3f;
    else if (dt > 0.3f)
    {
        f->y_prev  = x;
        f->ts_prev = now;
        return x;
    }

    float alpha = f->Tf / (f->Tf + dt);
    float y = alpha * f->y_prev + (1.0f - alpha) * x;

    f->y_prev  = y;
    f->ts_prev = now;
    return y;
}
