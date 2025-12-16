#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H

#include "stm32f10x.h"

typedef struct
{
    float Tf;
    float y_prev;
    uint32_t ts_prev;
} LowPassFilter;

void LowPassFilter_Init(LowPassFilter *f, float Tf);
float LowPassFilter_Calc(LowPassFilter *f, float x);

#endif
