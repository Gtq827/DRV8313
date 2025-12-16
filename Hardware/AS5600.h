#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f10x.h"

#define PI 3.14159265358979f
#define _2PI 6.28318530718f

typedef struct {
    I2C_TypeDef *i2c;

    float prev_angle;       // 上一次单圈角度
    int   turns;            // 圈数计数

    float prev_full;        // 上一次 full_rot
    uint32_t prev_ts;       // 上一次时间戳 us
	  float vel_prev_angle;   // 上一次的单圈角度 0~2π
    uint32_t vel_prev_ts;   // 上一次时间 micros()
} AS5600_t;

extern AS5600_t enc_M0; // I2C1
extern AS5600_t enc_M1; // I2C2

void AS5600_Init(void);

float AS5600_GetAngle(AS5600_t *enc);
float AS5600_GetFullRot(AS5600_t *enc);
float AS5600_GetVelocity(AS5600_t *enc);

#endif
