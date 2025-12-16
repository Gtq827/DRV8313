#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f10x.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================== AS5600 Encoder Class ===================== //
typedef struct {
    I2C_TypeDef *i2c;

    float prev_angle;
    int32_t turns;

    float prev_full_angle;
    uint32_t prev_ts;
} AS5600_Encoder;


// ==== API ==== //
void AS5600_Init();                   // 初始化 I2C + 编码器

void AS5600_Encoder_Init(AS5600_Encoder *enc, I2C_TypeDef *i2c);

float AS5600_GetAngle(AS5600_Encoder *enc);        // 单圈角度（弧度）
float AS5600_GetFullRot(AS5600_Encoder *enc);      // 连续角度（弧度）
float AS5600_GetVelocity(AS5600_Encoder *enc);     // 速度（rad/s）


// ==== 左右编码器实例 ==== //
extern AS5600_Encoder enc_L;
extern AS5600_Encoder enc_R;

#ifdef __cplusplus
}
#endif

#endif
