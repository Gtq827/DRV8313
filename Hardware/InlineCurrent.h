#ifndef __INLINE_CURRENT_H__
#define __INLINE_CURRENT_H__

#include "stm32f10x.h"

#ifndef _ADC_CONV
// 3.3V / 12-bit ADC
#define _ADC_CONV (3.3f / 4096.0f)
#endif

// ===== 两相采样：双电机共 4 路 ADC =====
// adc_buf[0] = M0_IA (PA0 / ADC0)
// adc_buf[1] = M0_IB (PA1 / ADC1)
// adc_buf[2] = M1_IA (PA3 / ADC3)
// adc_buf[3] = M1_IB (PA4 / ADC4)
#define ADC_BUF_LEN 4

extern volatile uint16_t adc_buf[ADC_BUF_LEN];

// 电流采集对象（保持你原结构，C 相用数学补全）
typedef struct
{
    int motor_id;

    // chC / idxC 保留兼容，但两相模式下不使用
    uint8_t chA, chB, chC;
    int8_t  idxA, idxB, idxC;

    float shunt;
    float gain;
    float va2ia;

    float offset_raw_a;
    float offset_raw_b;
    float offset_raw_c;   // 保留兼容

    float offset_a;
    float offset_b;
    float offset_c;       // 保留兼容
    float offset_alpha;   
	
    float ia, ib, ic;

} CurrSense_t;

// ===== 硬件初始化 =====
void TIM1_ADCTrigger_Init(void);    // TIM1: ~20kHz, 中点触发 ADC1
void ADC1_DMA_Current_Init(void);   // ADC1: 扫描 4 通道, DMA1_Ch1 → adc_buf[4]

// ===== 电流相关接口 =====
void CurrSense_Init(CurrSense_t *cs, int motor_id);
void CurrSense_Calibrate(CurrSense_t *cs);        // 电机不通电时调一次（只校准 A/B）
void CurrSense_SystemCalibrateAll(CurrSense_t *cs0, CurrSense_t *cs1);
void CurrSense_GetPhaseCurrents(CurrSense_t *cs); // 更新 ia/ib/ic（ic=-(ia+ib)）

#endif // __INLINE_CURRENT_H__
