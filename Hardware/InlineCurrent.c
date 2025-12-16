#include "InlineCurrent.h"
#include "Delay.h"

// 4 通道 DMA 缓冲（双电机两相）
volatile uint16_t adc_buf[ADC_BUF_LEN];

// ===============================
// TIM1: ~20kHz, 用来触发 ADC1
// PSC = 13, ARR = 254 → 72MHz/(14*255) ≈ 20.17kHz
// 采样点：CCR1 = ARR/2，周期中点
// ===============================
void TIM1_ADCTrigger_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef tim;
    TIM_TimeBaseStructInit(&tim);
    tim.TIM_Prescaler     = 14 - 1;
    tim.TIM_Period        = 255 - 1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &tim);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = ((255 - 1) *2)/3;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &oc);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // ADC 触发源用 T1_CC1（这里 TRGO 配什么其实无所谓）
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1Ref);

    TIM_Cmd(TIM1, ENABLE);
}

// ===============================
// ADC1 + DMA: 扫描模式，4 通道（双电机两相）
// 每次 TIM1_CC1 触发采 4 个通道，DMA 搬到 adc_buf[]
// 通道映射：
//   adc_buf[0] = PA0 (ADC0) = M0_IA
//   adc_buf[1] = PA1 (ADC1) = M0_IB
//   adc_buf[2] = PA3 (ADC3) = M1_IA
//   adc_buf[3] = PA4 (ADC4) = M1_IB
// ===============================
void ADC1_DMA_Current_Init(void)
{
    // ADC 时钟 72/6 = 12MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // PA0, PA1, PA3, PA4 模拟输入
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    // DMA1 Channel1: ADC1->DR → adc_buf[4]
    DMA_InitTypeDef dma;
    DMA_DeInit(DMA1_Channel1);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    dma.DMA_MemoryBaseAddr     = (uint32_t)adc_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize         = ADC_BUF_LEN;   // 4
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &dma);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // ADC1 初始化
    ADC_InitTypeDef adc;
    ADC_StructInit(&adc);
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T1_CC1;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = ADC_BUF_LEN;   // 4
    ADC_Init(ADC1, &adc);

    // Rank 1..4
    // 采样时间你先保持 55.5 周期，稳定后再考虑降到 28.5 / 13.5
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5); // PA0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5); // PA1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_55Cycles5); // PA3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_55Cycles5); // PA4

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ExternalTrigConvCmd(ADC1, ENABLE);

    // 校准
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

// ===============================
// CurrSense 对象初始化：双电机两相
// motor_id = 0 → M0: adc_buf[0]=IA, adc_buf[1]=IB
// motor_id = 1 → M1: adc_buf[2]=IA, adc_buf[3]=IB
// IC 不采样：ic = -(ia + ib)
// ===============================
void CurrSense_Init(CurrSense_t *cs, int motor_id)
{
    cs->motor_id = motor_id;

    if (motor_id == 0)
    {
        cs->idxA = 0;
        cs->idxB = 1;
        cs->idxC = -1;

        cs->chA  = ADC_Channel_0;
        cs->chB  = ADC_Channel_1;
        cs->chC  = 0;
    }
    else
    {
        cs->idxA = 2;
        cs->idxB = 3;
        cs->idxC = -1;

        cs->chA  = ADC_Channel_3;
        cs->chB  = ADC_Channel_4;
        cs->chC  = 0;
    }

    // 硬件参数：0.01Ω + INA199(50x)
    cs->shunt = 0.01f;
    cs->gain  = 50.0f;
    cs->va2ia = 1.0f / (cs->shunt * cs->gain); // 2 A/V

    cs->offset_raw_a = 0.0f;
    cs->offset_raw_b = 0.0f;
    cs->offset_raw_c = 0.0f;
    cs->offset_alpha = 1e-4f; 
		
    cs->offset_a = 0.0f;
    cs->offset_b = 0.0f;
    cs->offset_c = 0.0f;

    cs->ia = cs->ib = cs->ic = 0.0f;
}

// ===============================
// 电流零偏校准：电机不通电时调用一次
// 两相模式：只校准 A/B
// ===============================
void CurrSense_Calibrate(CurrSense_t *cs)
{
    uint32_t sumA = 0, sumB = 0;

    // 丢掉前面若干帧（让 DMA/ADC 稳定）
    for (int i = 0; i < 50; i++)
        Delay_us(50);

    // 按帧采样
    for (int i = 0; i < 200; i++)
    {
        Delay_us(50);
        sumA += adc_buf[(int)cs->idxA];
        sumB += adc_buf[(int)cs->idxB];
    }

    cs->offset_raw_a = sumA / 200.0f;
    cs->offset_raw_b = sumB / 200.0f;

    cs->offset_a = cs->offset_raw_a * _ADC_CONV;
    cs->offset_b = cs->offset_raw_b * _ADC_CONV;
}

// 你工程里已有这三个 extern，我沿用
extern void FOC_Enable(uint8_t M0_en, uint8_t M1_en);
extern void Delay_ms(uint32_t);

// ===============================
// 一次性校准所有电流传感器
// ===============================
void CurrSense_SystemCalibrateAll(CurrSense_t *cs0, CurrSense_t *cs1)
{
    FOC_Enable(0, 0);
    Delay_ms(10);

    CurrSense_Calibrate(cs0);
    CurrSense_Calibrate(cs1);

    FOC_Enable(1, 1);
    Delay_ms(5);
}

// ===============================
// 读取三相电流（单位：A）
// 两相实测，一相数学补全
// ===============================
void CurrSense_GetPhaseCurrents(CurrSense_t *cs)
{
    float Va = adc_buf[(int)cs->idxA] * _ADC_CONV;
    float Vb = adc_buf[(int)cs->idxB] * _ADC_CONV;

    cs->ia = (Va - cs->offset_a) * cs->va2ia;
    cs->ib = (Vb - cs->offset_b) * cs->va2ia;

    // 数学补全第三相（不再采样）
    cs->ic = -(cs->ia + cs->ib);
}
