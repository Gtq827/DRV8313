#include "PWM.h"

void PWM_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio;

    // TIM3: PA6 PA7
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // TIM3: PB0 PB1
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    // TIM4: PB8 PB9
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    // ------- TIM3 ------- //
    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitTypeDef base;
    base.TIM_ClockDivision      = TIM_CKD_DIV1;
    base.TIM_CounterMode        = TIM_CounterMode_Up;
    base.TIM_Period             = 255 - 1;
    base.TIM_Prescaler          = 14 - 1;   
    base.TIM_RepetitionCounter  = 0;
    TIM_TimeBaseInit(TIM3, &base);

    TIM_OCInitTypeDef oc;
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;

    TIM_OC1Init(TIM3, &oc);
    TIM_OC2Init(TIM3, &oc);
    TIM_OC3Init(TIM3, &oc);
    TIM_OC4Init(TIM3, &oc);
    TIM_Cmd(TIM3, ENABLE);

    // ------- TIM4 ------- //
    TIM_InternalClockConfig(TIM4);
    TIM_TimeBaseInit(TIM4, &base);

    TIM_OC3Init(TIM4, &oc);
    TIM_OC4Init(TIM4, &oc);
    TIM_Cmd(TIM4, ENABLE);
}
