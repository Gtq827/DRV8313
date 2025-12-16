#include "timer.h"
#include "LED.h"

volatile uint32_t micros_overflow = 0;

// ------ 返回微秒值 ------
uint32_t micros(void)
{
    return micros_overflow + TIM2->CNT;
}
static uint32_t led_count = 0;
// ------ TIM2 中断 ------
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        micros_overflow += 1000;  // 1ms = 1000us
        led_count++;
        if(led_count>=250){led_count=0;LED_Turn();}
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

// ------ TIM2 初始化（1us 精度） ------
void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef t;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    t.TIM_Period = 1000 - 1;    // 1ms 更新事件
    t.TIM_Prescaler = 72 - 1;   // 1us 计数一次
    t.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &t);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef n;
    n.NVIC_IRQChannel = TIM2_IRQn;
    n.NVIC_IRQChannelCmd = ENABLE;
    n.NVIC_IRQChannelPreemptionPriority = 2;
    n.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&n);

    TIM_Cmd(TIM2, ENABLE);
}
