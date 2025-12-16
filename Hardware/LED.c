#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);   // ★ 清空结构体（避免污染其他引脚）

    gpio.GPIO_Pin   = GPIO_Pin_14;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &gpio);
}

void LED_ON(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}

void LED_OFF(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_14);
}

void LED_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_14) == 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	}
}
