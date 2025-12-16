#include "stm32f10x.h"
#include "FOC.h"
#include "AS5600.h"
#include "PWM.h"
#include "InlineCurrent.h"
#include "Timer.h"
CurrSense_t CS_M0;
CurrSense_t CS_M1;

extern Motor_t M0;
extern Motor_t M1;
extern AS5600_t enc_M0;
extern AS5600_t enc_M1;

float a, b;
float c, d;
float e, f;
float p,i,d;
float g,h,i,j,k;
int main(void)
{
    LED_Init();
    Timer_Init();  
	  PWM_Init();
    TIM1_ADCTrigger_Init();
    ADC1_DMA_Current_Init();
    AS5600_Init();
    Delay_ms(10);
    
    CurrSense_Init(&CS_M0, 0);
    CurrSense_Init(&CS_M1, 1);
    
    CurrSense_SystemCalibrateAll(&CS_M0, &CS_M1);
    
    FOC_Init(&M0, &enc_M0, 12.0f, 7, -1);
    FOC_Init(&M1, &enc_M1, 12.0f, 7,  1);
	
	  FOC_AttachCurrentSensor(&M0, &CS_M0);
    FOC_AttachCurrentSensor(&M1, &CS_M1);
	
    
    FOC_SetAngPID(&M0, 8.0f, 0.0f, 0.0f, 100000, 40.0f);
    FOC_SetAngPID(&M1, 8.0f, 0.0f, 0.0f, 100000, 40.0f);
    
  
    FOC_SetVelPID(&M0, 0.15f, 0.05f, 0.0f, 100000, 40.0f);
    FOC_SetVelPID(&M1, 0.15f, 0.05f, 0.0f, 100000, 40.0f);
    
   
    FOC_SetCurrentPID(&M0, 1.2f, 0.0f, 0.0f, 100000, 6.0f);
    FOC_SetCurrentPID(&M1, 1.2f, 0.0f, 0.0f, 100000, 6.0f);


		
    while (1)
    {   
			  //FOC_SetCurrentPID(&M0, p, i, d, 100000, 6.0f);

//    FOC_SetAngPID(&M1, g, 0.0f, 0.0f, 100000, 40.0f);
//    FOC_SetVelPID(&M1, h, i, 0.0f, 100000, 40.0f);
//    FOC_SetCurrentPID(&M1, j, k, 0.0f, 100000, 6.0f);
        // 查看编码器
        c = FOC_Angle(&M0);
        d = FOC_Vel(&M1);
     
        // === 控制示例 ===
			  //FOC_setIq(&M0, a);
        FOC_setVelocity(&M0, a);
        FOC_setIq(&M1, b);

        // 当前 Iq
        e = FOC_GetIq(&M0);
        f = FOC_GetIq(&M1);
    }
}
