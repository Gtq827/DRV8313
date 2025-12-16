#include "AS5600.h"
#include "Delay.h"
#include "math.h"

#define AS5600_ADDR (0x36 << 1)
#define I2C_TIMEOUT 20000

AS5600_t enc_M0 = {0};
AS5600_t enc_M1 = {0};


/*-------------------------------------------------------
  I2C1 Bus Release
-------------------------------------------------------*/
static void I2C1_ReleaseBus(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;  // PB6=SCL PB7=SDA
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_SetBits(GPIOB, GPIO_Pin_7);

    for (int i=0; i<9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
        for (volatile int t=0; t<100; t++);
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
        for (volatile int t=0; t<100; t++);
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_7);
    for (volatile int t=0; t<200; t++);
    GPIO_SetBits(GPIOB, GPIO_Pin_7);

    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &gpio);
}


/*-------------------------------------------------------
  I2C2 Bus Release
-------------------------------------------------------*/
static void I2C2_ReleaseBus(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;  // PB10=SCL PB11=SDA
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_SetBits(GPIOB, GPIO_Pin_11);

    for (int i=0; i<9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        for (volatile int t=0; t<100; t++);
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        for (volatile int t=0; t<100; t++);
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    for (volatile int t=0; t<200; t++);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);

    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &gpio);
}


/*-------------------------------------------------------
  I2C Wait Event
-------------------------------------------------------*/
static uint8_t I2C_Wait(I2C_TypeDef *i2c, uint32_t evt)
{
    uint32_t t = I2C_TIMEOUT;
    while (!I2C_CheckEvent(i2c, evt)) {
        if (--t == 0) return 0;
    }
    return 1;
}


/*-------------------------------------------------------
  Read Raw Angle
-------------------------------------------------------*/
static uint16_t AS5600_ReadRaw(AS5600_t *enc)
{
    I2C_TypeDef *i2c = enc->i2c;

    if (I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY))
        return 0xFFFF;

    I2C_GenerateSTART(i2c, ENABLE);
    if (!I2C_Wait(i2c, I2C_EVENT_MASTER_MODE_SELECT)) goto FAIL;

    I2C_Send7bitAddress(i2c, AS5600_ADDR, I2C_Direction_Transmitter);
    if (!I2C_Wait(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto FAIL;

    I2C_SendData(i2c, 0x0C);
    if (!I2C_Wait(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) goto FAIL;

    I2C_GenerateSTART(i2c, ENABLE);
    if (!I2C_Wait(i2c, I2C_EVENT_MASTER_MODE_SELECT)) goto FAIL;

    I2C_Send7bitAddress(i2c, AS5600_ADDR, I2C_Direction_Receiver);
    if (!I2C_Wait(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) goto FAIL;

    if (!I2C_Wait(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED)) goto FAIL;
    uint8_t high = I2C_ReceiveData(i2c);

    I2C_AcknowledgeConfig(i2c, DISABLE);

    if (!I2C_Wait(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED)) goto FAIL;
    uint8_t low = I2C_ReceiveData(i2c);

    I2C_GenerateSTOP(i2c, ENABLE);
    I2C_AcknowledgeConfig(i2c, ENABLE);

    return ((high << 8) | low) & 0x0FFF;

FAIL:
    I2C_GenerateSTOP(i2c, ENABLE);
    I2C_AcknowledgeConfig(i2c, ENABLE);
    return 0xFFFF;
}


/*-------------------------------------------------------
  Init Both Encoders
-------------------------------------------------------*/
void AS5600_Init(void)
{
    I2C1_ReleaseBus();
    I2C2_ReleaseBus();

    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef  init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    // PB6/7 I2C1
    gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    // PB10/11 I2C2
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    I2C_StructInit(&init);
    init.I2C_ClockSpeed = 100000;
    init.I2C_Mode       = I2C_Mode_I2C;
    init.I2C_Ack        = I2C_Ack_Enable;
    init.I2C_DutyCycle  = I2C_DutyCycle_2;
    init.I2C_OwnAddress1= 0;

    I2C_DeInit(I2C1);
    I2C_Init(I2C1, &init);
    I2C_Cmd(I2C1, ENABLE);

    I2C_DeInit(I2C2);
    I2C_Init(I2C2, &init);
    I2C_Cmd(I2C2, ENABLE);
    
    enc_M0.i2c = I2C1;
    enc_M0.prev_angle = 0;
    enc_M0.turns      = 0;
    enc_M0.prev_full  = 0;
    enc_M0.prev_ts    = micros();

    enc_M0.vel_prev_ts    = 0;    // ★ 让速度模块自己 lazy init
    enc_M0.vel_prev_angle = 0;

    enc_M1.i2c = I2C2;
    enc_M1.prev_angle = 0;
    enc_M1.turns      = 0;
    enc_M1.prev_full  = 0;
    enc_M1.prev_ts    = micros();

    enc_M1.vel_prev_ts    = 0;    // ★ 同理
    enc_M1.vel_prev_angle = 0;

}


/*-------------------------------------------------------
  Single-turn angle
-------------------------------------------------------*/
float AS5600_GetAngle(AS5600_t *enc)
{
    uint16_t raw = AS5600_ReadRaw(enc);

    if (raw == 0xFFFF)
        return enc->prev_angle;

    float a = raw * (_2PI / 4096.0f);
    enc->prev_angle = a;
    return a;
}


/*-------------------------------------------------------
  Multi-turn angle
-------------------------------------------------------*/
float AS5600_GetFullRot(AS5600_t *enc)
{
    float a = AS5600_GetAngle(enc);

    float prev_raw = enc->prev_full - enc->turns * _2PI;
    float diff = a - prev_raw;

    if (fabsf(diff) > 0.8f * _2PI)
        enc->turns += (diff > 0 ? -1 : 1);

    enc->prev_full = enc->turns * _2PI + a;
    return enc->prev_full;
}


/*-------------------------------------------------------
  Velocity (rad/s)
-------------------------------------------------------*/
float AS5600_GetVelocity(AS5600_t *enc)
{
    uint32_t now = micros();

    // 第一次调用：只初始化，不计算速度
    if (enc->vel_prev_ts == 0)
    {
        enc->vel_prev_ts    = now;
        enc->vel_prev_angle = AS5600_GetFullRot(enc);  // 连续角度
        return 0.0f;
    }

    float dt = (now - enc->vel_prev_ts) * 1e-6f;
    if (dt <= 0) dt = 1e-3f;

    float angle = AS5600_GetFullRot(enc);     // 使用连续角度
    float diff  = angle - enc->vel_prev_angle;

    float vel = diff / dt;                    // rad/s

    enc->vel_prev_ts    = now;
    enc->vel_prev_angle = angle;

    return vel;
}





