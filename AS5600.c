#include "AS5600.h"

// ========= 左右编码器全局实例 ========= //
AS5600_Encoder enc_L = {0};
AS5600_Encoder enc_R = {0};


// ========= 内部函数：读寄存器 ========= //
static uint8_t AS5600_ReadReg(AS5600_Encoder *enc, uint8_t reg)
{
    I2C_TypeDef *i2c = enc->i2c;
    uint8_t addr = 0x36 << 1;  // AS5600 地址

    while (I2C_GetFlagStatus(i2c, I2C_FLAG_BUSY));

    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(i2c, addr, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(i2c, reg);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(i2c, ENABLE);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(i2c, addr, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while (!I2C_CheckEvent(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t data = I2C_ReceiveData(i2c);

    I2C_GenerateSTOP(i2c, ENABLE);

    return data;
}


// ========= 初始化单个编码器 ========= //
void AS5600_Encoder_Init(AS5600_Encoder *enc, I2C_TypeDef *i2c)
{
    enc->i2c = i2c;
    enc->prev_angle = 0;
    enc->turns = 0;
    enc->prev_full_angle = 0;
    enc->prev_ts = micros();
}


// ========= 初始化 I2C1（PB6/PB7） I2C2（PB10/PB11） ========= //
void AS5600_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  // I2C1
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // I2C2
    GPIO_Init(GPIOB, &gpio);

    I2C_InitTypeDef init;
    init.I2C_ClockSpeed = 400000;
    init.I2C_Mode = I2C_Mode_I2C;
    init.I2C_DutyCycle = I2C_DutyCycle_2;
    init.I2C_Ack = I2C_Ack_Enable;
    init.I2C_OwnAddress1 = 0;
    init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2C1, &init);
    I2C_Cmd(I2C1, ENABLE);

    I2C_Init(I2C2, &init);
    I2C_Cmd(I2C2, ENABLE);

    // 初始化左右编码器
    AS5600_Encoder_Init(&enc_L, I2C2);
    AS5600_Encoder_Init(&enc_R, I2C1);
}


// ========= 获取单圈角度（弧度） ========= //
float AS5600_GetAngle(AS5600_Encoder *enc)
{
    uint16_t high = AS5600_ReadReg(enc, 0x0E);
    uint16_t low  = AS5600_ReadReg(enc, 0x0F);
    uint16_t raw  = ((high << 8) | low) & 0x0FFF;

    // 转弧度： raw * (2π / 4096)
    return raw * 0.001533981f;  // 常数 = 2π/4096
}


// ========= 连续角度（支持无限旋转） ========= //
float AS5600_GetFullRot(AS5600_Encoder *enc)
{
    float angle = AS5600_GetAngle(enc);
    float d = angle - enc->prev_angle;

    if (fabs(d) > (0.8f * 6.283185f))
        enc->turns += (d > 0 ? -1 : 1);

    enc->prev_angle = angle;

    return enc->turns * 6.283185f + angle;
}


// ========= 获取角速度（rad/s） ========= //
float AS5600_GetVelocity(AS5600_Encoder *enc)
{
    uint32_t now = micros();
    float dt = (now - enc->prev_ts) * 1e-6f;

    if (dt <= 0) dt = 1e-3f;

    float full = AS5600_GetFullRot(enc);
    float vel = (full - enc->prev_full_angle) / dt;

    enc->prev_full_angle = full;
    enc->prev_ts = now;

    return vel;
}
