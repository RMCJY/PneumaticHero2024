#include "I2C_Software.h"
#include "gpio.h"
#include "stm32f103xe.h"


// 硬件I2C会出现卡死在BTF寄存器（用于检测数据是否完成发送）位置的情况，所以使用软件I2C

#define I2C_SDA_READ()	HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)
#define I2C_SCL(a)	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, a)
#define I2C_SDA(a)	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, a)

static void DelayUs(uint32_t u_sec)
{
	uint16_t cnt = 0;
 
	while(u_sec--)
	{
		for(cnt=168/5; cnt>0; cnt--);
	}
}

void I2CStart(void)
{
	I2C_SDA(GPIO_PIN_SET);
	I2C_SCL(GPIO_PIN_SET);
	DelayUs(5);
	I2C_SDA(GPIO_PIN_RESET);
	DelayUs(5);
	I2C_SCL(GPIO_PIN_RESET);
	DelayUs(5);
}

void I2CStop(void)
{
	I2C_SDA(GPIO_PIN_RESET);
	DelayUs(5);
	I2C_SCL(GPIO_PIN_SET);
	DelayUs(5);
	I2C_SDA(GPIO_PIN_SET);
	DelayUs(5);
}

/**
 * @brief: 等待I2C应答信号
 * @param: None
 * @retval: 0-应答成功 1-应答失败
**/
uint8_t I2CWaitAck(void)
{
	uint8_t wait_time = 0;
	
	I2C_SDA(GPIO_PIN_SET);
	DelayUs(5);
	I2C_SCL(GPIO_PIN_SET);
	DelayUs(5);
	while(I2C_SDA_READ())
	{
		wait_time++;
		if(wait_time > 50)
		{
			I2CStop();
			return 1;
		}
	}
	I2C_SCL(GPIO_PIN_RESET);
	DelayUs(5);

	return 0;
}

/** 
 * @brief: 产生ACK应答信号
 * @param: None
 * @retval: None
**/
void I2CAck(void)
{
	I2C_SCL(GPIO_PIN_RESET);
	DelayUs(5);
	I2C_SDA(GPIO_PIN_RESET);
	DelayUs(5);
	I2C_SCL(GPIO_PIN_SET);
	DelayUs(5);
	I2C_SCL(GPIO_PIN_RESET);
	DelayUs(5);
	I2C_SDA(GPIO_PIN_SET);
	DelayUs(5);
}

/** 
 * @brief: 不产生NACK应答信号
 * @param: None
 * @retval: None
**/
void I2CNAck(void)
{
	I2C_SDA(GPIO_PIN_SET);
	DelayUs(5);
	I2C_SCL(GPIO_PIN_SET);
	DelayUs(5);
	I2C_SCL(GPIO_PIN_RESET);
	DelayUs(5);
}

/**
 * @brief: I2C发送一个字节
 * @param: data-待发送的数据
 * @retval: None
**/	
void I2CSendByte(uint8_t data)
{
	uint8_t i = 0;
	
	for(i = 0; i < 8; i++)
	{
		if(data & 0x80)
		{
			I2C_SDA(GPIO_PIN_SET);
		}
		else
		{
			I2C_SDA(GPIO_PIN_RESET);
		}
		data <<= 1;
		DelayUs(5);
		I2C_SCL(GPIO_PIN_SET);
		DelayUs(5);
		I2C_SCL(GPIO_PIN_RESET);
		DelayUs(5);
	}
	I2C_SDA(GPIO_PIN_SET);
}

/** 
 * @brief: I2C接收一个字节
 * @param: ack-是否应答
 * @retval: 接收到的数据
**/	
uint8_t I2CReceiveByte(uint8_t ack)
{
	uint8_t i = 0;
	uint8_t data = 0;
	
	I2C_SDA(GPIO_PIN_SET);
	for(i = 0; i < 8; i++)
	{
		I2C_SCL(GPIO_PIN_SET);
		DelayUs(5);
		data <<= 1;
		if(I2C_SDA_READ())
		{
			data |= 0x01;
		}
		I2C_SCL(GPIO_PIN_RESET);
		DelayUs(5);
	}
	if(ack)
	{
		I2CAck();
	}
	else
	{
		I2CNAck();
	}
	
	return data;
}

/**
 * @brief: I2C写数据
 * @param: dev_addr-设备地址 data-待发送的数据 len-数据长度
 * @retval: 0-成功 1-失败
**/
uint8_t I2CWriteData(uint8_t dev_addr, uint8_t *data, uint8_t len)
{
	uint8_t i = 0;
	
	I2CStart();
	I2CSendByte(dev_addr);
	if(I2CWaitAck())
	{
		return 1;
	}
	for(i = 0; i < len; i++)
	{
		I2CSendByte(data[i]);
		if(I2CWaitAck())
		{
			return 1;
		}
	}
	I2CStop();
	
	return 0;
}

