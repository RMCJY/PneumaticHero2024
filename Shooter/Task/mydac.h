#include "main.h"
#include "i2c.h"

void DP8201S_Set_Voltage(float target_voltage)
{
    uint8_t I2C1_TX_buff[3];
    uint16_t voltage_send = target_voltage * 0x0fff / 5.0;
    if(voltage_send > 0x0fff)
    {
        voltage_send = 0x0fff;
    }
    I2C1_TX_buff[0] = 0x02;
    I2C1_TX_buff[1] = (uint8_t)((voltage_send & 0x00f) << 4);
    I2C1_TX_buff[2] = (uint8_t)(voltage_send >> 4);
    HAL_I2C_Master_Transmit(&hi2c1, 0xB0, I2C1_TX_buff, 3, 1000);
}