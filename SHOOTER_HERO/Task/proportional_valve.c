#include "main.h"
#include "adc.h"
#include "stm32f1xx_hal_adc.h"
#include "proportional_valve.h"
#include "I2C_Software.h"


volatile float voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);
}

void InitVrefintReciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}

float voltage;
float err;
float GetVoltage(void)
{
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc2, ADC_CHANNEL_11);
    // voltage =  (float)adcx * voltage_vrefint_proportion * 2.0 * 1.41;
    voltage =  (float)adcx * voltage_vrefint_proportion * 2.0;
    err = -0.0025 * voltage + 0.0866;   //实验测量得出的偏差曲线
    voltage = voltage + err;

    return voltage;
}

void DP8201SSetVoltage(float target_voltage)
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
    // HAL_I2C_Master_Transmit(&hi2c1,0xB0,I2C1_TX_buff,3,1000);
    // I2CWriteData(0xB0, I2C1_TX_buff, 3);

}

void DAC5571SetVoltage(float target_voltage)
{
    uint8_t I2C1_TX_buff[2];
    uint16_t voltage_send = target_voltage * 0x3ff / 5.0;
    if(voltage_send > 0x3ff)
    {
        voltage_send = 0x3ff;
    }
    I2C1_TX_buff[0] = (uint8_t)((voltage_send >> 6) & 0x0f);
    I2C1_TX_buff[1] = (uint8_t)((voltage_send & 0x3f) << 2);
    // HAL_I2C_Master_Transmit(&hi2c1,0x90,I2C1_TX_buff,2,1000);
    I2CWriteData(0x90, I2C1_TX_buff, 2);
}