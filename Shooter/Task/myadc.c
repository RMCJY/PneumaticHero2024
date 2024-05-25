/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-25 11:07:24
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-05-25 11:08:22
 * @FilePath: \Shooter\Task\myadc.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "stm32f1xx_hal_adc.h"

/* External variables --------------------------------------------------------*/
float voltage;

/* Private function prototypes -----------------------------------------------*/
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

void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}

float get_battery_voltage(void)
{
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc2, ADC_CHANNEL_11);
    voltage =  (float)adcx * voltage_vrefint_proportion * 2.0 * 1.41;

    return voltage;
}