/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-25 11:07:24
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-02 17:55:14
 * @FilePath: \Shooter\Task\myadc.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

/* Includes ------------------------------------------------------------------*/
#include "proportional_valve_control.hpp"

/* External variables --------------------------------------------------------*/
// float voltage;

/* Private function prototypes -----------------------------------------------*/

// volatile float voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;



namespace hero
{

/**
 * @brief       初始化阶段获得参考电压值，为读取外设电压做校准
 * @param        None
 * @retval       None
 * @note         None
 */
void ProportionalValve::InitVrefintReciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += AdcxGetChxValue(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
};

/**
 * @brief       通过内置ADC读取比例阀反馈电压值，由电压映射到气室气压
 * @param        None
 * @retval       voltage 比例阀反馈气压
 * @note         None
 */
float ProportionalValve::GetProportionalValveVoltage(void)
{
    uint16_t adcx = 0;

    adcx = AdcxGetChxValue(&hadc2, ADC_CHANNEL_11);
    voltage =  (float)adcx * voltage_vrefint_proportion * 2.0 * 1.41;

    return voltage;
};

/**
 * @brief       控制比例阀将气室气压维持在设定值
 * @param        target_voltage: 指定比例阀气压，映射到电压上
 * @retval       None
 * @note        第一版发射主控采用野鸡厂商的AP8291S芯片
 */
void ProportionalValve::DP8201SSetVoltage(float target_voltage)
{
    if(target_voltage > 5.0)
    {
        target_voltage = 5;
    }
    else if(target_voltage < 0.0)
    {
        target_voltage = 0;
    }
    
    uint8_t i2c_tx_buff[3];
    uint16_t voltage_send = target_voltage * 0x0fff / 5.0;

    i2c_tx_buff[0] = 0x02;
    i2c_tx_buff[1] = (uint8_t)((voltage_send & 0x00f) << 4);
    i2c_tx_buff[2] = (uint8_t)(voltage_send >> 4);
    HAL_I2C_Master_Transmit(&hi2c1, 0xB0, i2c_tx_buff, 3, 1000);
};

/**
 * @brief       控制比例阀将气室气压维持在设定值
 * @param        target_voltage: 指定比例阀气压，映射到电压上
 * @retval       None
 * @note        第二版发射主控板改用TI的DAC5571芯片
 */ 
void ProportionalValve::DAC5571SetVoltage(float target_voltage)
{
    
};

bool ProportionalValve::IsPhotogateCover(void)
{
    // #TODO 还需要查看光电门高低电平的含义
    bool is_photogate_cover = false;
    if(HAL_GPIO_ReadPin(Photogate_GPIO_Port,Photogate_Pin) == GPIO_PIN_RESET)
    {
        is_photogate_cover = true;
    }
    return is_photogate_cover;
};  

uint16_t ProportionalValve::AdcxGetChxValue(ADC_HandleTypeDef *ADCx, uint32_t ch)
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
};

} // namespace ProportionalValve
