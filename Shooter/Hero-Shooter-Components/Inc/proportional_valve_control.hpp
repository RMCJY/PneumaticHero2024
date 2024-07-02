/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-25 21:15:51
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-02 19:44:22
 * @FilePath: \Shooter\Hero-Shooter-Components\Inc\proportional_valve_control.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef PROPORTIONAL_VALVE_CONTROL_CPP_
#define PROPORTIONAL_VALVE_CONTROL_CPP_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "stm32f1xx_hal_adc.h"
#include "allocator.hpp"
#include "base.hpp"
#include <cstdint>

/* External variables --------------------------------------------------------*/
// extern float voltage;

/* Exported function prototypes ----------------------------------------------*/

namespace hero
{
class ProportionalValve : public hello_world::MemMang
{
public:
  ProportionalValve(){};
  ~ProportionalValve(){};

  float voltage;

    /**
   * @brief       初始化阶段获得参考电压值，为读取外设电压做校准
   * @param        None
   * @retval       None
   * @note         None
   */
  void InitVrefintReciprocal(void);

    /**
   * @brief       通过内置ADC读取比例阀反馈电压值，由电压映射到气室气压
   * @param        None
   * @retval       voltage 比例阀反馈气压
   * @note         None
   */
  float GetProportionalValveVoltage(void);

    /**
   * @brief       控制比例阀将气室气压维持在设定值
   * @param        target_voltage: 指定比例阀气压，映射到电压上
   * @retval       None
   * @note        第一版发射主控采用野鸡厂商的AP8291S芯片
   */
  void DP8201SSetVoltage(float target_voltage);

    /**
   * @brief       控制比例阀将气室气压维持在设定值
   * @param        target_voltage: 指定比例阀气压，映射到电压上
   * @retval       None
   * @note        第二版发射主控板改用TI的DAC5571芯片
   */ 
  void DAC5571SetVoltage(float target_voltage);  


  bool IsPhotogateCover(void);

protected:
  uint16_t AdcxGetChxValue(ADC_HandleTypeDef *ADCx, uint32_t ch);
  volatile float voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;
};

} // namespace hero

#endif