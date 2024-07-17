/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-23 09:46:19
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-05 00:08:31
 * @FilePath: \PCB_Test\Task\control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "HW_CAN.h"
#include "adc.h"
#include "stm32f1xx_hal_adc.h"
#include "shooter.h"
#include "main_task.h"
#include "proportional_valve.h"
#include "gimbal_shooter_comm.h"




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t time_ms = 0;
    static uint8_t TxDate[8] = {0};
    if(htim == &htim4)
    {
        time_ms++;

        ShooterTask();
        if(time_ms % 10 == 0)
        {
            // 以100Hz的频率与云台板通信
            ShooterGimbalComm();
        }
        
    }    
}


void HardwareInit(void)
{
    // 暂时使用广播模式，没有用到如下的地址设置
    HAL_GPIO_WritePin(I2C_A0_GPIO_Port, I2C_A0_Pin, RESET);         // 设置I2C地址为
}
