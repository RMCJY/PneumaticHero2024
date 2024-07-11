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
#include "control.h"
#include "proportional_valve.h"
#include "gimbal_shooter_comm.h"


// shooter_state_t shooter_state;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t time_ms = 0;
    static uint8_t TxDate[8] = {0};
    // static uint8_t DAC_send_flag = 0;
    // static uint8_t valve25_state = 0;
    if(htim == &htim4)
    {
        time_ms++;

        ShooterTask();

        if(time_ms % 10 == 0)
        {
            // 以100Hz的频率与云台板通信
            ShooterGimbalComm();
            // ShooterTask();
        }
        
        /*
        Photogate_State_Read();
        GetVoltage();
        DAC5571SetVoltage(1.5);
        if(shooter.shooter_command == 1 || shooter.shooter_command == 2 || shooter.shooter_command == 3)
        {
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
            // if(Shoot() == SHOOTER_FINISH)
            // {
            //     shooter.shooter_command = 0;
            //     HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
            // }

            //HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            // valve25_state = 1 - valve25_state;
            // Valve25_Send(valve25_state);
            // shooter.shooter_command = 0;

        }
        if(time_ms <= 1000)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
            time_ms++;
            HAL_GPIO_WritePin(Valve23_GPIO_Port, Valve23_Pin, RESET);
            HAL_GPIO_WritePin(Valve25_GPIO_Port, Valve25_Pin, RESET);
        }
        else if(time_ms <= 2000)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
            HAL_GPIO_WritePin(Valve23_GPIO_Port, Valve23_Pin, SET);
            HAL_GPIO_WritePin(Valve25_GPIO_Port, Valve25_Pin, SET);
            time_ms++;
        }
        else if(time_ms <= 3000)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
            HAL_GPIO_WritePin(Valve23_GPIO_Port, Valve23_Pin, RESET);
            HAL_GPIO_WritePin(Valve25_GPIO_Port, Valve25_Pin, RESET);
            time_ms++;
        }
        else if(time_ms <= 4000)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
            time_ms ++;
        }
        else 
        {
            time_ms = 0;
        }
        */
        // CAN发送测试
        // TxDate[0] = (TxDate[0] + 1) % 16;
        // CAN_Send_Msg(&hcan, TxDate, SHOOT_ID, 8);
        // if(time_ms % 5 == 0)
        // {
        //     // get_battery_voltage();
        //     // DP8201S_Set_Voltage(1.8);
        //     CAN_Send_Msg(&hcan, TxDate, SHOOT_ID, 8);
            
        // }
        // if(DAC_send_flag == 0)
        // {
        //     // DP8201S_Set_Voltage(3.0);
        //     DAC_send_flag = 1;
        // }
        
    }    
}


void HardwareInit(void)
{
    HAL_GPIO_WritePin(I2C_A0_GPIO_Port, I2C_A0_Pin, RESET);         // 设置I2C地址为
}
