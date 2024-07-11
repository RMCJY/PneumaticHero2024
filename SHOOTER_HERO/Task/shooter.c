/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-06-07 23:01:27
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-06-09 21:13:39
 * @FilePath: \PCB_Test\Task\shooter.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <math.h>
#include "shooter.h"
#include "gpio.h"
#include "control.h"
#include "pid.h"
#include "proportional_valve.h"
#include "iwdg.h"

Shooter_t shooter;

// 已经测试过01状态对应
void GetPhotogateState(void)
{
    if(HAL_GPIO_ReadPin(Photogate_GPIO_Port, Photogate_Pin))
    {
        
        shooter.phtogate_state = PHOTOGATE_COVER;
        HAL_GPIO_WritePin(LED3_GPIO_Port,LED4_Pin,RESET);
    }
    else
    {
        shooter.phtogate_state = PHOTOGATE_EMPTY;
        HAL_GPIO_WritePin(LED3_GPIO_Port,LED4_Pin,SET);
    }
}

void Valve23Send(uint8_t command)
{
    shooter.valve23_target = command;
    if(shooter.valve23_target == VALVE23_ON)
    {
        HAL_GPIO_WritePin(Valve23_GPIO_Port, Valve23_Pin, SET);
        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,RESET);
    }
    else if(shooter.valve23_target == VALVE23_OFF)
    {
        HAL_GPIO_WritePin(Valve23_GPIO_Port, Valve23_Pin, RESET);
        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,SET);
    }
}

void Valve25Send(uint8_t command)
{
    shooter.valve25_target = command;
    if(shooter.valve25_target == VALVE25_ON)
    {
        HAL_GPIO_WritePin(Valve25_GPIO_Port, Valve25_Pin, SET);
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,RESET);
    }
    else if(shooter.valve25_target == VALVE25_OFF)
    {
        HAL_GPIO_WritePin(Valve25_GPIO_Port, Valve25_Pin, RESET);
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,SET);
    }
}

uint8_t AirPressureControl(void)
{
    // 比例阀的输入不能是负值
    float output;
    // Air_Pressure_PID.fdb = shooter.proportional_valve_fdb - shooter.proportional_valve_ref;
    // Air_Pressure_PID.ref = 0;
    // PID_Calc(&Air_Pressure_PID);
    // output = Air_Pressure_PID.output + shooter.proportional_valve_ref;
    // if(output > 5.0)
    // {
    //     output = 5.0;
    // }
    // else if(output < 0)
    // {
    //     output = 0;
    // }
    output = shooter.proportional_valve_ref;
    DAC5571SetVoltage(output);
}

uint8_t JudgeAirBottleState(void)
{
    static uint16_t wait_time = 0;
    if(fabs(shooter.proportional_valve_fdb - shooter.proportional_valve_ref) < 3)
    {
        shooter.is_airpre_ready = AIR_PRESSURE_READY;
        shooter.is_air_bottle_ready = AIR_BOTTLE_READY;
        wait_time = 0;
    }
    else
    {
        wait_time++;
        shooter.is_airpre_ready = AIR_PRESSURE_ERR;
        if(wait_time > 1000)
        {
            // 如果等待一秒钟比例阀还没有达到设定气压说明气瓶欠压
            shooter.is_air_bottle_ready = AIR_BOTTLE_ERR;
            shooter.first_run_flag = 0;
            shooter.work_state = SHOOTER_WORK_STATE_IDLE;
        }
    }
}

void GetProportionalValveState(void)
{
    shooter.proportional_valve_fdb = GetVoltage();
}

void ShooterOnIdle(void)
{
    // 开始进入发射状态
    if(shooter.phtogate_state == PHOTOGATE_COVER)
    {
        shooter.is_shooter_ready = SHOOTER_ACTIVED;
        shooter.work_state = SHOOTER_WORK_STATE_VALVE25_WAIT;
        shooter.bullet_note = BULLET_READY;
    }
    else
    {
        // 如果在此状态下光电门没有检测到弹丸，说明拨弹不到位，需要重新拨弹
        shooter.bullet_note = BULLET_NOTE;
    }
}

void ShooterOnValve25Wait(void)
{
    shooter.valve25_wait_time++;
    Valve25Send(VALVE25_ON);
    // 确保气动阀开启后进入下一个状态
    if(shooter.valve25_wait_time > 300)
    {
        shooter.work_state = SHOOTER_WORK_STATE_VALVE23_WAIT;
    }
}

void ShooterOnValve23Wait(void)
{
    /* 以下仅测试时使用，将23/25分开*/
    if(shooter.shooter_command != SHOOTER_COMMAND_NEXT)
    {
        return 0;   // 只有在按键继续按下的时候，才会进入到这个状态
    }

    shooter.valve23_wait_time++;
    Valve23Send(VALVE23_ON);
    // 确保气动阀开启并完成发射后进入下一个状态
    if(shooter.valve23_wait_time > 500)
    {
        shooter.work_state = SHOOTER_WORK_STATE_FINISH;
    }
}

void ShooterOnFinish(void)
{
    shooter.valve23_wait_time = 0;
    shooter.valve25_wait_time = 0;

    // 完成发射后，发射机构进入空闲状态
    shooter.is_shooter_ready = SHOOTER_IDLE;

    // 强行将发射命令设置为空闲，防止连发或者状态更新错误
    shooter.shooter_command = SHOOTER_COMMAND_IDLE;

    shooter.work_state = SHOOTER_WORK_STATE_IDLE;

    // 关闭气动阀
    Valve25Send(VALVE25_OFF);
    Valve23Send(VALVE23_OFF);
}


void ShootOnWorkState(uint8_t work_state)
{
    switch (work_state)
    {
        case SHOOTER_WORK_STATE_IDLE        : ShooterOnIdle();          break;
        case SHOOTER_WORK_STATE_VALVE25_WAIT: ShooterOnValve25Wait();   break;
        case SHOOTER_WORK_STATE_VALVE23_WAIT: ShooterOnValve23Wait();   break;
        case SHOOTER_WORK_STATE_FINISH      : ShooterOnFinish();        break;
        default:    break;
    }
}

void ShooterTask(void)
{
    HAL_IWDG_Refresh(&hiwdg);
    static uint8_t valve23_wait_time = 0;
    static uint8_t valve25_wait_time = 0;
    GetPhotogateState();
    GetProportionalValveState();
    AirPressureControl();
    JudgeAirBottleState();


    if(shooter.is_air_bottle_ready)
    {
        if(shooter.is_airpre_ready)
        {
            // 刚上电时，为了保证弹道清空，需要先空发
            // if(shooter.first_run_flag == 0)
            // {
            //     if(shooter.work_state != SHOOTER_WORK_STATE_FINISH)
            //     {
            //         shooter.is_shooter_ready = SHOOTER_ACTIVED;
            //         ShootOnWorkState(shooter.work_state);
            //         // ShootOnWorkState(SHOOTER_WORK_STATE_VALVE25_WAIT);
            //     }
            //     else if(shooter.work_state == SHOOTER_WORK_STATE_FINISH)
            //     {
            //         ShootOnWorkState(shooter.work_state);
            //         shooter.first_run_flag = 1;
            //     }
            // }

            // 收到发射指令
            if(shooter.shooter_command == SHOOTER_COMMAND_ACTIVE || shooter.shooter_command == SHOOTER_COMMAND_NEXT)
            {
                ShootOnWorkState(shooter.work_state); 
            }
        }
    }
}

/*
uint8_t Shoot(void)
{
    static uint16_t valve25_ready_wait_time = 0;
    static uint16_t valve23_ready_wait_time = 0;
    static uint8_t shoot_start_flag = 0;
    static uint8_t shoot_finish_flag = 0;
    static uint8_t shooter_wait_time = 0;
    static uint8_t valce23_flag = 0;
    if(shooter.phtogate_state == PHOTOGATE_EMPTY && shooter.shooter_command == 1)
    {
        // 供弹没有到位返回拨弹环节
        return SHOOTER_PILL_EMPTY;
    }
    else
    {
        // 通过供弹检测进入发射环节
        if (shooter.shooter_command == 1)
        {
            shooter.shooter_command = 2;
        }   
    }
    
    if(valve25_ready_wait_time < 50)
    {
        Valve25Send(VALVE25_ON);
        valve25_ready_wait_time++;
        return SHOOTER_VALVE25_WAIT;
    }
    else
    {
        if(Air_Pressure_Control() == AIR_PRESSURE_ERR && valce23_flag == 0)
        {
           return SHOOTER_AIR_PRESSURE_ERR;
        }
        else
        {
            valce23_flag = 1;
            if(shooter.shooter_command == 3)
            {
                Valve23Send(VALVE23_ON);
                valve23_ready_wait_time++;
                return SHOOTER_VALVE23_WAIT;
            }
        }
    }
    if(shooter.shooter_command == 3)
    {
        Valve23Send(VALVE23_OFF);
        Valve25Send(VALVE25_OFF);
        valve23_ready_wait_time = 0;
        valve25_ready_wait_time = 0;
        valce23_flag = 0;
        return SHOOTER_FINISH;
    }
    
}
*/

void ShooterInit(void)
{
    shooter.proportional_valve_fdb = 0;
    shooter.proportional_valve_ref = 2;
    shooter.shooter_command = 0;
    shooter.valve23_target = 0;
    shooter.valve25_target = 0;
    shooter.is_airpre_ready = 0;
    shooter.is_air_bottle_ready = 0;
    shooter.is_shooter_ready = 0;
    shooter.valve23_wait_time = 0;
    shooter.valve25_wait_time = 0;
    // shooter.work_state = SHOOTER_WORK_STATE_VALVE25_WAIT;
    shooter.work_state = 0;
    shooter.first_run_flag = 0;

    Valve23Send(VALVE23_OFF);
    Valve25Send(VALVE25_OFF);
}
