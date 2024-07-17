
#include <math.h>
#include "shooter.h"
#include "gpio.h"
#include "main_task.h"
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
    // TODO：暂时使用开环控制就能达到不错的效果，后续还要研究是否闭环会达到更好的控制效果
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
    // 弹道清空环节不需要管光电门是否检测到弹丸
    if(shooter.phtogate_state == PHOTOGATE_COVER || shooter.shooter_clear_note == SHOOTER_CLEAR_NOTE)
    {
        shooter.is_shooter_ready = SHOOTER_ACTIVED;
        shooter.work_state = SHOOTER_WORK_STATE_VALVE25_WAIT;
        shooter.bullet_feed_note = BULLET_READY;
    }
    else
    {   
        // TODO：完成拨弹和发射指令下发到shooter之间的时间间隔需要考虑，避免重复拨弹发生，以下还需要测试
        // 如果在此状态下光电门没有检测到弹丸，说明拨弹不到位，需要重新拨弹
        shooter.bullet_feed_note = BULLET_NOTE;
    }
}

void ShooterOnValve25Wait(void)
{
    shooter.valve25_wait_time++;
    Valve25Send(VALVE25_ON);
    // 确保气动阀开启后进入下一个状态
    // TODO：这边的等待时间是随便设置的，为了提高发射频率这边的还需要根据测试结果进一步修改
    if(shooter.valve25_wait_time > 300)
    {
        shooter.work_state = SHOOTER_WORK_STATE_VALVE23_WAIT;
    }
}

void ShooterOnValve23Wait(void)
{
    /* 以下仅测试时使用，将23/25开合动作分开*/
    // if(shooter.shooter_signal != SHOOTER_COMMAND_NEXT)
    // {
    //     return 0;   // 只有在按键继续按下的时候，才会进入到这个状态
    // }

    shooter.valve23_wait_time++;
    Valve23Send(VALVE23_ON);
    // 确保气动阀开启并完成发射后进入下一个状态
    // TODO：这边的等待时间是随便设置的，为了提高发射频率这边的还需要根据测试结果进一步修改
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
    // TODO：不确定下面这句话会不会导致当连续下发指令的时候，会丢失一些时间间隔过短的指令，后续需要修改
    shooter.shooter_signal = SHOOTER_COMMAND_IDLE;

    shooter.work_state = SHOOTER_WORK_STATE_IDLE;

    // 关闭气动阀
    Valve25Send(VALVE25_OFF);
    Valve23Send(VALVE23_OFF);


    // 完成一次发射之后弹道清空任务完成
    if(shooter.first_run_flag == 0)
    {
        shooter.first_run_flag = 1;
    }
    if(shooter.shooter_clear_note == SHOOTER_CLEAR_NOTE)
    {
        shooter.shooter_clear_note = SHOOTER_CLEAR_READY;
    }
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

void GetShooterClearNote(void)
{
    // 第一次进入发射状态或者从发射机构下电到上电的过程中，需要空发保证弹道清空
    if(shooter.first_run_flag == 0)
    {
        shooter.shooter_clear_note = SHOOTER_CLEAR_NOTE;
    }
    else
    {
        shooter.shooter_clear_note = SHOOTER_CLEAR_READY;
    }
}

void ShooterTask(void)
{
    HAL_IWDG_Refresh(&hiwdg);
    GetPhotogateState();
    GetProportionalValveState();
    AirPressureControl();
    JudgeAirBottleState();
    GetShooterClearNote();

    // 只有在气瓶正常/气压达到设定值/发射机构上电的情况下才能进入发射状态
    if(shooter.is_air_bottle_ready && shooter.is_airpre_ready && shooter.is_rfr_shooter_power_on )
    {
        // 收到发射指令，保证每个发射周期的完整，防止卡弹
        if(shooter.shooter_signal == SHOOTER_COMMAND_ACTIVE || shooter.work_state != SHOOTER_WORK_STATE_IDLE)
        {
            ShootOnWorkState(shooter.work_state); 
        }
    }

    if(shooter.is_rfr_shooter_power_on == SHOOTER_RFR_POWER_OFF)
    {
        ShooterInit();
    }
}


void ShooterInit(void)
{
    shooter.valve23_target = VALVE23_OFF;
    shooter.valve25_target = VALVE25_OFF;
    shooter.phtogate_state = PHOTOGATE_EMPTY;
    shooter.first_run_flag = 0;
    shooter.valve23_wait_time = 0;
    shooter.valve25_wait_time = 0;
    shooter.proportional_valve_fdb = 0;
    shooter.proportional_valve_ref = 2;

    // shooter to gimbal
    shooter.is_shooter_ready = SHOOTER_IDLE;
    shooter.bullet_feed_note = BULLET_READY;
    shooter.shooter_clear_note = SHOOTER_CLEAR_READY;
    shooter.work_state = SHOOTER_WORK_STATE_IDLE;
    shooter.is_air_bottle_ready = AIR_BOTTLE_ERR;
    shooter.is_airpre_ready = AIR_PRESSURE_ERR;

    // gimbal to shooter
    shooter.is_rfr_shooter_power_on = SHOOTER_RFR_POWER_ON;
    shooter.shooter_signal = SHOOTER_COMMAND_IDLE;
    shooter.rfr_bullet_speed = 0;

    Valve23Send(VALVE23_OFF);
    Valve25Send(VALVE25_OFF);
}
