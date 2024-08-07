/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-06-07 23:01:37
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-06-08 20:20:55
 * @FilePath: \PCB_Test\Task\shooter.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SHOOTER_H
#define __SHOOTER_H

#include "main.h"
#include "stdint.h"

#define PHOTOGATE_COVER     1
#define PHOTOGATE_EMPTY     0

#define VALVE23_ON        1
#define VALVE23_OFF       0

#define VALVE25_ON        1
#define VALVE25_OFF       0

#define AIR_PRESSURE_READY  1
#define AIR_PRESSURE_ERR    0

#define AIR_BOTTLE_READY    1
#define AIR_BOTTLE_ERR      0

#define SHOOTER_ACTIVED      1
#define SHOOTER_IDLE        0

#define SHOOTER_COMMAND_ACTIVE     1
#define SHOOTER_COMMAND_NEXT        2       // 仅在23/25阀分开测试的时候使用
#define SHOOTER_COMMAND_IDLE        0

#define SHOOTER_PILL_EMPTY          0
#define SHOOTER_VALVE25_WAIT        1
#define SHOOTER_VALVE23_WAIT        2
#define SHOOTER_AIR_PRESSURE_ERR    3
#define SHOOTER_WAIT                4
#define SHOOTER_FINISH              10

#define SHOOTER_WORK_STATE_IDLE             0
#define SHOOTER_WORK_STATE_VALVE23_WAIT     1
#define SHOOTER_WORK_STATE_VALVE25_WAIT     2
#define SHOOTER_WORK_STATE_FINISH           3

#define BULLET_NOTE         1
#define BULLET_READY        2

#define SHOOTER_CLEAR_NOTE  1
#define SHOOTER_CLEAR_READY 0

#define SHOOTER_RFR_POWER_ON    1
#define SHOOTER_RFR_POWER_OFF   0

typedef struct 
{
    uint8_t valve23_target;
    uint8_t valve25_target;
    uint8_t phtogate_state;        
    uint8_t first_run_flag;          // 首次进入或者从异常状态重新返回正常，需要空发保证弹道清空
    uint16_t valve23_wait_time;
    uint16_t valve25_wait_time;
    float proportional_valve_ref;
    float proportional_valve_fdb;

    // shooter to gimbal
    uint8_t is_shooter_ready;       // 是否处于允许发射状态，正在发射过程中拨弹会造成卡弹
    uint8_t bullet_feed_note;       // 拨弹没有到位就下发发射指令时，反馈提醒拨弹
    uint8_t shooter_clear_note;     // 发射机构从断电到上电的过程中，或者首次启动的过程中，可能需要操作手空发来清空弹道
    uint8_t work_state;             // 发射状态
    uint8_t is_air_bottle_ready;    // 气瓶是否准备好，不处于欠压状态
    uint8_t is_airpre_ready;        // 气压是否到达设定值
    uint8_t airpre;

    // gimbal to shooter
    uint8_t is_rfr_shooter_power_on;// 从裁判系统读取发射机构是否上电
    uint8_t shooter_signal;         // 从云台接收到的发射指令
    float rfr_bullet_speed;         // 裁判系统读取的射速，暂时没用，后续可能会根据读取射速调整气压形成闭环？
} Shooter_t;



extern Shooter_t shooter;

void ShooterTask(void);
void ShooterInit(void);

#endif