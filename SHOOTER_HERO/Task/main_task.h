/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-27 14:16:18
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-06-08 10:33:21
 * @FilePath: \LOWER_FINAL\Task\control.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __MAIN_TASK_H
#define __MAIN_TASK_H

/* ------------------------------ Include ------------------------------ */
#include "main.h"

/* ------------------------------ Macro Definition ------------------------------ */
#define SHOOT_ID    0x1ff

/* ------------------------------ Type Definition ------------------------------ */
//上位机指令
typedef struct 
{
    uint8_t shoot_command;
} upcommand_t;


// typedef struct
// {
//     uint8_t is_air_bottle_ready;
//     uint8_t is_airpre_ready;
//     uint8_t valve23_state;
//     uint8_t valve25_state;
//     float proportional_valve_ref;
//     float proportional_valve_fdb;
// } shooter_state_t;


// shooter_state_t shooter_state;


void HardwareInit(void);

#endif