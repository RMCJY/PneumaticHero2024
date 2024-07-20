/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-27 14:16:18
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-01-30 17:06:17
 * @FilePath: \LOWER_FINAL\Task\HW_CAN.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __HW_CAN_H
#define __HW_CAN_H

/* ------------------------------ Include ------------------------------ */
#include "can.h"
#include "main.h"
#include "main_task.h"

/* ------------------------------ Macro Definition ------------------------------ */
#define GIMBAL_ADDR 0x100
#define SPEED_ADRR      0x1f1       // 仅测试使用，用于查看弹速

/* ------------------------------ Function Declaration ------------------------------ */
void CanFilter_Init(CAN_HandleTypeDef* hcan_init);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan_rx);
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t* msg, uint32_t id, uint8_t len);

#endif