/** 
 *******************************************************************************
 * @file      : ins_offline_checker.cpp
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ins_offline_checker.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hero::OfflineChecker unique_oc_motor_yaw(50);
hero::OfflineChecker unique_oc_motor_pitch(50);
// CHANGE 气动没有摩擦轮结构这边注释
// hero::OfflineChecker unique_oc_motor_fric_left(50);
// hero::OfflineChecker unique_oc_motor_fric_right(50);

// CHANGE 增加发射机构的offline checker
hero::OfflineChecker unique_oc_shooter_control_board(50);
hero::OfflineChecker unique_oc_chassis_control_board(50);
hero::OfflineChecker unique_oc_vision(100);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hero::OfflineChecker* CreateOcMotorYaw() { return &unique_oc_motor_yaw; };
hero::OfflineChecker* CreateOcMotorPitch() { return &unique_oc_motor_pitch; };
// CHANGE 气动没有摩擦轮结构这边注释
// hero::OfflineChecker* CreateOcMotorFricLeft() { return &unique_oc_motor_fric_left; };
// hero::OfflineChecker* CreateOcMotorFricRight() { return &unique_oc_motor_fric_right; };

// CHANGE 增加发射机构的offline checker
hero::OfflineChecker* CreateOcShooterControlBoard() { return &unique_oc_shooter_control_board; };
hero::OfflineChecker* CreateOcChassisControlBoard() { return &unique_oc_chassis_control_board; };
hero::OfflineChecker* CreateOcVision() { return &unique_oc_vision; };

/* Private function definitions ----------------------------------------------*/
