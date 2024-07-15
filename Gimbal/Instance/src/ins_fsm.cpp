/** 
 *******************************************************************************
 * @file      : ins_fsm.cpp
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
#include "ins_fsm.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hero::GimbalFsm unique_gimbal_fsm = hero::GimbalFsm();
hero::MiniGimbalFsm unique_mini_gimbal_fsm = hero::MiniGimbalFsm();
hero::ShooterFsm unique_shooter_fsm = hero::ShooterFsm();
hero::MainFsm unique_main_fsm = hero::MainFsm();
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hero::GimbalFsm* CreateGimbalFsm() { return &unique_gimbal_fsm; };
hero::MiniGimbalFsm* CreateMiniGimbalFsm() { return &unique_mini_gimbal_fsm; };
hero::ShooterFsm* CreateShooterFsm() { return &unique_shooter_fsm; };
hero::MainFsm* CreateMainFsm() { return &unique_main_fsm; };
/* Private function definitions ----------------------------------------------*/
