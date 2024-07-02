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

// hero::ChassisFsm unique_chassis_fsm = hero::ChassisFsm();
// hero::GimbalFsm unique_gimbal_fsm = hero::GimbalFsm();
hero::MainFsm unique_main_fsm = hero::MainFsm();
hero::ShooterFsm unique_shooter_fsm = hero::ShooterFsm();
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

// hero::ChassisFsm* CreateChassisFsm() { return &unique_chassis_fsm; };
// hero::GimbalFsm* CreateGimbalFsm() { return &unique_gimbal_fsm; };
hero::MainFsm* CreateMainFsm() { return &unique_main_fsm; };
hero::ShooterFsm* CreateShooterFsm() { return &unique_shooter_fsm; };
/* Private function definitions ----------------------------------------------*/
