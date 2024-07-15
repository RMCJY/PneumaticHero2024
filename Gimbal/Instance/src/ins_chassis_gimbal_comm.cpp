/** 
 *******************************************************************************
 * @file      : ins_chassis_gimbal_comm.cpp
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
#include "ins_chassis_gimbal_comm.hpp"
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hero::GimbalChassisComm unique_gimbal_chassis_comm = hero::GimbalChassisComm(hero::GimbalChassisComm::CodePart::kCodePartInGimbal, 0x01, 0x02);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hero::GimbalChassisComm* CreateGimbalChassisComm(void) { return &unique_gimbal_chassis_comm; };
/* Private function definitions ----------------------------------------------*/
