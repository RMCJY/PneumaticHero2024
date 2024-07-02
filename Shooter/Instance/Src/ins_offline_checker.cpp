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

hero::OfflineChecker unique_oc_proportional_valve(50);
hero::OfflineChecker unique_oc_gimbal_control_board(50);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hero::OfflineChecker* CreatOcProportionalValve() { return &unique_oc_proportional_valve; };
hero::OfflineChecker* CreateOcGimbalControlBoard() { return &unique_oc_gimbal_control_board; };

/* Private function definitions ----------------------------------------------*/
