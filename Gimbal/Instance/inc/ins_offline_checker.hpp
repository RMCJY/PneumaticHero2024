/** 
 *******************************************************************************
 * @file      : ins_offline_checker.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INSTANCE_INS_OFFLINE_CHECKER_HPP_
#define INSTANCE_INS_OFFLINE_CHECKER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "offline_checker.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

hero::OfflineChecker* CreateOcMotorYaw();
hero::OfflineChecker* CreateOcMotorPitch();
// CHANGE 气动没有摩擦轮结构这边注释
// hero::OfflineChecker* CreateOcMotorFricLeft();
// hero::OfflineChecker* CreateOcMotorFricRight();

// CHANGE 增加发射机构的offline checker
hero::OfflineChecker* CreateOcShooterControlBoard();

hero::OfflineChecker* CreateOcChassisControlBoard();
hero::OfflineChecker* CreateOcVision();

#endif /* INSTANCE_INS_OFFLINE_CHECKER_HPP_ */
