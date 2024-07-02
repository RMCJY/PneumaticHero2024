
/** 
 *******************************************************************************
 * @file      : ins_fsm.hpp
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
#ifndef INSTANCE_INS_FSM_HPP_
#define INSTANCE_INS_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
//#include "chassis_fsm.hpp"
//#include "gimbal_fsm.hpp"
#include "shooter_fsm.hpp"
#include "main_fsm.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

// hero::ChassisFsm* CreateChassisFsm();
// hero::GimbalFsm* CreateGimbalFsm();
hero::MainFsm* CreateMainFsm();
hero::ShooterFsm* CreateShooterFsm();

/* Exported function prototypes ----------------------------------------------*/

#endif /* INSTANCE_INS_FSM_HPP_ */
