/** 
 *******************************************************************************
 * @file      : ins_imu.cpp
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
#include "ins_imu.hpp"
/* Private constants ---------------------------------------------------------*/
const hero::ImuInitParams kImuInitParams;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hero::Imu unique_imu = hero::Imu(kImuInitParams);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hero::Imu *CreateImu(void) { return &unique_imu; };
/* Private function definitions ----------------------------------------------*/
