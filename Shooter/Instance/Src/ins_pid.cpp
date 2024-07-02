/** 
 *******************************************************************************
 * @file      : ins_pid.cpp
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
#include "ins_pid.hpp"
#include "proportional_valve_control.hpp"
#include "motor.hpp"

/* Private constants ---------------------------------------------------------*/

const float kMaxPidOutProportionalValve = 5.0f;

const hw_pid::OutLimit kOutLimitProportionalValve = hw_pid::OutLimit(true, 0, kMaxPidOutProportionalValve);

const hw_pid::MultiNodesPid::ParamsList kPidParamsProportionalValve = {
    {
     .auto_reset = true,
     .kp = 0,
     .ki = 0,
     .kd = 0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 1.0, 0.1),
     .period_sub = hw_pid::PeriodSub(true, 2 * PI),
     .out_limit = kOutLimitProportionalValve,
    },

};

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kMultiNodesPidTypeCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_pid::MultiNodesPid unique_pid_proportional_valve(kPidTypeCascade, kOutLimitProportionalValve, kPidParamsProportionalValve);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid* CreatePidProportionalValve() { return &unique_pid_proportional_valve; };

/* Private function definitions ----------------------------------------------*/
