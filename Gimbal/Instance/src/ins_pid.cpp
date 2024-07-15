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

#include "motor.hpp"

/* Private constants ---------------------------------------------------------*/
const float kMaxPidOutYaw = 6.9f;

// CHANGE 气动使用3508屁股控制pitch,这边修改
const float kMaxPidOutPitch = 16384.0f;
// const float kMaxPidOutPitch = 6.9f;
// const float kMaxPidOutFric = 16384.0f;

const hw_pid::OutLimit kOutLimitYaw = hw_pid::OutLimit(true, -kMaxPidOutYaw, kMaxPidOutYaw);
const hw_pid::OutLimit kOutLimitPitch = hw_pid::OutLimit(true, -kMaxPidOutPitch, kMaxPidOutPitch);
// const hw_pid::OutLimit kOutLimitFric = hw_pid::OutLimit(true, -kMaxPidOutFric, kMaxPidOutFric);

const hw_pid::MultiNodesPid::ParamsList kPidParamsYaw = {
    {
     .auto_reset = true,
     .kp = 24,
     .ki = 0.005,
     .kd = 0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
     .period_sub = hw_pid::PeriodSub(true, 2 * PI),
     .inte_anti_windup = hw_pid::InteAntiWindup(true, -2.0f, 2.0f),
     .diff_filter = hw_pid::DiffFilter(false, -0.0f, 0.0f, 0.0f),
     .out_limit = hw_pid::OutLimit(true, -40, 40),
     },
    {
     .auto_reset = true,
     .kp = 2.0,
     .ki = 0,
     .kd = 0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
     .period_sub = hw_pid::PeriodSub(false, 0),
     .diff_filter = hw_pid::DiffFilter(true, 0.0f, 0.0f, 0.0f),
     .diff_previous = hw_pid::DiffPrevious(true, 0.5f),
     .out_limit = kOutLimitYaw,
     },
};

const hw_pid::MultiNodesPid::ParamsList kPidParamsPitch = {
    {
     .auto_reset = true,
     .kp = 20,
     .ki = 0.005,
     .kd = 0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
     .period_sub = hw_pid::PeriodSub(true, 2 * PI),
     .inte_anti_windup = hw_pid::InteAntiWindup(true, -1.0f, 1.0f),
     .diff_filter = hw_pid::DiffFilter(false, -0.0f, 0.0f, 0.0f),
     .out_limit = hw_pid::OutLimit(true, -30, 30),
     },
    {
     .auto_reset = true,
     .kp = 1.0,
     .ki = 0,
     .kd = 0,
     .setpoint_ramping = hw_pid::SetpointRamping(false, -0.1, 0.1, 0.1),
     .period_sub = hw_pid::PeriodSub(false, 0),
     .diff_filter = hw_pid::DiffFilter(false, 0.0f, 0.0f, 0.0f),
     .diff_previous = hw_pid::DiffPrevious(false, 0.5f),
     .out_limit = kOutLimitPitch,
     },
};

// CHANGE 气动没有摩擦轮结构这边注释
// const hw_pid::MultiNodesPid::ParamsList kPidParamsFric = {
//     {
//      .auto_reset = true,
//      .kp = 100,
//      .ki = 0,
//      .kd = 0,
//      .out_limit = kOutLimitFric,
//      },
// };

const hw_pid::MultiNodesPid::Type kPidTypeCascade = hw_pid::MultiNodesPid::Type::kMultiNodesPidTypeCascade;
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_pid::MultiNodesPid unique_pid_yaw(kPidTypeCascade, kOutLimitYaw, kPidParamsYaw);
hw_pid::MultiNodesPid unique_pid_pitch(kPidTypeCascade, kOutLimitPitch, kPidParamsPitch);

// CHANGE 气动没有摩擦轮结构这边注释
// hw_pid::MultiNodesPid unique_pid_fric_left(kPidTypeCascade, kOutLimitFric, kPidParamsFric);
// hw_pid::MultiNodesPid unique_pid_fric_right(kPidTypeCascade, kOutLimitFric, kPidParamsFric);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hw_pid::MultiNodesPid* CreatePidMotorYaw() { return &unique_pid_yaw; };
hw_pid::MultiNodesPid* CreatePidMotorPitch() { return &unique_pid_pitch; };

// CHANGE 气动没有摩擦轮结构这边注释
// hw_pid::MultiNodesPid* CreatePidMotorFricLeft() { return &unique_pid_fric_left; };
// hw_pid::MultiNodesPid* CreatePidMotorFricRight() { return &unique_pid_fric_right; };

/* Private function definitions ----------------------------------------------*/
