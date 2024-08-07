/** 
 *******************************************************************************
 * @file      : ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private constants ---------------------------------------------------------*/

const hw_motor::OptionalParams kMotorParamsYaw = {
    .input_type = hw_motor::kInputTypeTorq,
    .angle_range = hw_motor::kAngleRangeNegPiToPosPi,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = false,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    // .ex_redu_rat = 14,
};


// const hw_motor::OptionalParams kMotorParamsPitch = {
//     .input_type = hw_motor::kInputTypeTorq,
//     .angle_range = hw_motor::kAngleRangeNegPiToPosPi,
//     .dir = hw_motor::kDirFwd,
//     /** 是否移除电机自带的减速器 */
//     .remove_build_in_reducer = false,
//     /** 电机输出端实际角度与规定角度的差值 */
//     .angle_offset = 0,
//     /** 电机外置减速器的减速比（额外） */
//     // .ex_redu_rat = 14,
// };

// CHANGE 气动的由3508的屁股控制pitch,这边修改参数
const hw_motor::OptionalParams kMotorParamsPitch = {
    .input_type = hw_motor::kInputTypeRaw,
    .angle_range = hw_motor::kAngleRangeNegInfToPosInf,
    .dir = hw_motor::kDirFwd,
    /** 是否移除电机自带的减速器 */
    .remove_build_in_reducer = true,
    /** 电机输出端实际角度与规定角度的差值 */
    .angle_offset = 0,
    /** 电机外置减速器的减速比（额外） */
    .ex_redu_rat = 2.0 * PI / 5.0,        // 电机旋转角度映射为丝杆运动过的里程
};

// CHANGE 气动没有摩擦轮结构这边注释
// const hw_motor::OptionalParams kMotorParamsFricLeft = {
//     .input_type = hw_motor::kInputTypeRaw,
//     .angle_range = hw_motor::kAngleRangeNegPiToPosPi,
//     .dir = hw_motor::kDirFwd,
//     /** 是否移除电机自带的减速器 */
//     .remove_build_in_reducer = true,
//     /** 电机输出端实际角度与规定角度的差值 */
//     .angle_offset = 0,
// };

// const hw_motor::OptionalParams kMotorParamsFricRight = {
//     .input_type = hw_motor::kInputTypeRaw,
//     .angle_range = hw_motor::kAngleRangeNegPiToPosPi,
//     .dir = hw_motor::kDirRev,
//     /** 是否移除电机自带的减速器 */
//     .remove_build_in_reducer = true,
//     /** 电机输出端实际角度与规定角度的差值 */
//     .angle_offset = 0,
// };

enum MotorID {
// CHANGE 气动没有摩擦轮结构这边注释
//   kMotorIdFricRight = 1u,
//   kMotorIdFricLeft = 2u,
  kMotorIdYaw = 3u,
  kMotorIdPitch = 1u,
};
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

hw_motor::DM_J4310 unique_motor_yaw = hw_motor::DM_J4310(kMotorIdYaw, kMotorParamsYaw);

// CHANGE 气动的由3508控制pitch
// hw_motor::DM_J4310 unique_motor_pitch = hw_motor::DM_J4310(kMotorIdPitch, kMotorParamsPitch);
hw_motor::M3508 unique_motor_pitch = hw_motor::M3508(kMotorIdPitch, kMotorParamsPitch);

// CHANGE 气动没有摩擦轮结构这边注释
// hw_motor::M3508 unique_motor_fric_left = hw_motor::M3508(kMotorIdFricLeft, kMotorParamsFricLeft);
// hw_motor::M3508 unique_motor_fric_right = hw_motor::M3508(kMotorIdFricRight, kMotorParamsFricRight);

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

hw_motor::Motor* CreateMotorYaw() { return &unique_motor_yaw; };
hw_motor::Motor* CreateMotorPitch() { return &unique_motor_pitch; };

// CHANGE 气动没有摩擦轮结构这边注释
// hw_motor::Motor* CreateMotorFricLeft() { return &unique_motor_fric_left; };
// hw_motor::Motor* CreateMotorFricRight() { return &unique_motor_fric_right; };

/* Private function definitions ----------------------------------------------*/
