/** 
 *******************************************************************************
 * @file      : mini_gimbal_fsm.cpp
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
#include "mini_gimbal_fsm.hpp"

#include "gimbal_fsm.hpp"
/* Private macro -------------------------------------------------------------*/
namespace hero
{
/* Private constants ---------------------------------------------------------*/
const float kUseScopeAng = 0.0f;
const float kNotUseScopeAng = 130.0f;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
#pragma region update functions

void MiniGimbalFsm::update()
{
  updateData();
  WorkState state = updateWorkState();
  if (state != work_state_) {
    work_state_ = state;
  }
};

void MiniGimbalFsm::updateData()
{
  work_tick_ = getCurrentTick();

  updateChassisBoard();
};

void MiniGimbalFsm::updateChassisBoard()
{
  OfflineChecker *oc_ptr = oc_ptr_[kOCIChassis];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Gimbal offline checker is nullptr", oc_ptr);
  if (oc_ptr->isOffline()) {
    is_chassis_board_ready_ = false;
    is_use_scope_ = false;
    mini_gimbal_ctrl_flag_ = 0;
    mini_gimbal_ang_delta_ = 0;
    setWorkingMode(WorkingMode::kGimbalNormalMode);
  } else {
    HW_ASSERT(gc_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr", gc_comm_ptr_);
    is_chassis_board_ready_ = gc_comm_ptr_->isChassisBoardReady();
    is_use_scope_ = gc_comm_ptr_->gimbal_data().use_scope_flag;
    working_mode_ = (WorkingMode)gc_comm_ptr_->gimbal_data().working_mode;
    mini_gimbal_ctrl_flag_ = gc_comm_ptr_->gimbal_data().mini_gimbal_ctrl_flag;
    mini_gimbal_ang_delta_ = gc_comm_ptr_->gimbal_data().getPitchDeltaFloat();
    setWorkingMode((WorkingMode)gc_comm_ptr_->gimbal_data().working_mode);
  }
};

MiniGimbalFsm::WorkState MiniGimbalFsm::updateWorkState()
{
  // 底盘掉线时，切到死亡状态
  if (!is_chassis_board_ready_) {
    return WorkState::kDead;
  } else {
    // 底盘在线时，云台工作状态与底盘保持一致
    return (WorkState)(gc_comm_ptr_->gimbal_data().work_state);
  }
};

#pragma endregion

#pragma region control functions

void MiniGimbalFsm::run()
{
  if (work_state_ == WorkState::kDead) {
    runOnDead();
  } else if (work_state_ == WorkState::kResurrection) {
    runOnResurrection();
  } else if (work_state_ == WorkState::kWorking) {
    runOnWorking();
  } else {
    // 其他状态，认为是死亡状态
    runOnDead();
  }
};

void MiniGimbalFsm::runOnDead()
{
  // 失能舵机
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(servo_ptr_[i] != nullptr, "pointer to servo %d is nullptr", i);
    servo_ptr_[i]->disable();
  }
  resetDataOnDead();
  setCommDataOnDead();
};

void MiniGimbalFsm::runOnResurrection()
{
  // 失能舵机
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(servo_ptr_[i] != nullptr, "pointer to servo %d is nullptr", i);
    servo_ptr_[i]->disable();
  }
  resetDataOnResurrection();
  setCommDataOnResurrection();
};

void MiniGimbalFsm::runOnWorking()
{
  // 使能舵机
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(servo_ptr_[i] != nullptr, "pointer to servo %d is nullptr", i);
    servo_ptr_[i]->enable();
  }
  calcMiniGimbalAngle();
  setCommDataOnWorking();
};

void MiniGimbalFsm::calcMiniGimbalAngle()
{
  if (mini_gimbal_ctrl_flag_) {
    float k = 0.01f;  // 灵敏度
    mini_gimbal_ang_ += k * mini_gimbal_ang_delta_;
    mini_gimbal_ang_ = hello_world::Bound(mini_gimbal_ang_, 60, 90);
  }
};

#pragma endregion

#pragma region communicate functions

void MiniGimbalFsm::setCommDataOnDead()
{
  // 机器人死亡时
  // 舵机都发送无效数据
  // 正常模式，舵机角度为90，保持与云台pitch水平
  servo_ptr_[kServoIdxMiniPitch]->setAngle(90);
  // 不使用倍镜
  servo_ptr_[kServoIdxScope]->setAngle(kNotUseScopeAng);
};

void MiniGimbalFsm::setCommDataOnResurrection()
{
  // 机器人复活时
  // 舵机都发送无效数据
  // 正常模式，舵机角度为90，保持与云台pitch水平
  servo_ptr_[kServoIdxMiniPitch]->setAngle(90);
  // 不使用倍镜
  servo_ptr_[kServoIdxScope]->setAngle(kNotUseScopeAng);
};

void MiniGimbalFsm::setCommDataOnWorking()
{
  // 机器人工作时
  // 舵机发送指定角度
  HW_ASSERT(servo_ptr_[kServoIdxMiniPitch] != nullptr, "pointer to servo %d is nullptr", kServoIdxMiniPitch);
  if (working_mode_ == WorkingMode::kGimbalFarshootMode) {
    // 吊射模式，舵机角度为预先存储的角度mini_gimbal_ang_
    servo_ptr_[kServoIdxMiniPitch]->setAngle(mini_gimbal_ang_);
  } else {
    // 正常模式，舵机角度为90，保持与云台pitch水平
    servo_ptr_[kServoIdxMiniPitch]->setAngle(90);
  }

  HW_ASSERT(servo_ptr_[kServoIdxScope] != nullptr, "pointer to servo %d is nullptr", kServoIdxScope);
  if (is_use_scope_ && (working_mode_ == kGimbalFarshootMode)) {
    // 使用倍镜
    servo_ptr_[kServoIdxScope]->setAngle(kUseScopeAng);
  } else {
    // 不使用倍镜
    servo_ptr_[kServoIdxScope]->setAngle(kNotUseScopeAng);
  }

  gc_comm_ptr_->gimbal_data().mini_pitch_ = mini_gimbal_ang_;
};

#pragma endregion

#pragma region reset functions

/** 
 * @brief 重置除指针之外的所有数据，使状态机回到初始状态
 */
void MiniGimbalFsm::reset()
{
  work_state_ = WorkState::kDead;  ///< 工作状态

  // work_tick_ = 0;                   ///< 记录云台模块的运行时间，单位为 ms

  // 在 runOnWorking 函数中更新的数据
  working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一次工作模式
  mini_gimbal_ang_ = 90;

  // 从底盘拿到的数据
  is_chassis_board_ready_ = false;  ///< 底盘是否就绪
  is_use_scope_ = false;
  memset(&mini_gimbal_ang_delta_, 0, sizeof(mini_gimbal_ang_delta_));
};

void MiniGimbalFsm::resetDataOnDead()
{
  // 在 runOnWorking 函数中更新的数据
  working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一次工作模式
  // memset(&mini_gimbal_ang_, 0, sizeof(mini_gimbal_ang_));  ///< 死亡不清除角度数据

  // 从底盘拿到的数据
  is_chassis_board_ready_ = false;  ///< 底盘是否就绪
  is_use_scope_ = false;
  memset(&mini_gimbal_ang_delta_, 0, sizeof(mini_gimbal_ang_delta_));
};

void MiniGimbalFsm::resetDataOnResurrection()
{
  // 在 runOnWorking 函数中更新的数据
  working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一次工作模式
  // memset(&mini_gimbal_ang_, 0, sizeof(mini_gimbal_ang_));  ///< 死亡不清除角度数据

  // 从底盘拿到的数据
  is_chassis_board_ready_ = false;  ///< 底盘是否就绪
  is_use_scope_ = false;
  memset(&mini_gimbal_ang_delta_, 0, sizeof(mini_gimbal_ang_delta_));
};

#pragma endregion

#pragma region tool functions

void MiniGimbalFsm::setWorkingMode(WorkingMode mode)
{
  last_working_mode_ = working_mode_;
  if (working_mode_ != mode) {
    working_mode_ = mode;
  }
};

#pragma endregion

#pragma region register functions

void MiniGimbalFsm::registerServo(Servo *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to servo %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kMotorNum, "index of servo out of range", idx);
  servo_ptr_[idx] = ptr;
};

void MiniGimbalFsm::registerOfflineChecker(OfflineChecker *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to OfflineChecker is nullptr", ptr);
  HW_ASSERT(idx >= 0 && idx < kOCINum, "index of OfflineChecker out of range", idx);
  oc_ptr_[idx] = ptr;
};

void MiniGimbalFsm::registerGimbalChassisComm(GimbalChassisComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gc_comm_ptr_ = ptr;
};

#pragma endregion

/* Private function definitions ----------------------------------------------*/

}  // namespace hero