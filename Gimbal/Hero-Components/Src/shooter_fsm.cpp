/** 
 *******************************************************************************
 * @file      : shooter_fsm.cpp
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
#include "shooter_fsm.hpp"
/* Private macro -------------------------------------------------------------*/
namespace hero
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
#pragma region update functions

void ShooterFsm::update()
{
  updateData();
  WorkState state = updateWorkState();
  if (state != work_state_) {
    work_state_ = state;
  }
}

void ShooterFsm::updateData()
{
  work_tick_ = getCurrentTick();

  updateChassisBoard();
  updateShooterBoard();
  updateVision();
};

/**
 * @brief 更新底盘板状态
 * 
 * 当底盘板通讯丢失时，认为无底盘板。此时，云台板始终处于未准备的状态。
 * 如果底盘板通讯正常，会检查底盘板是否准备就绪。
 */
void ShooterFsm::updateChassisBoard()
{
  // 当云台板通讯丢失时，认为无云台板
  // 此时，直接认为云台板准备就绪，使得可以在无云台板下工作
  OfflineChecker *oc_ptr = oc_ptr_[kOCIChassis];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Gimbal offline checker is nullptr", oc_ptr);
  if (oc_ptr->isOffline()) {
    is_chassis_board_ready_ = false;
  } else {
    HW_ASSERT(gc_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr", gc_comm_ptr_);
    is_chassis_board_ready_ = gc_comm_ptr_->shooter_data().work_state != 0;
  }
};

// CHANGE 气动的发射由发射机构主控板控制
/**
 * @brief 更新发射机构主控板状态
 * 
 * 当发射机构主控板通讯丢失时，认为无发射机构主控板。
 * 如果发射机构主控板通讯正常，会检查发射机构主控板是否准备就绪。
 */
void ShooterFsm::updateShooterBoard()
{
  // 当发射机构主控板通讯丢失时，认为无发射机构主控板
  // 此时，直接认为发射机构主控板板准备就绪，使得可以在无发射机构主控板下工作
  OfflineChecker *oc_ptr = oc_ptr_[kOCIShooter];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Shooter offline checker is nullptr", oc_ptr);
  if(oc_ptr->isOffline())
  {
    is_shooter_board_ready_ = false;
    is_shooter_ready_ = false;
    is_air_bottle_ready_ = false;
  }
  else
  {
    HW_ASSERT(gs_comm_ptr_ != nullptr, "pointer to GimbalShooterComm is nullptr", gs_comm_ptr_);
    // shooter和gimbal通信获得信息
    is_shooter_board_ready_ = gs_comm_ptr_->isShooterBoardReady();
    is_shooter_ready_ = gs_comm_ptr_->shooter_data().is_shooter_ready;
    is_air_bottle_ready_ = gs_comm_ptr_->shooter_data().is_air_bottle_ready;
    bullet_feed_note_ = gs_comm_ptr_->shooter_data().bullet_feed_note;

    // 发射信号从gimbal和chassis的通信信息中获得
    is_rfr_shooter_power_on_ = gc_comm_ptr_->referee_data().is_rfr_shooter_power_on;
    shoot_signal_ = gc_comm_ptr_->shooter_data().shoot_signal_;
    bullet_speed_fdb_uint_8 = gc_comm_ptr_->referee_data().bullet_speed_uint_8;

  }
}

void ShooterFsm::updateVision()
{
  OfflineChecker *oc_ptr = oc_ptr_[kOCIVision];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Gimbal offline checker is nullptr", oc_ptr);
  if (oc_ptr->isOffline()) {
    auto_shoot_flag_ = false;
  } else {
    auto_shoot_flag_ = vision_ptr_->getRxData().shoot_flag;
  }
};

ShooterFsm::WorkState ShooterFsm::updateWorkState()
{
  // 底盘掉线时，切到死亡状态
  if (!is_chassis_board_ready_) {
    return WorkState::kDead;
  }

  WorkState current_state = work_state_;
  if (current_state == WorkState::kDead) {
    // 死亡状态下，如果底盘上电，则切到复活状态
    OfflineChecker *shooter_ptr = oc_ptr_[kOCIShooter];
    HW_ASSERT(shooter_ptr != nullptr, "pointer to shooter is null", shooter_ptr);
    if (is_chassis_board_ready_ && !(shooter_ptr->isOffline())) {
      return WorkState::kResurrection;
    }
  } else if (current_state == WorkState::kResurrection) {
    // 复活状态下，如果气瓶准备就绪，则切到工作状态
    if (is_air_bottle_ready_) {
      return WorkState::kWorking;
    }
  } else if (current_state == WorkState::kWorking) {
    // 工作状态下，保持当前状态
  } else {
    // 其他状态，认为是死亡状态
    return WorkState::kDead;
  }
  return current_state;
};

#pragma endregion

#pragma region control functions

void ShooterFsm::run()
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
}
void ShooterFsm::runOnDead()
{
  resetDataOnDead();
  setCommDataOnDead();
};

void ShooterFsm::runOnResurrection()
{
  // resetDataOnResurrection();
  // setCommDataOnResurrection();
  //  复活状态下就开始工作
  setCommDataOnWorking();
};

void ShooterFsm::runOnWorking()
{
  setCommDataOnWorking();
};


#pragma endregion

#pragma region communicate functions

void ShooterFsm::setCommDataOnDead()
{
  // 机器人死亡时
  // 电机都发送无效数据
  vision_ptr_->setBulletSpeed(15.5f);

  // CHANGE 更新从gimbal板反馈到chassis板的shooter相关信息
  gc_comm_ptr_->shooter_data().bullet_feed_note = false;
  gc_comm_ptr_->shooter_data().is_air_bottle_ready = false;
  gc_comm_ptr_->shooter_data().is_shooter_ready = false;

  // 更新从gimbal板通信到shooter板的信息
  gs_comm_ptr_->shooter_data().shoot_signal = false;
  gs_comm_ptr_->shooter_data().is_rfr_shooter_power_on = false;
  gs_comm_ptr_->shooter_data().bullet_speed_fdb_uint_8 = bullet_speed_fdb_uint_8;
  
};

void ShooterFsm::setCommDataOnResurrection()
{
  // 机器人复活时
  // 电机都发送无效数据
  vision_ptr_->setBulletSpeed(15.5f);

  gc_comm_ptr_->shooter_data().bullet_feed_note = false;
  gc_comm_ptr_->shooter_data().is_air_bottle_ready = false;
  gc_comm_ptr_->shooter_data().is_shooter_ready = false;

  // 更新从gimbal板通信到shooter板的信息
  gs_comm_ptr_->shooter_data().shoot_signal = false;
  gs_comm_ptr_->shooter_data().is_rfr_shooter_power_on = false;
  gs_comm_ptr_->shooter_data().bullet_speed_fdb_uint_8 = bullet_speed_fdb_uint_8;

};

void ShooterFsm::setCommDataOnWorking()
{
  // 机器人工作时
  // 电机根据期望电流输入发送数据

  float bullet_speed = gc_comm_ptr_->referee_data().getBulletSpeedFloat();
  gc_comm_ptr_->shooter_data().bullet_feed_note = bullet_feed_note_;
  gc_comm_ptr_->shooter_data().is_air_bottle_ready = is_air_bottle_ready_;
  gc_comm_ptr_->shooter_data().is_shooter_ready = is_shooter_ready_;  
  // workstate在main_fsm中更新 

  gs_comm_ptr_->shooter_data().shoot_signal = shoot_signal_;
  gs_comm_ptr_->shooter_data().is_rfr_shooter_power_on = is_rfr_shooter_power_on_;
  gs_comm_ptr_->shooter_data().bullet_speed_fdb_uint_8 = bullet_speed_fdb_uint_8;

  vision_ptr_->setBulletSpeed(bullet_speed);
};

#pragma endregion

#pragma region reset functions

/** 
 * @brief 重置除指针之外的所有数据，使状态机回到初始状态
 */
void ShooterFsm::reset()
{
  // data wiil be updated in `update` function
  work_state_ = WorkState::kDead;

  is_chassis_board_ready_ = false;  ///< 云台主控板是否就绪

  // vision data
  auto_shoot_flag = false;  ///< 自动射击标志位

  // CHANGE shooter data 
  is_shooter_board_ready_ = false;
  is_shooter_ready_ = false;
  is_air_bottle_ready_ = false;
  bullet_feed_note_ = false;

  shoot_signal_ = false;

  // resetPids();
};

void ShooterFsm::resetDataOnDead()
{
  // data will be update in `runOnWorking` function

  is_chassis_board_ready_ = false;  ///< 底盘主控板是否就绪
  auto_shoot_flag = false;          ///< 自动射击标志位

  // CHANGE shooter data 
  bullet_speed_fdb_uint_8 = 0;
  is_rfr_shooter_power_on_ = false;
  is_shooter_board_ready_ = false;
  is_shooter_ready_ = false;
  is_air_bottle_ready_ = false;
  bullet_feed_note_ = false;
  shoot_signal_ = false;

  // resetPids();
}

void ShooterFsm::resetDataOnResurrection()
{
  // data will be update in `runOnWorking` function

  auto_shoot_flag = false;  ///< 自动射击标志位

}


#pragma endregion

#pragma region register functions

void ShooterFsm::registerOfflineChecker(OfflineChecker *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to OfflineChecker is nullptr", ptr);
  HW_ASSERT(idx >= 0 && idx < kOCINum, "index of OfflineChecker out of range", idx);
  oc_ptr_[idx] = ptr;
};
void ShooterFsm::registerGimbalChassisComm(GimbalChassisComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gc_comm_ptr_ = ptr;
};
void ShooterFsm::registerGimbalShooterComm(GimbalShooterComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to GimbalShooterComm is nullptr", ptr);
  gs_comm_ptr_ = ptr;
};
void ShooterFsm::registerVision(Vision *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to Vision is nullptr", ptr);
  vision_ptr_ = ptr;
};

#pragma endregion

/* Private function definitions ----------------------------------------------*/

// TODO：目前这个fsm的主要用途其实就是更新gs_comm_ptr_中的数据有一说一感觉可以去掉这个状态机，在main_fsm中更新
}  // namespace hero