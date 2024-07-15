/** 
 *******************************************************************************
 * @file      : main_fsm.cpp
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
#include "main_fsm.hpp"
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
void MainFsm::update()
{
  updateData();
  WorkState state = updateWorkState();
  if (state != work_state_) {
    work_state_ = state;
  }
};
void MainFsm::updateData()
{
  work_tick_ = getCurrentTick();
  updateImu();
};
void MainFsm::updateImu()
{
  HW_ASSERT(imu_ptr_ != nullptr, "IMU pointer is null", imu_ptr_);
  imu_ptr_->update();
  if (imu_ptr_->isNormWorking()) {
    is_imu_caled_offset_ = true;
  }
};
MainFsm::WorkState MainFsm::updateWorkState()
{
  WorkState current_state = work_state_;
  if (current_state == WorkState::kDead) {
    // 主控板程序在跑就意味着有电，所以直接从死亡状态进入复活状态
    return WorkState::kResurrection;
  } else if (current_state == WorkState::kResurrection) {
    if (is_imu_caled_offset_) {
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

#pragma region run functions
void MainFsm::run()
{
  if (work_state_ == WorkState::kDead) {
    runOnDead();
  } else if (work_state_ == WorkState::kResurrection) {
    runOnResurrection();
  } else if (work_state_ == WorkState::kWorking) {
    runOnWorking();
  } else {
    runOnDead();
  }
};
void MainFsm::runOnDead()
{
  resetDataOnDead();

  HW_ASSERT(gimbal_fsm_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_fsm_ptr_);
  gimbal_fsm_ptr_->reset();

  HW_ASSERT(shooter_fsm_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_fsm_ptr_);
  shooter_fsm_ptr_->reset();

  setCommData();
  sendCommData();
};
void MainFsm::runOnResurrection()
{
  resetDataOnResurrection();

  HW_ASSERT(gimbal_fsm_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_fsm_ptr_);
  gimbal_fsm_ptr_->reset();

  HW_ASSERT(mini_gimbal_fsm_ptr_ != nullptr, "MiniGimbal FSM pointer is null", mini_gimbal_fsm_ptr_);
  mini_gimbal_fsm_ptr_->reset();

  HW_ASSERT(shooter_fsm_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_fsm_ptr_);
  shooter_fsm_ptr_->reset();

  setCommData();
  sendCommData();
};
void MainFsm::runOnWorking()
{
  HW_ASSERT(gimbal_fsm_ptr_ != nullptr, "Gimbal FSM pointer is null", gimbal_fsm_ptr_);
  gimbal_fsm_ptr_->update();
  gimbal_fsm_ptr_->run();

  HW_ASSERT(mini_gimbal_fsm_ptr_ != nullptr, "Gimbal FSM pointer is null", mini_gimbal_fsm_ptr_);
  mini_gimbal_fsm_ptr_->update();
  mini_gimbal_fsm_ptr_->run();

  HW_ASSERT(shooter_fsm_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_fsm_ptr_);
  shooter_fsm_ptr_->update();
  shooter_fsm_ptr_->run();

  setCommData();
  sendCommData();
};
#pragma endregion

#pragma region reset functions
void MainFsm::reset() {};
void MainFsm::resetDataOnDead() {};
void MainFsm::resetDataOnResurrection() {};
#pragma endregion

#pragma region communication functions

void MainFsm::setCommData()
{
  if (work_state_ != WorkState::kWorking) {
    setCommDataMotorStandby();
  }
  setCommDataGimbalChassis();
  setCommDataGimbalShooter();
  setCommDataVison();
};
void MainFsm::setCommDataMotorStandby()
{
  // CHANGE 气动没有摩擦轮结构这边注释
  // MotorIdx motor_idxs[4] = {
  //     kMotorIdxYaw,
  //     kMotorIdxPitch,
  //     kMotorIdxFricLeft,
  //     kMotorIdxFricRight,
  // };
  // for (size_t i = 0; i < 4; i++) {
  //   MotorIdx idx = motor_idxs[i];
  //   Motor *motor_ptr = motor_ptr_[idx];
  //   HW_ASSERT(motor_ptr != nullptr, "%d motor pointer is null", idx);
  //   motor_ptr->setInput(0);
  // }
  MotorIdx motor_idxs[2] = {
      kMotorIdxYaw,
      kMotorIdxPitch,
  };
  for (size_t i = 0; i < 2; i++) {
    MotorIdx idx = motor_idxs[i];
    Motor *motor_ptr = motor_ptr_[idx];
    HW_ASSERT(motor_ptr != nullptr, "%d motor pointer is null", idx);
    motor_ptr->setInput(0);
  }
};
void MainFsm::setCommDataGimbalChassis()
{
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);
  gc_comm_ptr_->main_board_data().gimbal_work_state = (hero::GimbalChassisComm::WorkState)work_state_;

  HW_ASSERT(vision_ptr_ != nullptr, "Vision pointer is null", vision_ptr_);
  gc_comm_ptr_->vision_data().auto_shoot_flag = vision_ptr_->getShootFlag();
  gc_comm_ptr_->vision_data().detected_targets = vision_ptr_->getRxData().target_ids;
  gc_comm_ptr_->vision_data().vtm_x = vision_ptr_->getRxData().vtm_x;
  gc_comm_ptr_->vision_data().vtm_y = vision_ptr_->getRxData().vtm_y;
};
void MainFsm::setCommDataVison()
{
  HW_ASSERT(vision_ptr_ != nullptr, "Vision pointer is null", vision_ptr_);
  HW_ASSERT(gc_comm_ptr_ != nullptr, "GimbalChassisComm pointer is null", gc_comm_ptr_);

  Vision::WorkState vision_work_State = Vision::WorkState::kVwsNotWorking;

  GimbalChassisComm::GimbalData &gimbal_data = gc_comm_ptr_->gimbal_data();
  if (gimbal_data.ctrl_mode == 1) {
    if (gimbal_data.working_mode == 1) {
      vision_work_State = Vision::WorkState::kVwsNoAntiTop;
    } else {
      vision_work_State = Vision::WorkState::kVwsNoAntiTop;
    }
  }
  
  float bullet_speed = gc_comm_ptr_->referee_data().getBulletSpeedFloat();
  if (bullet_speed != 0)
  {
    vision_ptr_->setBulletSpeed(gc_comm_ptr_->referee_data().getBulletSpeedFloat());
  }

  uint8_t robot_color = gc_comm_ptr_->referee_data().robot_color;
  if (robot_color == 1) {
    vision_ptr_->setTargetColor(Vision::Color::kBlue);
  } else if (robot_color == 2) {
    vision_ptr_->setTargetColor(Vision::Color::kRed);
  } else {
    vision_work_State = Vision::WorkState::kVwsNotWorking;
  }

  vision_ptr_->setWorkState(vision_work_State);
};

// TODO：还需要修改
// CHANGE 增加云台和发射机构通信数据设置函数
void MainFsm::setCommDataGimbalShooter()
{
  HW_ASSERT(gs_comm_ptr_ != nullptr, "GimbalShooterComm pointer is null", gs_comm_ptr_);
  gs_comm_ptr_->main_board_data().gimbal_work_state = (hero::GimbalShooterComm::WorkState)work_state_;

  // gs_comm_ptr_->shooter_data().bullet_speed_uint_8
};

void MainFsm::EnableDMYaw()
{
  uint8_t tx_data[8];
  memset(tx_data, 0xFF, sizeof(uint8_t) * 8);
  tx_data[7] = 0xFC;
  SendCanData(&hcan1, motor_ptr_[kMotorIdxYaw]->txId(), tx_data);
};

// CHANGE 使用3508控制pitch，不需要enable电机操作
// void MainFsm::EnableDMPitch()
// {
//   uint8_t tx_data[8];
//   memset(tx_data, 0xFF, sizeof(uint8_t) * 8);
//   tx_data[7] = 0xFC;
//   SendCanData(&hcan2, motor_ptr_[kMotorIdxPitch]->txId(), tx_data);
// };

void MainFsm::DisableDMYaw()
{
  uint8_t tx_data[8];
  memset(tx_data, 0xFF, sizeof(uint8_t) * 8);
  tx_data[7] = 0xFD;
  SendCanData(&hcan1, motor_ptr_[kMotorIdxYaw]->txId(), tx_data);
};

// CHANGE 使用3508控制pitch，不需要disable电机操作
// void MainFsm::DisableDMPitch()
// {
//   uint8_t tx_data[8];
//   memset(tx_data, 0xFF, sizeof(uint8_t) * 8);
//   tx_data[7] = 0xFD;
//   SendCanData(&hcan2, motor_ptr_[kMotorIdxPitch]->txId(), tx_data);
// };

void MainFsm::sendCommData()
{
  sendCanData();
  sendUsartData();
  // CHANGE 气动使用3508屁股控制pitch，这边做修改
  sendPitchData();
  // CHANGE 气动没有摩擦轮结构这边注释
  // sendFricData();
};

void MainFsm::sendCanData()
{
  if (work_state_ != WorkState::kDead) {
    // 新组件库DM有自动使能功能
    sendYawData();
    sendPitchData();
  } else if (work_state_ == WorkState::kResurrection) {
    if (work_tick_ % 100 == 0) {
      // 以10Hz的频率使能能达妙
      EnableDMYaw();
      // CHANGE 使用3508控制pitch，不需要enable电机操作
      // EnableDMPitch();
    }
  } else {
    if (work_tick_ % 100 == 0) {
      // 以10Hz的频率失能达妙
      DisableDMYaw();
      // CHANGE 使用3508控制pitch，不需要disable电机操作
      // DisableDMPitch();
    }
  }
  // 以100Hz的频率与底盘通信
  if (work_tick_ % 10 == 0) {
    sendGimbalChassisCommData();
    // CHANGE 增加云台和发射机构通信数据发送函数
    sendGimbalShooterCommData();
  }
};

void MainFsm::sendYawData()
{
  uint8_t tx_data[8] = {0};
  unsigned int len = 8;
  motor_ptr_[kMotorIdxYaw]->encode(tx_data,&len);
  SendCanData(&hcan1, motor_ptr_[kMotorIdxYaw]->txId(), tx_data);
}

void MainFsm::sendPitchData()
{
  unsigned int len = 8;
  uint8_t tx_data[8] = {0};
  motor_ptr_[kMotorIdxPitch]->encode(tx_data, &len);
  SendCanData(&hcan2, motor_ptr_[kMotorIdxPitch]->txId(), tx_data);
}

// CHANGE 气动没有摩擦轮结构这边注释
// void MainFsm::sendFricData()
// {
//   MotorIdx motor_idx[2] = {
//       kMotorIdxFricLeft,
//       kMotorIdxFricRight,
//   };
//   uint8_t tx_data[8] = {0};
//   unsigned int len = 8;
//   for (size_t i = 0; i < 2; i++) {
//     motor_ptr_[motor_idx[i]]->encode(tx_data, &len);
//   }
//   SendCanData(&hcan2, motor_ptr_[motor_idx[0]]->txId(), tx_data);
// }

void MainFsm::sendGimbalChassisCommData()
{
  uint8_t tx_data[8] = {0};
  gc_comm_ptr_->encode(tx_data, gc_comm_ptr_->getTxMsgId());
  SendCanData(&hcan1, gc_comm_ptr_->getTxMsgId(), tx_data);
};

// CHANGE 增加云台和发射机构通信数据发送函数
void MainFsm::sendGimbalShooterCommData()
{
  uint8_t tx_data[8] = {0};
  gs_comm_ptr_->encodeG2S(tx_data);
  SendCanData(&hcan1, gs_comm_ptr_->getTxMsgId(), tx_data);
};

void MainFsm::sendUsartData()
{
  if (work_tick_ % 5 == 0) sendVisionData();
};

void MainFsm::sendVisionData()
{
  size_t len = 0;
  if (vision_ptr_->encode(vision_tx_data_, len)) {
    HAL_UART_Transmit(&huart6, vision_tx_data_, len, 0x1);
  }
};

#pragma endregion

#pragma region register functions

void MainFsm::registerGimbalFsm(GimbalFsm *ptr)
{
  HW_ASSERT(ptr != nullptr, "Gimbal FSM pointer is null", ptr);
  gimbal_fsm_ptr_ = ptr;
};
void MainFsm::registerMiniGimbalFsm(MiniGimbalFsm *ptr)
{
  HW_ASSERT(ptr != nullptr, "MiniGimbal FSM pointer is null", ptr);
  mini_gimbal_fsm_ptr_ = ptr;
};
void MainFsm::registerShooterFsm(ShooterFsm *ptr)
{
  HW_ASSERT(ptr != nullptr, "Shooter FSM pointer is null", ptr);
  shooter_fsm_ptr_ = ptr;
};
void MainFsm::registerImu(Imu *ptr)
{
  HW_ASSERT(ptr != nullptr, "IMU pointer is null", ptr);
  imu_ptr_ = ptr;
};
void MainFsm::registerMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "Motor pointer is null", ptr);
  HW_ASSERT(idx >= 0 && idx < kMaxMotorNum, "Motor index is out of range", idx);
  motor_ptr_[idx] = ptr;
};
void MainFsm::registerGimbalChassisComm(GimbalChassisComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "GimbalChassisComm pointer is null", ptr);
  gc_comm_ptr_ = ptr;
};

// CHANGE 增加云台和发射机构通信指针注册
void MainFsm::registerGimbalShooterComm(GimbalShooterComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "GimbalShooterComm pointer is null", ptr);
  gs_comm_ptr_ = ptr;
};

void MainFsm::registerVison(Vision *ptr)
{
  HW_ASSERT(ptr != nullptr, "Vision pointer is null", ptr);
  vision_ptr_ = ptr;
};
#pragma endregion
/* Private function definitions ----------------------------------------------*/
}  // namespace hero