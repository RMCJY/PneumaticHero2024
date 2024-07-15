/** 
 *******************************************************************************
 * @file      : gimbal_fsm.cpp
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
#include "gimbal_fsm.hpp"
/* Private macro -------------------------------------------------------------*/
namespace hero
{
/* Private constants ---------------------------------------------------------*/

// TODO：pitch轴的最大最小最小角度还需要测试修改
const float kMaxPitchAngle = 0.64;  ///< Pitch轴最大角度，单位 rad

const float kMinPitchAngle = -0.40;  ///< Pitch轴最小角度，单位 rad

const float kMaxSensitivity = 600 / 1000.0 * PI / 180.0;  ///< 云台运动最大灵敏度，rad/ms

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
#pragma region update functions

void GimbalFsm::update()
{
  updateData();
  WorkState state = updateWorkState();
  if (state != work_state_) {
    work_state_ = state;
  }
}

void GimbalFsm::updateData()
{
  work_tick_ = getCurrentTick();

  updateMotor();
  updateIMU();
  updateChassisBoard();
  updateVision();
};

// TODO：3508控制pitch轴，而非达妙，这边的反馈读取还需要修改
void GimbalFsm::updateMotor()
{
  OfflineChecker *oc_ptr = nullptr;
  Motor *motor_ptr = nullptr;

  MotorIdx motor_idxs[2] = {kMotorIdxYaw, kMotorIdxPitch};
  OfflineCheckerIdx oc_idxs[2] = {kOCIMotorYaw, kOCIMotorPitch};

  bool is_any_motor_ready = false;
  bool is_all_motor_ready = true;

  for (size_t i = 0; i < 2; i++) {
    MotorIdx motor_idx = motor_idxs[i];
    OfflineCheckerIdx oc_idx = oc_idxs[i];
    oc_ptr = oc_ptr_[oc_idx];
    motor_ptr = motor_ptr_[motor_idx];
    HW_ASSERT(oc_ptr != nullptr, "pointer %d to offline checker %d is nullptr", oc_ptr, oc_idx);
    HW_ASSERT(motor_ptr != nullptr, "pointer %d to motor %d is nullptr", motor_ptr, motor_idx);
    if (oc_ptr->isOffline()) {
      joint_ang_fdb_motor_[motor_idx] = 0.0;
      joint_spd_fdb_motor_[motor_idx] = 0.0;
      is_all_motor_ready = false;
    } else {
      is_any_motor_ready = true;
      joint_ang_fdb_motor_[motor_idx] = motor_ptr->angle();
      // joint_spd_fdb_motor_[joint_idx] = motor_ptr->vel();
      // 达妙速度反馈噪声大，使用td滤波计算速度
      motor_spd_td_ptr_[motor_idx]->calc(&joint_ang_fdb_motor_[motor_idx], &joint_spd_fdb_motor_[motor_idx]);
    }
  }
  is_any_motor_ready_ = is_any_motor_ready;
  is_all_motor_ready_ = is_all_motor_ready;
};

void GimbalFsm::updateIMU()
{
  HW_ASSERT(imu_ptr_ != nullptr, "pointer %d to imu %d is nullptr", imu_ptr_);
  // TODO(ZSC): 之后需要检查此处的数据映射
  joint_ang_fdb_imu_[JointIdx::kJointIdxYaw] = imu_ptr_->getAngYaw();
  joint_ang_fdb_imu_[JointIdx::kJointIdxPitch] = -imu_ptr_->getAngPitch();
  joint_ang_fdb_imu_[JointIdx::kJointIdxRoll] = imu_ptr_->getAngRoll();

  joint_spd_fdb_imu_[JointIdx::kJointIdxYaw] = imu_ptr_->getGyroYaw();
  joint_spd_fdb_imu_[JointIdx::kJointIdxPitch] = -imu_ptr_->getGyroPitch();
  joint_spd_fdb_imu_[JointIdx::kJointIdxRoll] = imu_ptr_->getGyroRoll();
};
void GimbalFsm::updateVision()
{
  HW_ASSERT(vision_ptr_ != nullptr, "pointer %d to vision is nullptr", vision_ptr_);
  HW_ASSERT(oc_ptr_[kOCIVision] != nullptr, "pointer %d to vision offline checker is nullptr", oc_ptr_[kOCIVision]);
  if (oc_ptr_[kOCIVision]->isOffline()) {
    joint_ang_ref_vision_[0] = joint_ang_fdb_imu_[kMotorIdxYaw];
    joint_ang_ref_vision_[1] = joint_ang_fdb_imu_[kMotorIdxPitch];
  } else {
    joint_ang_ref_vision_[kMotorIdxYaw] = vision_ptr_->getAngRefYaw();
    joint_ang_ref_vision_[kMotorIdxPitch] = vision_ptr_->getAngRefPitch();
  }
};

/**
 * @brief 更新底盘板状态
 * 
 * 当底盘板通讯丢失时，认为无底盘板。此时，云台板始终处于未准备的状态。
 * 如果底盘板通讯正常，会检查底盘板是否准备就绪。
 */
void GimbalFsm::updateChassisBoard()
{
  // 当云台板通讯丢失时，认为无云台板
  // 此时，直接认为云台板准备就绪，使得可以在无云台板下工作
  OfflineChecker *oc_ptr = oc_ptr_[kOCIChassis];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Gimbal offline checker is nullptr", oc_ptr);
  if (oc_ptr->isOffline()) {
    is_chassis_board_ready_ = false;
    joint_ang_delta_.yaw = 0.0;
    joint_ang_delta_.pitch = 0.0;
    mini_gimbal_ctrl_flag_ = false;
    turn_back_flag_ = false;
    setWorkingMode(WorkingMode::kGimbalNormalMode);
    setCtrlMode(CtrlMode::kCtrlManual);
  } else {
    HW_ASSERT(gc_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr", gc_comm_ptr_);
    is_chassis_board_ready_ = gc_comm_ptr_->isChassisBoardReady();
    joint_ang_delta_.yaw = gc_comm_ptr_->gimbal_data().getYawDeltaFloat();
    joint_ang_delta_.pitch = gc_comm_ptr_->gimbal_data().getPitchDeltaFloat();
    mini_gimbal_ctrl_flag_ = gc_comm_ptr_->gimbal_data().mini_gimbal_ctrl_flag;
    turn_back_flag_ = gc_comm_ptr_->gimbal_data().turn_back_flag;
    setWorkingMode((WorkingMode)gc_comm_ptr_->gimbal_data().working_mode);
    setCtrlMode((CtrlMode)gc_comm_ptr_->gimbal_data().ctrl_mode);
  }
};

GimbalFsm::WorkState GimbalFsm::updateWorkState()
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

void GimbalFsm::run()
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
void GimbalFsm::runOnDead()
{
  HW_ASSERT(laser_ptr_ != nullptr, "pointer to laser is nullptr", laser_ptr_);
  laser_ptr_->disable();
  resetDataOnDead();
  setCommDataOnDead();
};

void GimbalFsm::runOnResurrection()
{
  HW_ASSERT(laser_ptr_ != nullptr, "pointer to laser is nullptr", laser_ptr_);
  laser_ptr_->disable();
  resetDataOnResurrection();
  setCommDataOnResurrection();
};

void GimbalFsm::runOnWorking()
{
  HW_ASSERT(laser_ptr_ != nullptr, "pointer to laser is nullptr", laser_ptr_);
  if (ctrl_mode_ == CtrlMode::kCtrlAuto) {
    laser_ptr_->disable();
  } else {
    laser_ptr_->enable();
  }

  calcGimbalAngleRef();
  calcGimbalMotorInput();
  setCommDataOnWorking();
};

void GimbalFsm::calcGimbalAngleRef()
{
  // 判断工作模式是否发生变化
  if (last_working_mode_ != working_mode_) {
    // 如果工作模式发生变化，从反馈值更新上一控制周期的关节角度参考值
    if (working_mode_ == WorkingMode::kGimbalFarshootMode) {
      last_joint_ang_ref_.yaw = joint_ang_fdb_motor_[JointIdx::kJointIdxYaw];
      // CHANGE 3508+丝杆控制pitch轴，电机反馈的角度没有意义
      // last_joint_ang_ref_.pitch = joint_ang_fdb_motor_[JointIdx::kJointIdxPitch];
      last_joint_ang_ref_.pitch = joint_ang_fdb_imu_[JointIdx::kJointIdxPitch];
    } else {
      last_joint_ang_ref_.yaw = joint_ang_fdb_imu_[JointIdx::kJointIdxYaw];
      last_joint_ang_ref_.pitch = joint_ang_fdb_imu_[JointIdx::kJointIdxPitch];
    }
  } else {
    last_joint_ang_ref_ = joint_ang_ref_;
  }

  Cmd tmp_ang_ref = {0.0f};
  // 如果控制模式是自动，且视觉模块没有离线、视觉模块检测到有效目标，且视觉反馈角度与当前角度相差不大
  if (ctrl_mode_ == CtrlMode::kCtrlAuto && oc_ptr_[kOCIVision]->isOnline() && vision_ptr_->isTargetDetected() &&
      abs(last_joint_ang_ref_.yaw - joint_ang_ref_vision_[JointIdx::kJointIdxYaw]) < 0.5f &&
      abs(last_joint_ang_ref_.pitch - joint_ang_ref_vision_[JointIdx::kJointIdxPitch]) < 0.2f) {
    tmp_ang_ref.yaw = joint_ang_ref_vision_[JointIdx::kJointIdxYaw];
    tmp_ang_ref.pitch = joint_ang_ref_vision_[JointIdx::kJointIdxPitch];
    joint_ang_ref_ = setCmdSmoothly(tmp_ang_ref, last_joint_ang_ref_, 1.0);
  } else {
    // {
    // 否则，根据上一控制周期的关节角度参考值计算当前控制周期的关节角度参考值
    float sensitivity = kMaxSensitivity;  // 角度灵敏度，单位 rad/ms
    if (working_mode_ == WorkingMode::kGimbalFarshootMode) {
      sensitivity *= 0.1f;
    } else if (working_mode_ == WorkingMode::kGimbalNormalMode) {
    } else {
      HW_ASSERT(false, "unknown working mode %d", working_mode_);
    }
    if (!mini_gimbal_ctrl_flag_) {
      // mini gimbal控制期间，云台不动
      if (turn_back_flag_ && work_tick_ - last_turn_back_tick_ > 200) {
        tmp_ang_ref.yaw = last_joint_ang_ref_.yaw + PI;
        last_turn_back_tick_ = work_tick_;
      } else {
        tmp_ang_ref.yaw = last_joint_ang_ref_.yaw + joint_ang_delta_.yaw * sensitivity;
      }
      tmp_ang_ref.pitch = last_joint_ang_ref_.pitch + joint_ang_delta_.pitch * sensitivity;
      joint_ang_ref_ = setCmdSmoothly(tmp_ang_ref, last_joint_ang_ref_, 0.8);
    }
  }

  // TODO：这里还需要改，motor的角度反馈已经没有意义了
  if (working_mode_ == WorkingMode::kGimbalFarshootMode) {
    // 吊射模式下，Pitch轴限位
    joint_ang_ref_.pitch = hello_world::Bound(joint_ang_ref_.pitch, kMaxPitchAngle, kMinPitchAngle);
  } else if (working_mode_ == WorkingMode::kGimbalNormalMode) {
    // 正常模式下，Pitch轴限位
    // {fdb}_{imu} - {lim}_{imu} = {fdb}_{motor} - {lim}_{motor}
    // {lim}_{imu} = {lim}_{motor} + {fdb}_{imu} - {fdb}_{motor}

    // CHANGE 气动使用3508+丝杆控制pitch轴，电机反馈角度没有意义。
    // float motor_imu_delta = joint_ang_fdb_imu_[JointIdx::kJointIdxPitch] - joint_ang_fdb_motor_[JointIdx::kJointIdxPitch];
    // joint_ang_ref_.pitch = hello_world::Bound(joint_ang_ref_.pitch, kMaxPitchAngle + motor_imu_delta, kMinPitchAngle + motor_imu_delta);
    joint_ang_ref_.pitch = hello_world::Bound(joint_ang_ref_.pitch, kMaxPitchAngle, kMinPitchAngle);
  } else {
    HW_ASSERT(false, "unknown working mode %d", working_mode_);
  }
};

void GimbalFsm::calcGimbalMotorInput()
{
  // 串级PID计算云台电机期望电流值

  JointIdx joint_idxs[2] = {JointIdx::kJointIdxYaw, JointIdx::kJointIdxPitch};
  if (working_mode_ == WorkingMode::kGimbalFarshootMode) {
    // // 吊射模式下，反馈值来自电机
    // for (size_t i = 0; i < 2; i++) {
    //   JointIdx joint_idx = joint_idxs[i];
    //   joint_ang_fdb_[joint_idx] = joint_ang_fdb_motor_[joint_idx];
    //   joint_vel_fdb_[joint_idx] = joint_spd_fdb_imu_[joint_idx];

    // CHANGE 吊射模式下，pitch反馈值也是来IMU，丝杆有自锁的特性，到达设定值后，发0即可
    JointIdx joint_idx = joint_idxs[1];
    joint_ang_fdb_[joint_idx] = joint_ang_fdb_motor_[joint_idx];
    joint_vel_fdb_[joint_idx] = joint_spd_fdb_imu_[joint_idx];
    
    joint_idx = joint_idxs[2];
    joint_ang_fdb_[joint_idx] = joint_ang_fdb_imu_[joint_idx];
    joint_vel_fdb_[joint_idx] = joint_spd_fdb_imu_[joint_idx];
  } 
  else if (working_mode_ == WorkingMode::kGimbalNormalMode) {
    // 正常模式下，反馈值来自IMU
    for (size_t i = 0; i < 2; i++) {
      JointIdx joint_idx = joint_idxs[i];
      joint_ang_fdb_[joint_idx] = joint_ang_fdb_imu_[joint_idx];
      joint_vel_fdb_[joint_idx] = joint_spd_fdb_imu_[joint_idx];
    }
  } 
  else {
    HW_ASSERT(false, "unknown working mode %d", working_mode_);
  }

  float yaw_fdb[2] = {joint_ang_fdb_[JointIdx::kJointIdxYaw], joint_vel_fdb_[JointIdx::kJointIdxYaw]};
  float pitch_fdb[2] = {joint_ang_fdb_[JointIdx::kJointIdxPitch], joint_vel_fdb_[JointIdx::kJointIdxPitch]};

  float pid_ref[2] = {0, 0};
  HW_ASSERT(pid_ptr_[kPidIdxYaw] != nullptr, "pointer to PID %d is nullptr", kPidIdxYaw);
  MultiNodesPid &pid_yaw = *pid_ptr_[kPidIdxYaw];
  pid_ref[0] = joint_ang_ref_.yaw;
  pid_yaw.calc(pid_ref, yaw_fdb, nullptr, &joint_ref_.yaw);


  HW_ASSERT(pid_ptr_[kPidIdxPitch] != nullptr, "pointer to PID %d is nullptr", kPidIdxPitch);
  MultiNodesPid &pid_pitch = *pid_ptr_[kPidIdxPitch];
  pid_ref[0] = joint_ang_ref_.pitch;
  pid_pitch.calc(pid_ref, pitch_fdb, nullptr, &joint_ref_.pitch);
}

#pragma endregion

#pragma region communicate functions

void GimbalFsm::setCommDataOnDead()
{
  // 机器人死亡时
  // 电机都发送无效数据
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(motor_ptr_[i] != nullptr, "pointer to motor %d is nullptr", i);
    motor_ptr_[i]->setInput(0);
  }
  gc_comm_ptr_->gimbal_data().is_any_motor_ready = false;
  gc_comm_ptr_->gimbal_data().is_all_motor_ready = false;

  // roll, pitch, yaw
  vision_ptr_->setAngle(joint_ang_fdb_imu_[kJointIdxRoll], joint_ang_fdb_imu_[kJointIdxPitch], joint_ang_fdb_imu_[kJointIdxYaw]);
};

void GimbalFsm::setCommDataOnResurrection()
{
  // 机器人复活时
  // 电机都发送无效数据
  for (size_t i = 0; i < 2; i++) {
    HW_ASSERT(motor_ptr_[i] != nullptr, "pointer to motor %d is nullptr", i);
    motor_ptr_[i]->setInput(0);
  }
  gc_comm_ptr_->gimbal_data().is_any_motor_ready = is_any_motor_ready_;
  gc_comm_ptr_->gimbal_data().is_all_motor_ready = is_all_motor_ready_;

  // roll, pitch, yaw
  vision_ptr_->setAngle(joint_ang_fdb_imu_[kJointIdxRoll], joint_ang_fdb_imu_[kJointIdxPitch], joint_ang_fdb_imu_[kJointIdxYaw]);
};

void GimbalFsm::setCommDataOnWorking()
{
  // 机器人工作时
  // 电机根据期望电流输入发送数据
  HW_ASSERT(motor_ptr_[kMotorIdxYaw] != nullptr, "pointer to motor %d is nullptr", kMotorIdxYaw);
  HW_ASSERT(oc_ptr_[kOCIMotorYaw] != nullptr, "pointer to motor %d offline checker is nullptr", kOCIMotorYaw);
  HW_ASSERT(pid_ptr_[kPidIdxYaw] != nullptr, "pointer to PID %d is nullptr", kPidIdxYaw);
  if (oc_ptr_[kOCIMotorYaw]->isOffline()) {
    pid_ptr_[kPidIdxYaw]->reset();
    motor_ptr_[kMotorIdxYaw]->setInput(0);
  } else {
    // motor_ptr_[kMotorIdxYaw]->setInput(0);
    motor_ptr_[kMotorIdxYaw]->setInput(joint_ref_.yaw);
  }

  HW_ASSERT(motor_ptr_[kMotorIdxPitch] != nullptr, "pointer to motor %d is nullptr", kMotorIdxPitch);
  HW_ASSERT(oc_ptr_[kOCIMotorPitch] != nullptr, "pointer to motor %d offline checker is nullptr", kOCIMotorPitch);
  HW_ASSERT(pid_ptr_[kPidIdxPitch] != nullptr, "pointer to PID %d is nullptr", kPidIdxPitch);
  if (oc_ptr_[kOCIMotorPitch]->isOffline()) {
    pid_ptr_[kPidIdxPitch]->reset();
    motor_ptr_[kMotorIdxPitch]->setInput(0);
  } else {
    // motor_ptr_[kMotorIdxPitch]->setInput(0);
    motor_ptr_[kMotorIdxPitch]->setInput(joint_ref_.pitch);
  }

  gc_comm_ptr_->gimbal_data().is_any_motor_ready = is_any_motor_ready_;
  gc_comm_ptr_->gimbal_data().is_all_motor_ready = is_all_motor_ready_;
  gc_comm_ptr_->gimbal_data().pitch_fdb = joint_ang_fdb_[kJointIdxPitch];
  gc_comm_ptr_->gimbal_data().yaw_fdb = joint_ang_fdb_[kJointIdxYaw];
  gc_comm_ptr_->gimbal_data().pitch_ref = joint_ang_ref_.pitch;
  gc_comm_ptr_->gimbal_data().yaw_ref = joint_ang_ref_.yaw;

  if (working_mode_ == WorkingMode::kGimbalFarshootMode) {
    // roll, pitch, yaw
    
    // CHANGE 气动使用3508+丝杆控制pitch轴，电机反馈角度没有意义。
    // TODO：这句话还没明白是什么意思
    // vision_ptr_->setAngle(joint_ang_fdb_imu_[kJointIdxRoll], joint_ang_fdb_motor_[kMotorIdxPitch], joint_ang_fdb_motor_[kMotorIdxYaw]);
    vision_ptr_->setAngle(joint_ang_fdb_imu_[kJointIdxRoll], joint_ang_fdb_imu_[kMotorIdxPitch], joint_ang_fdb_motor_[kMotorIdxYaw]);

    // } else if (working_mode_ == WorkingMode::kGimbalNormalMode) {
  } else {
    // roll, pitch, yaw
    vision_ptr_->setAngle(joint_ang_fdb_imu_[kJointIdxRoll], joint_ang_fdb_imu_[kJointIdxPitch], joint_ang_fdb_imu_[kJointIdxYaw]);
  }
};

#pragma endregion

#pragma region reset functions

/** 
 * @brief 重置除指针之外的所有数据，使状态机回到初始状态
 */
void GimbalFsm::reset()
{
  work_state_ = WorkState::kDead;  ///< 工作状态

  // work_tick_ = 0;                   ///< 记录云台模块的运行时间，单位为 ms

  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::kCtrlManual;                   ///< 控制模式
  working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一次工作模式

  memset(&joint_ang_ref_, 0, sizeof(joint_ang_ref_));            ///< 控制指令，基于关节空间
  memset(&last_joint_ang_ref_, 0, sizeof(last_joint_ang_ref_));  ///< 上一次控制指令，基于关节空间
  memset(&joint_ref_, 0, sizeof(joint_ref_));          ///< 控制指令，基于关节力矩

  // 从底盘拿到的数据
  is_chassis_board_ready_ = false;  ///< 底盘是否就绪
  mini_gimbal_ctrl_flag_ = false;
  memset(&joint_ang_delta_, 0, sizeof(joint_ang_delta_));  ///< 云台关节角度增量

  // 从电机中拿的数据
  is_any_motor_ready_ = false;                                    ///< 任意电机是否处于就绪状态
  is_all_motor_ready_ = false;                                    ///< 所有电机是否都处于就绪状态
  memset(joint_ang_fdb_motor_, 0, sizeof(joint_ang_fdb_motor_));  ///< 云台关节角度反馈值【电机】
  memset(joint_spd_fdb_motor_, 0, sizeof(joint_spd_fdb_motor_));  ///< 云台关节速度反馈值【电机】

  // 从IMU中拿的数据
  memset(joint_ang_fdb_imu_, 0, sizeof(joint_ang_fdb_imu_));  ///< 云台关节角度反馈值【IMU】
  memset(joint_spd_fdb_imu_, 0, sizeof(joint_spd_fdb_imu_));  ///< 云台关节速度反馈值【IMU】

  // 从视觉通讯组件中拿的数据
  memset(joint_ang_ref_vision_, 0, sizeof(joint_ang_ref_vision_));  ///< 云台关节角度参考值【视觉】

  resetPids();
};

void GimbalFsm::resetDataOnDead()
{
  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::kCtrlManual;                   ///< 控制模式
  working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一次工作模式

  memset(&joint_ang_ref_, 0, sizeof(joint_ang_ref_));            ///< 控制指令，基于关节空间
  memset(&last_joint_ang_ref_, 0, sizeof(last_joint_ang_ref_));  ///< 上一次控制指令，基于关节空间
  memset(&joint_ref_, 0, sizeof(joint_ref_));          ///< 控制指令，基于关节力矩

  // 从底盘拿到的数据
  // is_chassis_board_ready_ = false;                        ///< 底盘是否就绪
  mini_gimbal_ctrl_flag_ = false;
  memset(&joint_ang_delta_, 0, sizeof(joint_ang_delta_));  ///< 云台关节角度增量

  // 从电机中拿的数据
  is_any_motor_ready_ = false;  ///< 任意电机是否处于就绪状态
  is_all_motor_ready_ = false;  ///< 所有电机是否都处于就绪状态
  // memset(joint_ang_fdb_motor_, 0, sizeof(joint_ang_fdb_motor_));  ///< 云台关节角度反馈值【电机】
  // memset(joint_spd_fdb_motor_, 0, sizeof(joint_spd_fdb_motor_));  ///< 云台关节速度反馈值【电机】

  // 从IMU中拿的数据
  // memset(joint_ang_fdb_imu_, 0, sizeof(joint_ang_fdb_imu_));  ///< 云台关节角度反馈值【IMU】
  // memset(joint_spd_fdb_imu_, 0, sizeof(joint_spd_fdb_imu_));  ///< 云台关节速度反馈值【IMU】

  // 从视觉通讯组件中拿的数据
  // memset(joint_ang_ref_vision_, 0, sizeof(joint_ang_ref_vision_));  ///< 云台关节角度参考值【视觉】

  // resetPids();
}

void GimbalFsm::resetDataOnResurrection()
{
  // 在 runOnWorking 函数中更新的数据
  ctrl_mode_ = CtrlMode::kCtrlManual;                   ///< 控制模式
  working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一次工作模式

  joint_ang_ref_.yaw = joint_ang_fdb_imu_[kJointIdxYaw];
  joint_ang_ref_.pitch = joint_ang_fdb_imu_[kJointIdxPitch];

  last_joint_ang_ref_.yaw = joint_ang_fdb_imu_[kJointIdxYaw];
  last_joint_ang_ref_.pitch = joint_ang_fdb_imu_[kJointIdxPitch];

  memset(&joint_ref_, 0, sizeof(joint_ref_));  ///< 控制指令，基于关节力矩

  // 从底盘拿到的数据
  // is_chassis_board_ready_ = false;                        ///< 底盘是否就绪
  mini_gimbal_ctrl_flag_ = false;
  memset(&joint_ang_delta_, 0, sizeof(joint_ang_delta_));  ///< 云台关节角度增量

  // 从电机中拿的数据
  // is_any_motor_ready_ = false;  ///< 任意电机是否处于就绪状态
  // is_all_motor_ready_ = false;  ///< 所有电机是否都处于就绪状态
  // memset(joint_ang_fdb_motor_, 0, sizeof(joint_ang_fdb_motor_));  ///< 云台关节角度反馈值【电机】
  // memset(joint_spd_fdb_motor_, 0, sizeof(joint_spd_fdb_motor_));  ///< 云台关节速度反馈值【电机】

  // 从IMU中拿的数据
  // memset(joint_ang_fdb_imu_, 0, sizeof(joint_ang_fdb_imu_));  ///< 云台关节角度反馈值【IMU】
  // memset(joint_spd_fdb_imu_, 0, sizeof(joint_spd_fdb_imu_));  ///< 云台关节速度反馈值【IMU】

  // 从视觉通讯组件中拿的数据
  // memset(joint_ang_ref_vision_, 0, sizeof(joint_ang_ref_vision_));  ///< 云台关节角度参考值【视觉】

  // resetPids();
}

void GimbalFsm::resetPids()
{
  for (size_t i = 0; i < kPidNum; i++) {
    HW_ASSERT(pid_ptr_[i] != nullptr, "pointer to PID %d is nullptr", i);
    pid_ptr_[i]->reset();
  }
};

#pragma endregion

#pragma region tool functions

GimbalFsm::Cmd GimbalFsm::setCmdSmoothly(const Cmd &cmd, const Cmd &last_cmd, float beta)
{
  Cmd cmd_smoothly = {0};
  cmd_smoothly.yaw = last_cmd.yaw + beta * hello_world::PeriodDataSub(cmd.yaw, last_cmd.yaw, 2 * PI);
  cmd_smoothly.pitch = last_cmd.pitch + beta * hello_world::PeriodDataSub(cmd.pitch, last_cmd.pitch, 2 * PI);
  return cmd_smoothly;
};

void GimbalFsm::setWorkingMode(WorkingMode mode)
{
  last_working_mode_ = working_mode_;
  if (working_mode_ != mode) {
    working_mode_ = mode;
  }
};
void GimbalFsm::setCtrlMode(CtrlMode mode) { ctrl_mode_ = mode; };

#pragma endregion

#pragma region register functions

void GimbalFsm::registerMotor(Motor *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to motor %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kMotorNum, "index of motor out of range", idx);
  motor_ptr_[idx] = ptr;
};
void GimbalFsm::registerPid(MultiNodesPid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kPidNum, "index of PID out of range", idx);
  pid_ptr_[idx] = ptr;
};
void GimbalFsm::registerOfflineChecker(OfflineChecker *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to OfflineChecker is nullptr", ptr);
  HW_ASSERT(idx >= 0 && idx < kOCINum, "index of OfflineChecker out of range", idx);
  oc_ptr_[idx] = ptr;
};
void GimbalFsm::registerGimbalChassisComm(GimbalChassisComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gc_comm_ptr_ = ptr;
};
void GimbalFsm::registerImu(Imu *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to imu is nullptr", ptr);
  imu_ptr_ = ptr;
};

void GimbalFsm::registerLaser(Laser *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to laser is nullptr", ptr);
  laser_ptr_ = ptr;
};

void GimbalFsm::registerTd(Td *ptr, size_t idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to Td is nullptr", ptr);
  HW_ASSERT(idx >= 0 && idx < kMotorNum, "index of Td out of range", idx);
  motor_spd_td_ptr_[idx] = ptr;
};

void GimbalFsm::registerVision(Vision *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to Vision is nullptr", ptr);
  vision_ptr_ = ptr;
};

#pragma endregion

/* Private function definitions ----------------------------------------------*/

}  // namespace hero