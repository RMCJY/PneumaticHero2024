/** 
 *******************************************************************************
 * @file      : mini_gimbal_fsm.hpp
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
#ifndef HEROCOMPONENTS_MINI_GIMBAL_FSM_HPP_
#define HEROCOMPONENTS_MINI_GIMBAL_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "fsm.hpp"
#include "gimbal_chassis_comm.hpp"
#include "offline_checker.hpp"
#include "servo.hpp"

namespace hw_fsm = hello_world::fsm;
namespace hw_servo = hello_world::servo;
/* Exported macro ------------------------------------------------------------*/
namespace hero
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class MiniGimbalFsm : public hello_world::MemMgr
{
 public:
  typedef hw_servo::Servo Servo;
  typedef hero::GimbalChassisComm GimbalChassisComm;
  typedef hello_world::OfflineChecker OfflineChecker;

  /** 工作状态 */
  enum WorkState : uint8_t {
    kDead,
    kResurrection,
    kWorking,
    kWorkStateNum,
  };

  /** 工作模式 */
  enum WorkingMode : uint8_t {
    kGimbalNormalMode,
    kGimbalFarshootMode,
  };

  /** 舵机ID */
  enum ServoIdx : uint8_t {
    kServoIdxMiniPitch,
    kServoIdxScope,
    kServoNum,
  };

  /** 离线检测ID */
  enum OfflineCheckerIdx : uint8_t {
    kOCIChassis,
    kOCINum,
  };


  MiniGimbalFsm(){};
  ~MiniGimbalFsm(){};

  void update();

  void run();

  void reset();

  // 注册组件指针
  void registerServo(Servo *ptr, int idx);
  void registerOfflineChecker(OfflineChecker *ptr, int idx);
  void registerGimbalChassisComm(GimbalChassisComm *ptr);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateChassisBoard();
  WorkState updateWorkState();

  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();

  // 设置通讯组件数据函数
  void setCommDataOnDead();
  void setCommDataOnResurrection();
  void setCommDataOnWorking();

  // 工作状态下，更改工作模式的函数
  void calcMiniGimbalAngle();

  // 工具函数
  uint32_t getCurrentTick() const { return hello_world::tick::GetTickMs(); };
  void setWorkingMode(WorkingMode mode);

  // 在 update 函数中更新的数据
  WorkState work_state_ = WorkState::kDead;  ///< 工作状态
  uint32_t work_tick_ = 0;                   ///< 记录云台模块的运行时间，单位为 ms
  bool is_chassis_board_ready_ = false;  ///< 底盘是否就绪
  bool is_use_scope_ = false;            ///< 是否使用倍镜
  bool mini_gimbal_ctrl_flag_ = false;
  float mini_gimbal_ang_delta_ = 0;

  // 在 runOnWorking 函数中更新的数据
  WorkingMode working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  WorkingMode last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一控制周期的工作模式
  float mini_gimbal_ang_ = 90;      ///< mini pitch角度

  // 各组件指针
  // 无通信功能的组件指针
  OfflineChecker *oc_ptr_[kOCINum] = {nullptr};  ///< 离线检查器指针
  // 只接收数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;  ///< 云台底盘通信器指针 只接收数据
  // 只发送数据的组件指针
  Servo *servo_ptr_[kServoNum] = {nullptr};   ///< 舵机指针 接收、发送数据
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace hero

#endif /* HEROCOMPONENTS_MINI_GIMBAL_FSM_HPP_ */
