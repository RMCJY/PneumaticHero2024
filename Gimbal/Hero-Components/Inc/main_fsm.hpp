/** 
 *******************************************************************************
 * @file      : main_fsm.hpp
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
#ifndef HEROCOMPONENTS_MAIN_FSM_HPP_
#define HEROCOMPONENTS_MAIN_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "buzzer.hpp"
#include "can.h"
#include "communication_tools.hpp"
#include "gimbal_chassis_comm.hpp"
#include "gimbal_fsm.hpp"
#include "hero_imu.hpp"
#include "mini_gimbal_fsm.hpp"
#include "motor.hpp"
#include "shooter_fsm.hpp"
#include "tick.hpp"
#include "vision.hpp"

namespace hw_fsm = hello_world::fsm;
namespace hw_motor = hello_world::motor;

/* Exported macro ------------------------------------------------------------*/

namespace hero
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class MainFsm : public hello_world::MemMgr
{
 public:
  typedef hw_motor::Motor Motor;
  typedef hero::Imu Imu;
  typedef hero::GimbalFsm GimbalFsm;
  typedef hero::MiniGimbalFsm MiniGimbalFsm;
  typedef hero::ShooterFsm ShooterFsm;

  enum WorkState : uint8_t { kDead, kResurrection, kWorking, kWorkStageNtm };

  /** 控制模式 */
  enum CtrlMode : uint8_t {
    kCtrlManual,
    kCtrlAuto,
  };

  /** 手动控制来源 */
  enum ManualCtrlSrc : uint8_t {
    kManualCtrlByRc,  ///< 遥控器操控
    kManualCtrlByKb,  ///< 键盘操控
  };

  enum MotorIdx : uint8_t {
    // CHANGE 气动没有摩擦轮结构这边注释
    // kMotorIdxFricLeft,
    // kMotorIdxFricRight,
    kMotorIdxYaw,    ///< YAW轴电机下标
    kMotorIdxPitch,  ///< PITCH轴电机下标
    kMotorNum,       ///< 电机数量
  };

  MainFsm(){};
  ~MainFsm(){};

  // 状态机主要接口函数
  void update();
  void run();
  void reset();

  // 注册组件指针
  void registerGimbalFsm(GimbalFsm *ptr);
  void registerMiniGimbalFsm(MiniGimbalFsm *ptr);
  void registerShooterFsm(ShooterFsm *ptr);
  void registerImu(Imu *ptr);
  void registerMotor(Motor *ptr, int idx);
  void registerGimbalChassisComm(GimbalChassisComm *ptr);
  // CHANGE 增加云台和发射机构通信组件指针注册函数
  void registerGimbalShooterComm(GimbalShooterComm *ptr);
  void registerVison(Vision *ptr);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateImu();
  WorkState updateWorkState();

  // 各工作状态任务执行函数
  void runOnDead();
  void runOnResurrection();
  void runOnWorking();

  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();

  // 设置通讯组件数据函数
  void setCommData();
  void setCommDataMotorStandby();
  void setCommDataGimbalChassis();
  // CHANGE 增加云台和发射机构通信数据设置函数
  void setCommDataGimbalShooter();
  void setCommDataVison();

  // 发送通讯组件数据函数
  // CHANGE 气动没有摩擦轮结构这边注释，用3508控制pitch不需要enable和disable
  void EnableDMYaw();
  // void EnableDMPitch();
  void DisableDMYaw();
  // void DisableDMPitch();
  void sendCommData();
  void sendCanData();
  void sendYawData();
  void sendPitchData();
  // void sendFricData();
  void sendGimbalChassisCommData();
  // CHANGE 增加云台和发射机构通信数据发送函数
  void sendGimbalShooterCommData();
  void sendUsartData();
  void sendVisionData();

  uint32_t getCurrentTick() const { return hello_world::tick::GetTickMs(); }

  // 在 update 函数中更新的数据
  WorkState work_state_ = WorkState::kDead;  ///< 工作状态
  uint32_t work_tick_ = 0;                   ///< 记录模块的运行时间，单位为 ms 【reset 无效】

  uint8_t vision_tx_data_[kVisionTxLen] = {0};

  // rc fdb data 在 update 函数中更新

  // IMU 数据在 update 函数中更新
  bool is_imu_caled_offset_ = false;  ///< IMU 数据是否计算完零飘

  // 主要模块状态机组件指针
  GimbalFsm *gimbal_fsm_ptr_ = nullptr;
  MiniGimbalFsm *mini_gimbal_fsm_ptr_ = nullptr;
  ShooterFsm *shooter_fsm_ptr_ = nullptr;

  // 无通信功能的组件指针
  Imu *imu_ptr_ = nullptr;

  // 只接收数据的组件指针
  // 只发送数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;  ///< 云台底盘通信模块指针 发送数据
  // CHANGE 增加云台和发射机构通信模块指针
  GimbalShooterComm *gs_comm_ptr_ = nullptr;  ///< 云台发射机构通信模块指针 发送数据  
  Motor *motor_ptr_[kMotorNum] = {nullptr};   ///< 电机指针数组 发送数据【底盘主控不控制云台关节电机】
  Vision *vision_ptr_ = nullptr;              ///< 视觉通信模块指针
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero
#endif /* HEROCOMPONENTS_MAIN_FSM_HPP_ */
