/** 
 *@file       : shooter_fsm.hpp
 *@date       : 2024-03-11
 *@brief      : 
 *@version   1.0.0
 *
 *@copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 *@attention 
 *
 *@par history
 *| Version | Date | Author | Description |
 *| :---: | :---: | :---: | :---: |
 *| 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHOOTER_FSM_HPP_
#define __SHOOTER_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "fsm.hpp"
#include "gimbal_chassis_comm.hpp"
#include "gimbal_shooter_comm.hpp"
#include "hero_imu.hpp"
#include "offline_checker.hpp"
#include "vision.hpp"

namespace hw_fsm = hello_world::fsm;

/* Exported macro ------------------------------------------------------------*/
namespace hero
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class ShooterFsm : public hello_world::MemMgr
{
 public:
  typedef hero::OfflineChecker OfflineChecker;
  typedef hero::GimbalChassisComm GimbalChassisComm;

  /** 工作状态 */
  enum WorkState : uint8_t {
    kDead,
    kResurrection,
    kWorking,
    kWorkStateNum,
  };


  /** 离线检测ID */
  enum OfflineCheckerIdx : uint8_t {
    kOCIShooter,
    kOCIChassis,
    kOCIVision,
    kOCINum,
  };

  ShooterFsm(){};
  ~ShooterFsm(){};

  void update();

  void run();

  void reset();

  // 注册组件指针
  void registerOfflineChecker(OfflineChecker *ptr, int idx);
  void registerGimbalChassisComm(GimbalChassisComm *ptr);
  void registerGimbalShooterComm(GimbalShooterComm *ptr);
  void registerVision(Vision *ptr);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateChassisBoard();
  void updateShooterBoard();
  void updateVision();
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


  // 工具函数
  uint32_t getCurrentTick() const { return hello_world::tick::GetTickMs(); };

  // 在 update 函数中更新的数据
  WorkState work_state_ = WorkState::kDead;  ///< 工作状态
  uint32_t work_tick_ = 0;                   ///< 记录摩擦轮模块的运行时间，单位为 ms

  // 在 update 函数中更新
  bool is_chassis_board_ready_ = false;  ///< 底盘主控板是否就绪
  bool auto_shoot_flag = false;          ///< 自动射击标志位
  bool shoot_signal_ = false;            ///< 发射信号
  bool is_rfr_shooter_power_on_ = false; ///< 发射机构是否上电
  uint8_t bullet_speed_fdb_uint_8 = 0;
  // 从与发射机构主控板的CAN通讯中接收到
  bool is_shooter_board_ready_ = false;  ///< 发射机构主控板是否就绪
  bool is_shooter_ready_ = false;         ///< 发射机构是否就绪，气压异常和正在发射状态时候都为false
  bool is_air_bottle_ready_ = false;      ///< 气瓶是否就绪
  bool bullet_feed_note_ = false;         ///< 是否许需要拨弹，可能存在拨弹不到位的问题

  // 从 vision 组件获取的数据
  bool auto_shoot_flag_ = false;  ///< 自动射击标志位

  // 各组件指针
  // 无通信功能的组件指针
  OfflineChecker *oc_ptr_[kOCINum] = {nullptr};  ///< 离线检查器指针
  // 只接收数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;  ///< 云台底盘通信器指针 只接收数据
  GimbalShooterComm *gs_comm_ptr_ = nullptr;  ///< 云台发射机构通信器指针 只接收数据
  // 接收、发送数据的组件指针
  Vision *vision_ptr_ = nullptr;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace hero

#endif /* __SHOOTER_FSM_HPP_ */
