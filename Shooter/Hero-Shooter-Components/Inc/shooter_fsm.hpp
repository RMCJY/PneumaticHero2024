/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-06-30 16:03:20
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-02 19:46:49
 * @FilePath: \Shooter\Hero-Shooter-Components\Inc\shooter_fsm.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SHOOTER_FSM_HPP_
#define __SHOOTER_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "offline_checker.hpp"
#include "gimbal_shooter_comm.hpp"
#include "proportional_valve_control.hpp"
#include "fsm.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "gpio.h"

namespace hw_fsm = hello_world::fsm;
namespace hw_pid = hello_world::pid;

namespace hero
{
class ShooterFsm : public hello_world::MemMang
{
public:
    typedef hw_pid::MultiNodesPid MultiNodesPid;
    typedef hero::OfflineChecker OfflineChecker;
    typedef hero::GimbalShooterComm GimbalShooterComm;
    typedef hero::ProportionalValve ProportionalValve;

  /** 工作状态 */
  enum WorkState : uint8_t {
    kDead, 
    kResurrection,
    kWorking, 
    kWorkStateNum, 
  };

  /** PID ID */
  enum PidIdx : uint8_t {
    kPidIdxProportionalValve,
    kPidNum,
  };

  /** 离线检测ID */
  enum OfflineCheckerIdx : uint8_t {
    kOCIProportionalValve, 
    kOCIGimbal,
    kOCINum,
  };

  /** 2位3通阀（用于发射）的启闭状态 */
  enum Valve23State : uint8_t{
    kValve23Open,
    kValve23Close,
  };

  /** 2位5通阀（用于供弹）的启闭状态 */
  enum Valve25State : uint8_t{
    kValve25Open,
    kValve25Close,
  };

  ShooterFsm(){};
  ~ShooterFsm(){};

  void update();

  void run();

  void reset();

    // 注册组件指针
  void registerPid(MultiNodesPid *ptr, int idx);
  void registerOfflineChecker(OfflineChecker *ptr, int idx);
  void registerGimbalShooterComm(GimbalShooterComm *ptr);
  void registerProportionalValve(ProportionalValve *ptr);

private:
    //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateGimbalBoard();
  void updatePillState();
  void updateProportionalValve();
  WorkState updateWorkState();

  void runOnDead();
  void runOnResurrection();
  void runOnWorking();
  
  // 重置数据函数
  void resetDataOnDead();
  void resetDataOnResurrection();
  void resetPids();

  // 设置通讯组件数据函数
  void setCommDataOnDead();
  void setCommDataOnResurrection();
  void setCommDataOnWorking();

  // 工作状态下，获取控制指令的函数
  void getProportionalValveAirpreRef();
  void getAirBottleState();
  void calcProportionalValveInput();

    // 工具函数
  uint32_t getCurrentTick() const { return hello_world::tick::GetTickMs(); };
  void SetValve23State(Valve23State state);
  void SetValve25State(Valve25State state);

  // 在 update 函数中更新的数据
  WorkState work_state_ = WorkState::kDead;  ///< 工作状态
  uint32_t work_tick_ = 0;                ///< 记录比例阀的气压保持时间，单位为 ms
  

  // 在 runOnWorking 函数中更新的数据
  float proportional_valve_airpre_ref_ = 0;  ///< 比例阀气压的参考值，用电压表征，单位为V
  float proportional_valve_airpre_raw_input_ = 0;   ///< 比例阀气压控制原始输入数据 
  
  // 在 update 函数中更新
  // #TODO需要加判断气瓶气压是否达到要求的代码吗？当气瓶气压不够的时候，比例阀将会在一个较低的水平保持不变
  bool is_gimbal_board_ready_ = false;  ///< 云台主控板是否就绪
  bool is_shooter_ready_ = true;       ///< 发射机构是否处于可发射状态，正在发射不处于可发射状态
  bool is_shooter_pill_ready_ = false;   ///< 大弹丸是否到位
  bool is_proportional_valve_ready_ = false;         ///< 比例阀气压是否在线
  bool is_airpre_ready_ = false;        ///< 检测气室气压是否达到目标设定值
  bool is_air_bottle_ready_ = false;    ///< 检查气瓶是否处于欠压状态
  uint32_t airpre_duration_ = 0;        ///< 气压达到设定值的计数器，判断气瓶是否出现欠压
  uint32_t shooter_wait_time = 0;
  bool is_valve23_close = false;
  bool is_valve25_close = false;
  float proportional_valve_airpre_fdb_ = 0;

  // 各组件指针
  // 无通信功能的组件指针
  MultiNodesPid *pid_ptr_[kPidNum] = {nullptr};  ///< PID 指针
  OfflineChecker *oc_ptr_[kOCINum] = {nullptr};  ///< 离线检查器指针
  // 只接收数据的组件指针
  GimbalShooterComm *gs_comm_ptr_ = nullptr;                ///< 云台发射机构通信器指针 只接收数据
  // 接受、发送数据的组件之恒
  ProportionalValve *pv_ptr_ = nullptr;    ///< 比例阀指针 接受、发送数据
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}
#endif /* __SHOOTER_FSM_HPP_ */