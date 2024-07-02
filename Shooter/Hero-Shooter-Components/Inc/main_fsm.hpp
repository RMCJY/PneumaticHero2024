/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-07-02 19:49:27
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-02 20:04:10
 * @FilePath: \Shooter\Hero-Shooter-Components\Inc\main_fsm.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef HEROSHOOTERCOMPONENTS_MAIN_FSM_HPP_
#define HEROSHOOTERCOMPONENTS_MAIN_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "gimbal_shooter_comm.hpp"
#include "communication_tools.hpp"
#include "proportional_valve_control.hpp"
#include "shooter_fsm.hpp"
#include "tick.hpp"

namespace hw_fsm = hello_world::fsm;

namespace hero
{

class MainFsm : public hello_world::MemMang
{
 public:
  typedef hero::ShooterFsm ShooterFsm;
  typedef hero::ProportionalValve ProportionalValve;

  enum WorkState : uint8_t { kDead, kResurrection, kWorking, kWorkStageNtm };

  MainFsm(){};
  ~MainFsm(){};

  // 状态机主要接口函数
  void update();
  void run();
  void reset();

  // 注册组件指针
  void registerShooterFsm(ShooterFsm *ptr);
  void registerProportionalValve(ProportionalValve *ptr);
  void registerGimbalShooterComm(GimbalShooterComm *ptr);

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
  void setGimbalShooterCommMainBoardData();

  void sendCommData();
  void sendCanData();
  void sendGimbalShooterCommData();

  uint32_t getCurrentTick() const { return hello_world::tick::GetTickMs(); }

  // 在 update 函数中更新的数据
  WorkState work_state_ = WorkState::kDead;  ///< 工作状态
  uint32_t work_tick_ = 0;                   ///< 记录模块的运行时间，单位为 ms 【reset 无效】


  //比例阀ADC是否完成校准
  bool is_adc_caled_offset_ = false;

  // 主要模块状态机组件指针
  ShooterFsm *shooter_fsm_ptr_ = nullptr;
  ProportionalValve *pv_ptr_ = nullptr;

  //只发送数据的指针
  GimbalShooterComm *gs_comm_ptr_ = nullptr;

};

} // namespace hero

#endif