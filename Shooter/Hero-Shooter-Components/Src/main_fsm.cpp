/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-07-02 19:49:43
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-02 22:18:46
 * @FilePath: \Shooter\Hero-Shooter-Components\Src\main_fsm.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#include "main_fsm.hpp"

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
};
MainFsm::WorkState MainFsm::updateWorkState()
{
  WorkState current_state = work_state_;
  if (current_state == WorkState::kDead) {
    // 主控板程序在跑就意味着有电，所以直接从死亡状态进入复活状态
    return WorkState::kResurrection;
  } else if (current_state == WorkState::kResurrection) {
    return WorkState::kWorking;
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

  HW_ASSERT(shooter_fsm_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_fsm_ptr_);
  shooter_fsm_ptr_->reset();

  setGimbalShooterCommMainBoardData();
  sendCommData();
};
void MainFsm::runOnResurrection()
{
  resetDataOnResurrection();
  HW_ASSERT(shooter_fsm_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_fsm_ptr_);
  shooter_fsm_ptr_->reset();

  setGimbalShooterCommMainBoardData();
  sendCommData();
};
void MainFsm::runOnWorking()
{
  HW_ASSERT(shooter_fsm_ptr_ != nullptr, "Shooter FSM pointer is null", shooter_fsm_ptr_);
  shooter_fsm_ptr_->update();
  shooter_fsm_ptr_->run();

  setGimbalShooterCommMainBoardData();
  sendCommData();
};
#pragma endregion

#pragma region reset functions
void MainFsm::reset() {};
void MainFsm::resetDataOnDead() {};
void MainFsm::resetDataOnResurrection() {};
#pragma endregion

#pragma region communication functions

void MainFsm::setGimbalShooterCommMainBoardData()
{
  gs_comm_ptr_->main_board_data().shooter_work_state = (hero::GimbalShooterComm::WorkState) work_state_;
};

void MainFsm::sendCommData()
{
  sendCanData();
};

void MainFsm::sendCanData()
{
  if(work_tick_ % 10 == 0)
  {
    sendGimbalShooterCommData();
  }
};

void MainFsm::sendGimbalShooterCommData()
{
  uint8_t tx_data[8] = {0};
  gs_comm_ptr_->encode(tx_data,gs_comm_ptr_->getTxMsgId());
  SendCanData(&hcan, gs_comm_ptr_->getTxMsgId(), tx_data);
};

#pragma endregion

#pragma region register functions

void MainFsm::registerShooterFsm(ShooterFsm *ptr)
{
  HW_ASSERT(ptr != nullptr, "Shooter FSM pointer is null", ptr);
  shooter_fsm_ptr_ = ptr;
};
void MainFsm::registerProportionalValve(ProportionalValve *ptr)
{
  HW_ASSERT(ptr != nullptr, "Proportional Valve pointer is null", ptr);
  pv_ptr_ = ptr;
}
void MainFsm::registerGimbalShooterComm(GimbalShooterComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "GimbalShooterComm pointer is null", ptr);
  gs_comm_ptr_ = ptr;
};
#pragma endregion
} // namespace hero
