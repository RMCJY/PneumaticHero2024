/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-06-30 16:03:34
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-03 07:22:54
 * @FilePath: \Shooter\Hero-Shooter-Components\Src\shooter_fam.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "shooter_fsm.hpp"
#include <string.h>

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
};

void ShooterFsm::updateData()
{
  work_tick_ = getCurrentTick();

  updateGimbalBoard();
  updatePillState();
  updateProportionalValve();
};

/**
 * @brief 更新云台板状态
 * 
 * 当云台板通讯丢失时，认为无云台板。此时，发射板始终处于未准备的状态。
 * 如果云台板通讯正常，会检查云台板是否准备就绪。
 */
void ShooterFsm::updateGimbalBoard()
{
  // 当发射板通讯丢失时，认为无发射板
  // 此时，直接认为发射板准备就绪，使得可以在无发射板下工作
  OfflineChecker *oc_ptr = oc_ptr_[kOCIGimbal];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Gimbal offline checker is nullptr", oc_ptr);
  if (oc_ptr->isOffline()) {
    is_gimbal_board_ready_ = false;
  } else {
    HW_ASSERT(gs_comm_ptr_ != nullptr, "pointer to GimbalChassisComm is nullptr", gs_comm_ptr_);
    is_gimbal_board_ready_ = gs_comm_ptr_->isGimbalBoardReady();
  } 
};

/**
 * @brief 更新大弹丸状态
 * 
 * 这个函数会更新光电门的状态，主要探测光电门是否饭测到大弹丸到位
 */
void ShooterFsm::updatePillState()
{
  // 如果光电门被遮挡说明大弹丸到位
  HW_ASSERT(pv_ptr_ != nullptr, "pointer to Proportional Valve offline checker is nullptr", pv_ptr_);
  is_shooter_pill_ready_ = pv_ptr_->IsPhotogateCover();
};

/**
 * @brief 更新比例阀状态
 * 
 * 这个函数会更新比例阀的状态，主要是比例阀的气压反馈
 */
void ShooterFsm::updateProportionalValve()
{
  // 感觉没有必要做离线检验，唯一可能得离线检验的方法是测试ADC读到的数据是不是0
  OfflineChecker *oc_ptr = oc_ptr_[kOCIProportionalValve];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Proportional Valve offline checker is nullptr", oc_ptr);
  HW_ASSERT(pv_ptr_ != nullptr, "pointer to Proportional Valve offline checker is nullptr", pv_ptr_);
  
  // 上述这段为判断比例阀是否离线的代码要放在main.fsm中去判断
  if(oc_ptr->isOffline())
  {
    is_proportional_valve_ready_ = false;
    proportional_valve_airpre_fdb_ = 0;
  }
  else
  {
    is_proportional_valve_ready_  = true;
    proportional_valve_airpre_fdb_ = pv_ptr_->GetProportionalValveVoltage();
  }

};

ShooterFsm::WorkState ShooterFsm::updateWorkState()
{
  // 云台掉线时，切到死亡状态
  if (!is_gimbal_board_ready_) {
    return WorkState::kDead;
  }

  WorkState current_state = work_state_;
  if (current_state == WorkState::kDead) {
    if (is_gimbal_board_ready_) {
      return WorkState::kResurrection;
    }
  } else if (current_state == WorkState::kResurrection) {
    // 复活状态下，如果比例阀准备就绪，则切到工作状态
    // if (is_proportional_valve_ready_) {
    //   return WorkState::kWorking;
    // }
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
  resetDataOnResurrection();
  setCommDataOnResurrection();
};

void ShooterFsm::runOnWorking()
{
  // 在工作状态下，持续保持气室气压在设定值，设定值可由云台版设定，气室气压控制速度。
  getProportionalValveAirpreRef();
  calcProportionalValveInput();
  getAirBottleState();
  setCommDataOnWorking();
};

void ShooterFsm::getProportionalValveAirpreRef()
{
  proportional_valve_airpre_ref_ = (gs_comm_ptr_->shooter_data().proportional_valve_airpre_ref) / 256.0 * 5.0;
}

/**
 * @brief 获取气瓶状态
 * 
 * 这个函数会判断气瓶是否出现欠压状态，并判断当前气压是否达到设定的目标气压值
 */
void ShooterFsm::getAirBottleState()
{
  float thre = 0.01;
  if(fabs(proportional_valve_airpre_fdb_ - proportional_valve_airpre_ref_) < thre)
  {
    is_air_bottle_ready_ = true;
    is_airpre_ready_ = true;
    airpre_duration_ = 0;
  }
  else
  {
    is_airpre_ready_ = false;
    airpre_duration_++;
    if(airpre_duration_ > 500)
    {
      // 气室气压0.5s没有达到设定值，认为气室欠压
      is_air_bottle_ready_ = false;
    }
  }
};

void ShooterFsm::calcProportionalValveInput()
{
  // #TODO这里还需要改，改成预测值+err 
  HW_ASSERT(pid_ptr_[kPidIdxProportionalValve] != nullptr, "pointer to Proportional Valve PID is nullptr", pid_ptr_[kPidIdxProportionalValve]);
  pid_ptr_[kPidIdxProportionalValve]->calc(&proportional_valve_airpre_ref_, &proportional_valve_airpre_fdb_, nullptr, &proportional_valve_airpre_raw_input_);
};

/**
 * @brief 获取气瓶状态
 * 
 * 这个函数控制二位三通阀的状态，当状态为close时候，就是发弹
 */
void ShooterFsm::SetValve23State(Valve23State state)
{
  if(state == Valve23State::kValve23Close)
  {
    HAL_GPIO_WritePin(Valve23_GPIO_Port, Valve23_Pin, GPIO_PIN_RESET);
    is_valve23_close = false;
  }
  else if(state == Valve23State::kValve23Open)
  {
    HAL_GPIO_WritePin(Valve23_GPIO_Port,Valve23_Pin, GPIO_PIN_SET);
    is_valve23_close = true;
  }
};

/**
 * @brief 获取气瓶状态
 * 
 * 这个函数控制二位五通阀的状态，当状态为close时候，就是送弹到待发射的位置
 */
void ShooterFsm::SetValve25State(Valve25State state)
{
  if(state == Valve25State::kValve25Close)
  {
    HAL_GPIO_WritePin(Valve25_GPIO_Port, Valve25_Pin, GPIO_PIN_RESET);
    is_valve25_close = false;
  }
  else if(state == Valve25State::kValve25Open)
  {
    HAL_GPIO_WritePin(Valve25_GPIO_Port,Valve25_Pin, GPIO_PIN_SET);
    is_valve25_close = true;
  }
};

#pragma endregion

#pragma region communicate functions

void ShooterFsm::setCommDataOnDead()
{
  HW_ASSERT(pv_ptr_ != nullptr, "pointer to Proportional Valve offline checker is nullptr", pv_ptr_);
  pv_ptr_->DAC5571SetVoltage(0);
  gs_comm_ptr_->shooter_data().is_shooter_ready = false;
  gs_comm_ptr_->shooter_data().is_shooter_pill_ready = false;
  gs_comm_ptr_->shooter_data().is_airpre_ready = false;
  gs_comm_ptr_->shooter_data().is_air_bottle_ready= false;
  // gs_comm_ptr_->shooter_data().proportional_valve_airpre_ref = 0;
};

void ShooterFsm::setCommDataOnResurrection()
{
  HW_ASSERT(pv_ptr_ != nullptr, "pointer to Proportional Valve offline checker is nullptr", pv_ptr_);
  pv_ptr_->DAC5571SetVoltage(0);
  gs_comm_ptr_->shooter_data().is_shooter_ready = false;
  gs_comm_ptr_->shooter_data().is_shooter_pill_ready = false;
  gs_comm_ptr_->shooter_data().is_airpre_ready = false;
  gs_comm_ptr_->shooter_data().is_air_bottle_ready= false;
  // gs_comm_ptr_->shooter_data().proportional_valve_airpre_ref = 0;
};

void ShooterFsm::setCommDataOnWorking()
{
  OfflineChecker *oc_ptr = oc_ptr_[kOCIProportionalValve];
  HW_ASSERT(oc_ptr != nullptr, "pointer to Proportional Valve offline checker is nullptr", oc_ptr);
  HW_ASSERT(pid_ptr_[kPidIdxProportionalValve] != nullptr, "pointer to Proportional Valve PID is nullptr", pid_ptr_[kPidIdxProportionalValve]);
  HW_ASSERT(pv_ptr_ != nullptr, "pointer to Proportional Valve offline checker is nullptr", pv_ptr_);

  if(oc_ptr->isOffline())
  {
    pid_ptr_[kPidIdxProportionalValve]->reset();
    pv_ptr_->DAC5571SetVoltage(0);
  }
  else
  {
    pv_ptr_->DAC5571SetVoltage(proportional_valve_airpre_raw_input_);
  }

  if(is_airpre_ready_ && gs_comm_ptr_->gimbal_data().shoot_command)
  {
    if(shooter_wait_time == 0)
    {
      if(is_shooter_pill_ready_ && is_valve25_close == Valve25State::kValve25Open)
      {
        SetValve25State(Valve25State::kValve25Close);
        shooter_wait_time++;
        is_shooter_ready_ = false;
      }
    }
    else if(shooter_wait_time <= 500)
    {
      SetValve25State(Valve25State::kValve25Close);
      shooter_wait_time++;
    }
    else if(shooter_wait_time <= 1000)
    {
      if(is_airpre_ready_)
      {
        SetValve23State(Valve23State::kValve23Close);
        shooter_wait_time++;
      }
      else
      {

      }
    }
    else if(shooter_wait_time <= 1500)
    {
      SetValve23State(Valve23State::kValve23Open);
      SetValve25State(Valve25State::kValve25Open);
      shooter_wait_time++;
    }
    else
    {
      shooter_wait_time = 0;
      is_shooter_ready_ = true;
    }
    
  }
  gs_comm_ptr_->shooter_data().is_shooter_ready = is_shooter_ready_;
  gs_comm_ptr_->shooter_data().is_shooter_pill_ready = is_shooter_pill_ready_;
  gs_comm_ptr_->shooter_data().is_airpre_ready = is_airpre_ready_;
  gs_comm_ptr_->shooter_data().is_air_bottle_ready= is_air_bottle_ready_;
  // gs_comm_ptr_->shooter_data().proportional_valve_airpre_ref = proportional_valve_airpre_ref_;  
};
#pragma endregion

#pragma region reset functions

/** 
 * @brief 重置除指针之外的所有数据，使状态机回到初始状态
 */
void ShooterFsm::reset()
{
  work_state_ = WorkState::kDead;
  memset(&proportional_valve_airpre_ref_, 0, sizeof(proportional_valve_airpre_ref_));
  memset(&proportional_valve_airpre_raw_input_, 0, sizeof(proportional_valve_airpre_raw_input_));

  // 在update函数中更新
  is_gimbal_board_ready_ = false;
  is_shooter_ready_ = true;
  is_shooter_pill_ready_ = false;
  is_proportional_valve_ready_ = false;
  is_airpre_ready_ = false;
  is_air_bottle_ready_ = false;
  SetValve23State(Valve23State::kValve23Open);
  SetValve25State(Valve25State::kValve25Open);
  memset(&proportional_valve_airpre_fdb_, 0, sizeof(proportional_valve_airpre_fdb_));

  resetPids();
};

void ShooterFsm::resetDataOnDead()
{
  memset(&proportional_valve_airpre_ref_, 0, sizeof(proportional_valve_airpre_ref_));
  memset(&proportional_valve_airpre_raw_input_, 0, sizeof(proportional_valve_airpre_raw_input_));

  SetValve23State(Valve23State::kValve23Open);
  SetValve25State(Valve25State::kValve25Open);

  resetPids();
};

void ShooterFsm::resetDataOnResurrection()
{
  memset(&proportional_valve_airpre_ref_, 0, sizeof(proportional_valve_airpre_ref_));
  memset(&proportional_valve_airpre_raw_input_, 0, sizeof(proportional_valve_airpre_raw_input_));

  // #TODO 还要修改一下，如果该状态下有弹丸在两阀之间会出问题，所以这边要把2位3通阀先空放一下
  SetValve23State(Valve23State::kValve23Open);
  SetValve25State(Valve25State::kValve25Open);

  resetPids();
}

void ShooterFsm::resetPids()
{
  for (size_t i = 0; i < kPidNum; i++) {
    HW_ASSERT(pid_ptr_[i] != nullptr, "pointer to PID %d is nullptr", i);
    pid_ptr_[i]->reset();
  }
};

#pragma endregion

#pragma region register functions

void ShooterFsm::registerProportionalValve(ProportionalValve *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to Proportional Valve offline checker is nullptr", ptr);
  pv_ptr_ = ptr;
};
void ShooterFsm::registerPid(MultiNodesPid *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to PID %d is nullptr", idx);
  HW_ASSERT(idx >= 0 && idx < kPidNum, "index of PID out of range", idx);
  pid_ptr_[idx] = ptr;
};
void ShooterFsm::registerOfflineChecker(OfflineChecker *ptr, int idx)
{
  HW_ASSERT(ptr != nullptr, "pointer to OfflineChecker is nullptr", ptr);
  HW_ASSERT(idx >= 0 && idx < kOCINum, "index of OfflineChecker out of range", idx);
  oc_ptr_[idx] = ptr;
};
void ShooterFsm::registerGimbalShooterComm(GimbalShooterComm *ptr)
{
  HW_ASSERT(ptr != nullptr, "pointer to GimbalChassisComm is nullptr", ptr);
  gs_comm_ptr_ = ptr;
}

#pragma endregion

/* Private function definitions ----------------------------------------------*/

}