/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-25 21:18:01
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-07-03 00:15:30
 * @FilePath: \Shooter\Task\Src\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "main_task.hpp"

#include "communication_tools.hpp"
#include "proportional_valve_control.hpp"
#include "ins_all.hpp"
#include "tim.h"

namespace hw_motor = hello_world::motor;

using hero::ShooterFsm;
using hero::GimbalShooterComm;
using hero::MainFsm;
using hero::ProportionalValve;
using hero::OfflineChecker;
using hw_pid::MultiNodesPid;

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

MainFsm* main_fsm_ptr = nullptr;
ShooterFsm* shooter_fsm_ptr = nullptr;

GimbalShooterComm* gimbal_shooter_comm_ptr = nullptr;

ProportionalValve* pv_ptr = nullptr;

OfflineChecker* oc_gimbal_shooter_comm_ptr = nullptr;

static void PrivatePointerInit(void);
static void MainFsmInit(void);
static void ShooterFsmInit(void);
static void HardWareInit(void);

void MainTaskInit(void)
{
  PrivatePointerInit();
  MainFsmInit();
  ShooterFsmInit();
  HardWareInit();
};

void MainTask(void)
{
  HW_ASSERT(main_fsm_ptr != nullptr, "MainFsm is nullptr", main_fsm_ptr);
  main_fsm_ptr->update();
  main_fsm_ptr->run();
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim4) {
    // 每1ms进入一次中断
    MainTask();
  }
}

/**
 * @brief   FIFO1接收回调函数
 * @param   none
 * @retval  none
 * @note    对stm32f1只有一个CAN
**/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan_rx)
{
  CAN_RxHeaderTypeDef rx_header = {0};  // 通信报文帧头
  uint8_t rx_data[8];             // 数据帧

  if(hcan_rx == &hcan)
  {
    if (HAL_CAN_GetRxMessage(hcan_rx, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)	
    {
      HW_ASSERT(gimbal_shooter_comm_ptr != nullptr, "gs_comm is nullptr", gimbal_shooter_comm_ptr);
      HW_ASSERT(oc_gimbal_shooter_comm_ptr != nullptr, "OfflineChecker is nullptr", oc_gimbal_shooter_comm_ptr);
      if(gimbal_shooter_comm_ptr->decode(rx_data, rx_header.StdId))
      {
        oc_gimbal_shooter_comm_ptr->update();
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan_rx, CAN_IT_RX_FIFO1_MSG_PENDING);
};

static void PrivatePointerInit(void)
{
  main_fsm_ptr = CreateMainFsm();
  shooter_fsm_ptr = CreateShooterFsm();
  pv_ptr = CreateProportionalValve();

  gimbal_shooter_comm_ptr = CraeteGimbalShooterComm();
  oc_gimbal_shooter_comm_ptr = CreateOcGimbalControlBoard();
};

static void MainFsmInit(void)
{
  HW_ASSERT(main_fsm_ptr != nullptr, "MainFsm is nullptr", main_fsm_ptr);
  main_fsm_ptr->registerShooterFsm(CreateShooterFsm());
  main_fsm_ptr->registerProportionalValve(CreateProportionalValve());
  main_fsm_ptr->registerGimbalShooterComm(CraeteGimbalShooterComm());
 
};
static void ShooterFsmInit(void)
{
  HW_ASSERT(shooter_fsm_ptr != nullptr, "Shootersm is nullptr", shooter_fsm_ptr);
  shooter_fsm_ptr->registerPid(CreatePidProportionalValve(), ShooterFsm::kPidIdxProportionalValve);
  shooter_fsm_ptr->registerOfflineChecker(CreatOcProportionalValve(),ShooterFsm::kOCIProportionalValve);
  shooter_fsm_ptr->registerGimbalShooterComm(CraeteGimbalShooterComm());
  shooter_fsm_ptr->registerProportionalValve(CreateProportionalValve());
};

static void HardWareInit(void)
{
  InitCanFilter(&hcan);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
  
  HW_ASSERT(pv_ptr != nullptr, "Proportional Valve is nullptr", pv_ptr);
  pv_ptr->InitVrefintReciprocal();
};