/** 
 *******************************************************************************
 * @file      : main_task.cpp
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
#include "main_task.hpp"

#include "communication_tools.hpp"
#include "ins_all.hpp"
#include "tick.hpp"
#include "tim.h"

namespace hw_tick = hello_world::tick;
namespace hw_motor = hello_world::motor;
namespace hw_laser = hello_world::laser;
namespace hw_filter = hello_world::filter;
namespace hw_servo = hello_world::servo;

using hello_world::PeriodAngle2ContAngleRad;
using hero::GimbalChassisComm;
// CHANGE 增加云台和发射机构通信
using hero::GimbalShooterComm;
using hero::GimbalFsm;
using hero::Imu;
using hero::MainFsm;
using hero::MiniGimbalFsm;
using hero::ShooterFsm;
using hero::Vision;
using hw_filter::Td;
using hw_laser::Laser;
using hw_motor::Motor;
using hw_pid::MultiNodesPid;
using hw_servo::Servo;
using hello_world::OfflineChecker;

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

MainFsm* main_fsm_ptr = nullptr;
GimbalFsm* gimbal_fsm_ptr = nullptr;
MiniGimbalFsm* mini_gimbal_fsm_ptr = nullptr;
ShooterFsm* shooter_fsm_ptr = nullptr;
Imu* imu_ptr = nullptr;
Laser* laser_ptr = nullptr;
Td* td_yaw_ptr = nullptr;
Td* td_pitch_ptr = nullptr;
Servo* mini_pitch_servo = nullptr;
Servo* scope_servo = nullptr;

// rx communication components objects
Motor* motor_yaw_ptr = nullptr;
Motor* motor_pitch_ptr = nullptr;
// CHANGE 气动没有摩擦轮结构这边注释
// Motor* motor_fric_left_ptr = nullptr;
// Motor* motor_fric_right_ptr = nullptr;
GimbalChassisComm* gimbal_chassis_comm_ptr = nullptr;
// CHANGE 增加云台和发射机构通信
GimbalShooterComm* gimbal_shooter_comm_ptr = nullptr;
Vision* vision_ptr = nullptr;

OfflineChecker* oc_yaw_ptr = nullptr;
OfflineChecker* oc_pitch_ptr = nullptr;
// CHANGE 气动没有摩擦轮结构这边注释
// OfflineChecker* oc_fric_left_ptr = nullptr;
// OfflineChecker* oc_fric_right_ptr = nullptr;
OfflineChecker* oc_gimbal_chassis_comm_ptr = nullptr;
// CHANGE 增加云台和发射机构通信的offline checker
OfflineChecker* oc_gimbal_shooter_comm_ptr = nullptr;
OfflineChecker* oc_vision_ptr = nullptr;

// global receive buffer for usart communication with DMA
const size_t rx_vision_buffer_len = 0xff;
uint8_t rx_vision_buffer[rx_vision_buffer_len] = {0};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void PrivatePointerInit(void);
static void MainFsmInit(void);
static void GimbalFsmInit(void);
static void MiniGimbalFsmInit(void);
static void ShooterFsmInit(void);
static void HardWareInit(void);

void MainTaskInit(void)
{
  PrivatePointerInit();
  MainFsmInit();
  GimbalFsmInit();
  MiniGimbalFsmInit();
  ShooterFsmInit();
  HardWareInit();
};
void MainTask(void)
{
  HW_ASSERT(main_fsm_ptr != nullptr, "MainFsm is nullptr", main_fsm_ptr);
  main_fsm_ptr->update();
  main_fsm_ptr->run();

  // if (hw_tick::GetTickMs() % 5 == 0) {
  //   HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_vision_buffer, hero::kVisionRxLen);
  // }
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim6) {
    MainTask();
  }
}

/**
 * @brief   FIFO0接收回调函数
 * @param   none
 * @retval  none
 * @note    根据hcan句柄判断是CAN1还是CAN2
 * @note    根据帧头判断应写入哪个变量
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  CAN_RxHeaderTypeDef rx_header;  // 通信报文帧头
  uint8_t rx_data[8];             // 数据帧

  // CAN1
  if (hcan == &hcan1) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (rx_header.StdId == gimbal_chassis_comm_ptr->getRxMsgId()) {
      HAL_IWDG_Refresh(&hiwdg);
      HW_ASSERT(gimbal_chassis_comm_ptr != nullptr, "gc_comm is nullptr", gimbal_chassis_comm_ptr);
      HW_ASSERT(oc_gimbal_chassis_comm_ptr != nullptr, "OfflineChecker is nullptr", oc_gimbal_chassis_comm_ptr);
      if (gimbal_chassis_comm_ptr->decode(rx_data, rx_header.StdId)) {
        oc_gimbal_chassis_comm_ptr->update();
      }
    }

    HW_ASSERT(motor_yaw_ptr != nullptr, "Motor is nullptr", motor_yaw_ptr);
    HW_ASSERT(oc_yaw_ptr != nullptr, "OfflineChecker is nullptr", oc_yaw_ptr);
    if (motor_yaw_ptr->decode(rx_data, rx_header.StdId) == hw_motor::kStateOk) {
      oc_yaw_ptr->update();
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   FIFO1接收回调函数
 * @param   none
 * @retval  none
 * @note    根据hcan句柄判断是CAN1还是CAN2
 * @note    根据帧头判断应写入哪个变量
**/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  CAN_RxHeaderTypeDef rx_header;  // 通信报文帧头
  uint8_t rx_data[8];             // 数据帧

  // CAN2
  if (hcan == &hcan2) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    HW_ASSERT(motor_pitch_ptr != nullptr, "Motor is nullptr", motor_pitch_ptr);
    HW_ASSERT(oc_pitch_ptr != nullptr, "OfflineChecker is nullptr", oc_pitch_ptr);
    if (motor_pitch_ptr->decode(rx_data, rx_header.StdId) == hw_motor::kStateOk) {
      oc_pitch_ptr->update();
    }

    // CHANGE 气动没有摩擦轮结构这边注释
    // HW_ASSERT(motor_fric_left_ptr != nullptr, "Motor is nullptr", motor_fric_left_ptr);
    // HW_ASSERT(oc_fric_left_ptr != nullptr, "OfflineChecker is nullptr", oc_fric_left_ptr);
    // if (motor_fric_left_ptr->decode(rx_data, rx_header.StdId) == hw_motor::kStateOk) {
    //   oc_fric_left_ptr->update();
    // }

    // HW_ASSERT(motor_fric_right_ptr != nullptr, "Motor is nullptr", motor_fric_right_ptr);
    // HW_ASSERT(oc_fric_right_ptr != nullptr, "OfflineChecker is nullptr", oc_fric_right_ptr);
    // if (motor_fric_right_ptr->decode(rx_data, rx_header.StdId) == hw_motor::kStateOk) {
    //   oc_fric_right_ptr->update();
    // }
    
    // CHANGE 增加发射机构的CAN通信接收
    if (gimbal_shooter_comm_ptr->getRxMsgId() == rx_header.StdId) {
      HW_ASSERT(gimbal_shooter_comm_ptr != nullptr, "gs_comm is nullptr", gimbal_shooter_comm_ptr);
      HW_ASSERT(oc_gimbal_shooter_comm_ptr != nullptr, "OfflineChecker is nullptr", oc_gimbal_shooter_comm_ptr);
      gimbal_shooter_comm_ptr->decodeS2G(rx_data);
      oc_gimbal_shooter_comm_ptr->update(); 
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief   UART接收回调函数
 * @param   none
 * @retval  none
 * @note    none
**/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  // NX
  if (huart == &huart6) {
    bool vision_dec_res = false;
    for (int i = 0; i < Size; i++) {
      vision_dec_res |= vision_ptr->processByte(rx_vision_buffer[i]);
    }
    if (vision_dec_res) {
      oc_vision_ptr->update();
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_vision_buffer, rx_vision_buffer_len);
  }
}

static void PrivatePointerInit(void)
{
  main_fsm_ptr = CreateMainFsm();
  gimbal_fsm_ptr = CreateGimbalFsm();
  mini_gimbal_fsm_ptr = CreateMiniGimbalFsm();
  shooter_fsm_ptr = CreateShooterFsm();
  imu_ptr = CreateImu();
  laser_ptr = CreateLaser();
  td_yaw_ptr = CreateTdYaw();
  td_pitch_ptr = CreateTdPitch();
  mini_pitch_servo = CreateMiniPitchServo();
  scope_servo = CreateScopeServo();

  motor_yaw_ptr = CreateMotorYaw();
  motor_pitch_ptr = CreateMotorPitch();
  // CHANGE 气动没有摩擦轮结构这边注释
  // motor_fric_left_ptr = CreateMotorFricLeft();
  // motor_fric_right_ptr = CreateMotorFricRight();
  gimbal_chassis_comm_ptr = CreateGimbalChassisComm();
  // CHANGE 创建云台和发射机构通信指针
  gimbal_shooter_comm_ptr = CreateGimbalShooterComm();
  vision_ptr = CreateVision();

  oc_yaw_ptr = CreateOcMotorYaw();
  oc_pitch_ptr = CreateOcMotorPitch();
  // CHANGE 气动没有摩擦轮结构这边注释
  // oc_fric_left_ptr = CreateOcMotorFricLeft();
  // oc_fric_right_ptr = CreateOcMotorFricRight();
  oc_gimbal_chassis_comm_ptr = CreateOcChassisControlBoard();
  // CHANGE 创建云台和发射机构通信的offline checker
  oc_gimbal_shooter_comm_ptr = CreateOcShooterControlBoard();
  oc_vision_ptr = CreateOcVision();
};
static void MainFsmInit(void)
{
  HW_ASSERT(main_fsm_ptr != nullptr, "MainFsm is nullptr", main_fsm_ptr);
  // main_fsm 组件指针注册
  // 主要模块状态机组件指针
  main_fsm_ptr->registerGimbalFsm(CreateGimbalFsm());
  main_fsm_ptr->registerMiniGimbalFsm(CreateMiniGimbalFsm());
  main_fsm_ptr->registerShooterFsm(CreateShooterFsm());

  // 无通信功能的组件指针
  main_fsm_ptr->registerImu(CreateImu());

  // 只接收数据的组件指针
  // 只发送数据的组件指针
  main_fsm_ptr->registerGimbalChassisComm(CreateGimbalChassisComm());
  // CHANGE 增加云台和发射机构通信组件指针注册
  main_fsm_ptr->registerGimbalShooterComm(CreateGimbalShooterComm());

  main_fsm_ptr->registerMotor(CreateMotorYaw(), MainFsm::kMotorIdxYaw);
  main_fsm_ptr->registerMotor(CreateMotorPitch(), MainFsm::kMotorIdxPitch);

  // CHANGE 气动没有摩擦轮结构这边注释
  // main_fsm_ptr->registerMotor(CreateMotorFricLeft(), MainFsm::kMotorIdxFricLeft);
  // main_fsm_ptr->registerMotor(CreateMotorFricRight(), MainFsm::kMotorIdxFricRight);

  main_fsm_ptr->registerVison(CreateVision());
};
static void GimbalFsmInit(void)
{
  HW_ASSERT(gimbal_fsm_ptr != nullptr, "GimbalFsm is nullptr", gimbal_fsm_ptr);

  // 各组件指针

  gimbal_fsm_ptr->registerPid(CreatePidMotorYaw(), GimbalFsm::kPidIdxYaw);
  gimbal_fsm_ptr->registerPid(CreatePidMotorPitch(), GimbalFsm::kPidIdxPitch);
  // CHANGE 增加寻找丝杆机械零点时需要使用到的单速度环PID
  gimbal_fsm_ptr->registerPid(CreatePidMotorPitchFind0(), GimbalFsm::kPidIdxPitchFind0);
  
  // 无通信功能的组件指针
  gimbal_fsm_ptr->registerOfflineChecker(CreateOcMotorYaw(), GimbalFsm::kOCIMotorYaw);
  gimbal_fsm_ptr->registerOfflineChecker(CreateOcMotorPitch(), GimbalFsm::kOCIMotorPitch);
  gimbal_fsm_ptr->registerOfflineChecker(CreateOcChassisControlBoard(), GimbalFsm::kOCIChassis);
  gimbal_fsm_ptr->registerOfflineChecker(CreateOcVision(), GimbalFsm::kOCIVision);
  gimbal_fsm_ptr->registerImu(CreateImu());
  gimbal_fsm_ptr->registerLaser(CreateLaser());
  gimbal_fsm_ptr->registerTd(CreateTdYaw(), GimbalFsm::kMotorIdxYaw);
  gimbal_fsm_ptr->registerTd(CreateTdPitch(), GimbalFsm::kMotorIdxPitch);

  // 接收、发送数据的组件指针
  gimbal_fsm_ptr->registerGimbalChassisComm(CreateGimbalChassisComm());

  gimbal_fsm_ptr->registerMotor(CreateMotorYaw(), GimbalFsm::kMotorIdxYaw);
  gimbal_fsm_ptr->registerMotor(CreateMotorPitch(), GimbalFsm::kMotorIdxPitch);

  gimbal_fsm_ptr->registerVision(CreateVision());
};
static void MiniGimbalFsmInit(void)
{
  HW_ASSERT(mini_gimbal_fsm_ptr != nullptr, "MiniGimbalFsm is nullptr", mini_gimbal_fsm_ptr);

  // 各组件指针
  // 无通信功能的组件指针
  mini_gimbal_fsm_ptr->registerOfflineChecker(CreateOcChassisControlBoard(), MiniGimbalFsm::kOCIChassis);

  // 仅发送数据的组件指针
  mini_gimbal_fsm_ptr->registerGimbalChassisComm(CreateGimbalChassisComm());

  mini_gimbal_fsm_ptr->registerServo(CreateMiniPitchServo(), MiniGimbalFsm::kServoIdxMiniPitch);
  mini_gimbal_fsm_ptr->registerServo(CreateScopeServo(), MiniGimbalFsm::kServoIdxScope);
};
static void ShooterFsmInit(void)
{
  HW_ASSERT(shooter_fsm_ptr != nullptr, "ShooterFsm is nullptr", shooter_fsm_ptr);

  // 各组件指针
  // CHANGE 气动没有摩擦轮结构这边注释
  // shooter_fsm_ptr->registerPid(CreatePidMotorFricLeft(), ShooterFsm::kPidIdxFricLeft);
  // shooter_fsm_ptr->registerPid(CreatePidMotorFricRight(), ShooterFsm::kPidIdxFricRight);

  // 无通信功能的组件指针

  // shooter_fsm_ptr->registerOfflineChecker(CreateOcMotorFricLeft(), ShooterFsm::kOCIMotorFricLeft);
  // shooter_fsm_ptr->registerOfflineChecker(CreateOcMotorFricRight(), ShooterFsm::kOCIMotorFricRight);
  // CHANGE 增加云台和发射机构通信的offline checker
  shooter_fsm_ptr->registerOfflineChecker(CreateOcChassisControlBoard(), ShooterFsm::kOCIChassis);
  shooter_fsm_ptr->registerOfflineChecker(CreateOcShooterControlBoard(), ShooterFsm::kOCIShooter);
  shooter_fsm_ptr->registerOfflineChecker(CreateOcVision(), ShooterFsm::kOCIVision);

  // 接收、发送数据的组件指针
  // CHANGE 增加云台和发射机构通信组件指针注册
  shooter_fsm_ptr->registerGimbalChassisComm(CreateGimbalChassisComm());
  shooter_fsm_ptr->registerGimbalShooterComm(CreateGimbalShooterComm());


  // CHANGE 气动没有摩擦轮结构这边注释
  // shooter_fsm_ptr->registerMotor(CreateMotorFricLeft(), ShooterFsm::kMotorIdxFricLeft);
  // shooter_fsm_ptr->registerMotor(CreateMotorFricRight(), ShooterFsm::kMotorIdxFricRight);

  shooter_fsm_ptr->registerVision(CreateVision());
};
static void HardWareInit(void)
{
  // IMU Init
  imu_ptr->initHardware();
  // buzzer init
  // TODO: 等后续组件更新
  // buzzer_ptr->initHardware();
  // CAN init
  InitCanFilter(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  InitCanFilter(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  // NX DMA init
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_vision_buffer, rx_vision_buffer_len);
};