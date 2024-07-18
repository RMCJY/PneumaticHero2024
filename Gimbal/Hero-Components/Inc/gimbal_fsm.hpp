/** 
 *@file       : gimbal_fsm.hpp
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
#ifndef __GIMBAL_FSM_HPP_
#define __GIMBAL_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "filter.hpp"
#include "fsm.hpp"
#include "gimbal_chassis_comm.hpp"
#include "hero_imu.hpp"
#include "laser.hpp"
#include "motor.hpp"
#include "offline_checker.hpp"
#include "pid.hpp"
#include "vision.hpp"

namespace hw_fsm = hello_world::fsm;
namespace hw_motor = hello_world::motor;
namespace hw_pid = hello_world::pid;
namespace hw_laser = hello_world::laser;
namespace hw_filter = hello_world::filter;

/* Exported macro ------------------------------------------------------------*/
namespace hero
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
union GimbalCmd {
  struct {
    float yaw;
    float pitch;
  };
  float data[2];

  GimbalCmd operator+(const GimbalCmd &other) const { return {yaw + other.yaw, pitch + other.pitch}; }

  GimbalCmd operator-(const GimbalCmd &other) const { return {yaw - other.yaw, pitch - other.pitch}; }

  GimbalCmd operator*(float scalar) const { return {yaw * scalar, pitch * scalar}; }

  GimbalCmd operator+=(const GimbalCmd &other)
  {
    yaw += other.yaw;
    pitch += other.pitch;
    return *this;
  }

  GimbalCmd operator-=(const GimbalCmd &other)
  {
    yaw -= other.yaw;
    pitch -= other.pitch;
    return *this;
  }

  GimbalCmd operator*=(float scalar)
  {
    yaw *= scalar;
    pitch *= scalar;
    return *this;
  }
  friend GimbalCmd operator*(float scalar, const GimbalCmd &cmd);
};

inline GimbalCmd operator*(float scalar, const GimbalCmd &cmd) { return cmd * scalar; };

class GimbalFsm : public hello_world::MemMgr
{
 public:
  typedef GimbalCmd Cmd;

  typedef hw_motor::Motor Motor;
  typedef hw_laser::Laser Laser;
  typedef hw_pid::MultiNodesPid MultiNodesPid;
  typedef hello_world::OfflineChecker OfflineChecker;
  typedef hero::GimbalChassisComm GimbalChassisComm;
  typedef hw_filter::Td Td;
  typedef hello_world::PeriodAngle2ContAngleRad p2c;

  /** 工作状态 */
  enum WorkState : uint8_t {
    kDead,
    kResurrection,
    kWorking,
    kWorkStateNum,
  };

  /** 工作模式 */
  enum WorkingMode : uint8_t {
    kGimbalNormalMode,    ///< 正常模式，云台角度以IMU为反馈
    kGimbalFarshootMode,  ///< 吊射模式，云台角度以关节角度为反馈
  };

  /** 操纵模式 */
  enum CtrlMode : uint8_t {
    kCtrlManual,
    kCtrlAuto,
  };

  /** 电机ID */
  enum MotorIdx : uint8_t {
    kMotorIdxYaw,
    kMotorIdxPitch,
    kMotorNum,
  };

  /** PID ID */
  enum PidIdx : uint8_t {
    kPidIdxYaw,
    kPidIdxPitch,
    kPidIdxPitchFind0,    ///< 零位设定
    kPidNum,
  };

  enum JointIdx : uint8_t {
    kJointIdxYaw,
    kJointIdxPitch,
    kJointIdxRoll,
    kJointNum,
  };

  /** 离线检测ID */
  enum OfflineCheckerIdx : uint8_t {
    kOCIMotorYaw,
    kOCIMotorPitch,
    kOCIChassis,
    kOCIVision,
    kOCINum,
  };

  GimbalFsm(){};
  ~GimbalFsm(){};

  void update();

  void run();

  void reset();

  // 注册组件指针
  void registerMotor(Motor *ptr, int idx);
  void registerPid(MultiNodesPid *ptr, int idx);
  void registerOfflineChecker(OfflineChecker *ptr, int idx);
  void registerGimbalChassisComm(GimbalChassisComm *ptr);
  void registerImu(Imu *ptr);
  void registerLaser(Laser *ptr);
  void registerTd(Td *ptr, size_t idx);
  void registerVision(Vision *ptr);

 private:
  //  数据更新和工作状态更新，由 update 函数调用
  void updateData();
  void updateMotor();
  void updateIMU();
  void updateVision();
  void updateChassisBoard();
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

  // 工作状态下，更改工作模式的函数
  // void changeWorkingMode();
  void calcGimbalAngleRef();
  void calcGimbalMotorInput();

  // CHANGE 复活模式下寻找机械零点的函数
  void calcPitchFind0MotorInput();
  void getPitchState();

  // 工具函数
  uint32_t getCurrentTick() const { return hello_world::tick::GetTickMs(); };
  Cmd setCmdSmoothly(const Cmd &cmd, const Cmd &last_cmd, float beta = 0.9);
  void setCtrlMode(CtrlMode mode);
  void setWorkingMode(WorkingMode mode);

  // 在 update 函数中更新的数据
  WorkState work_state_ = WorkState::kDead;  ///< 工作状态
  uint32_t work_tick_ = 0;                   ///< 记录云台模块的运行时间，单位为 ms

  // 在 runOnWorking 函数中更新的数据
  CtrlMode ctrl_mode_ = CtrlMode::kCtrlManual;                      ///< 控制模式
  WorkingMode working_mode_ = WorkingMode::kGimbalNormalMode;       ///< 工作模式
  WorkingMode last_working_mode_ = WorkingMode::kGimbalNormalMode;  ///< 上一控制周期的工作模式

  Cmd joint_ang_ref_ = {0};       ///< 关节角度指令，基于关节空间
  float joint_ang_fdb_[2] = {0};  ///< 关节角度反馈，基于关节空间
  float joint_vel_fdb_[2] = {0};  ///< 关节速度反馈，基于关节空间
  Cmd last_joint_ang_ref_ = {0};  ///< 上一次控制指令, 基于关节空间
  Cmd joint_ref_ = {0};           ///< 控制指令，yaw轴基于力矩，pitch轴基于原始报文

  // CHANGE机械零点寻找
  bool is_pitch_0_finded_ = false;
  uint32_t pitch_stuck_duration = 0;
  float pitch_find0_speed_ref = 0;  ///< 丝杆电机上升速度指令
  float pitch_find0_speed_fdb = 0;  ///< 丝杆电机上升速度反馈值
  float pitch_find0_raw_input = 0;  ///< 控制指令，基于原始报文

  // 从底盘拿到的数据
  bool is_chassis_board_ready_ = false;  ///< 底盘是否就绪
  bool mini_gimbal_ctrl_flag_ = false;   ///< 小型云台控制标志位
  bool turn_back_flag_ = false;          ///< 回转 180 标志位
  uint32_t last_turn_back_tick_ = 0;     ///< 上一次回转 180 时间戳
  Cmd joint_ang_delta_ = {0};            ///< 云台关节角度增量

  // 从电机中拿的数据
  bool is_any_motor_ready_ = false;             ///< 任意电机是否处于就绪状态
  bool is_all_motor_ready_ = false;             ///< 所有电机是否都处于就绪状态
  float joint_ang_fdb_motor_[kMotorNum] = {0};  ///< 云台关节角度反馈值【电机】
  float joint_spd_fdb_motor_[kMotorNum] = {0};  ///< 云台关节速度反馈值【电机】

  // 从IMU中拿的数据
  float joint_ang_fdb_imu_[3] = {0};  ///< 云台关节角度反馈值【IMU】
  float joint_spd_fdb_imu_[3] = {0};  ///< 云台关节速度反馈值【IMU】

  // 从视觉通讯组件中拿的数据
  float joint_ang_ref_vision_[2] = {0};  ///< 云台关节角度参考值【视觉】

  // 各组件指针
  // 无通信功能的组件指针
  MultiNodesPid *pid_ptr_[kPidNum] = {nullptr};  ///< PID 指针
  OfflineChecker *oc_ptr_[kOCINum] = {nullptr};  ///< 离线检查器指针
  Laser *laser_ptr_ = nullptr;                   ///< 激光指针
  Td *motor_spd_td_ptr_[kMotorNum] = {nullptr};  ///< 电机速度滤波器指针
  // 只接收数据的组件指针
  GimbalChassisComm *gc_comm_ptr_ = nullptr;  ///< 云台底盘通信器指针 只接收数据
  Imu *imu_ptr_ = nullptr;
  // 接收、发送数据的组件指针
  Motor *motor_ptr_[kMotorNum] = {nullptr};  ///< 电机指针 接收、发送数据
  Vision *vision_ptr_ = nullptr;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace hero

#endif /* __GIMBAL_FSM_HPP_ */
