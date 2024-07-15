/** 
 *******************************************************************************
 * @file      : gimbal_chassis_comm.hpp
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
#ifndef HERO_COMPONENTS_GIMBAL_CHASSIS_COMM_HPP_
#define HERO_COMPONENTS_GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "allocator.hpp"
#include "base.hpp"
#include "rfr_official_pkgs.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hw_rfr_ids = hello_world::referee::ids;
namespace hero
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class GimbalChassisComm : public hello_world::MemMgr
{
 public:
  typedef hw_rfr_ids::RobotId RobotId;
  typedef hw_rfr_ids::TeamColor TeamColor;

  enum CodePart : uint8_t {
    kCodePartInChassis = 0,
    kCodePartInGimbal = 1,
  };

  /** 工作模式 */
  enum WorkState : uint8_t {
    kWorkStateDead,
    kWorkStateResurrection,
    kWorkStateWorking,
    kWorkStateNum,
  };

  struct MainBoardData {
    // chassis to gimbal
    WorkState chassis_work_state = WorkState::kWorkStateDead;   ///< 底盘主控工作状态
    // gimbal to chassis
    WorkState gimbal_work_state = WorkState::kWorkStateDead;    ///< 云台主控工作状态
    WorkState shooter_work_state = WorkState::kWorkStateDead;   ///< 发射机构主控工作状态
  };

  struct GimbalData {
    // chassis to gimbal
    bool turn_back_flag = false;   ///< 云台快速后转 180 标志
    int8_t yaw_delta_int_8 = 0;    ///< 将归一化的角度增量值转换到-127~127
    int8_t pitch_delta_int_8 = 0;  ///< 将归一化的角度增量值转换到-127~127
    uint8_t work_state = 0;        ///< 云台模块工作状态
    uint8_t ctrl_mode = 0;         ///< 云台模块控制模式（工作状态为 kWorkStateWorking 时有效）
    uint8_t working_mode = 0;      ///< 云台模块的工作模式（工作状态为 kWorkStateWorking 时有效）
    uint8_t manual_ctrl_src = 0;   ///< 云台模块在手动操控模式时的控制数据来源（所有工作状态都会刷新）
    bool use_scope_flag = false;
    bool mini_gimbal_ctrl_flag = false;
    // gimbal to chassis
    bool is_any_motor_ready = false;  ///< 任意一个云台电机上电完成，通讯正常
    bool is_all_motor_ready = false;  ///< 所有云台电机上电完成，通讯正常
    float yaw_fdb = 0.0f;             ///< 云台的当前偏航角度(关节空间)
    float pitch_fdb = 0.0f;           ///< 云台的当前俯仰角度(关节空间)
    float yaw_ref = 0.0f;             ///< 云台的期望偏航角度(关节空间)
    float pitch_ref = 0.0f;           ///< 云台的期望俯仰角度(关节空间)
    float mini_pitch_ = 0.0f;         ///< 小云台的当前俯仰角度(关节空间)

    void setYawDeltaInt8(float delta)
    {
      delta *= 127;
      yaw_delta_int_8 = delta;
      yaw_delta_int_8 = hello_world::Bound(yaw_delta_int_8, -127, 127);
    };
    void setPitchDeltaInt8(float delta)
    {
      delta *= 127;
      pitch_delta_int_8 = delta;
      pitch_delta_int_8 = hello_world::Bound(pitch_delta_int_8, -127, 127);
    };

    float getYawDeltaFloat() const { return static_cast<float>(yaw_delta_int_8) / 127.0f; }
    float getPitchDeltaFloat() const { return static_cast<float>(pitch_delta_int_8) / 127.0f; }
  };

  struct ShooterData {
    // chassis to gimbal
    uint8_t work_state = 0;  ///< 发射机构模块工作状态
    bool shoot_signal_ = false;

    // CHANGE 气动没有摩擦轮结构，在此修改
    // float fric_spd_ref = 0;  ///< 摩擦轮转速
    bool shoot_signal = false;

    // gimbal to chassis

    // CHANGE 气动没有摩擦轮结构，在此修改
    // bool is_fric_ready = false;        ///< 摩擦轮转速达到目标值，摩擦轮工作正常
    // bool is_fric_stuck_ = false;       ///< 摩擦轮是否卡住
    // bool is_any_fric_online_ = false;  ///< 是否有任意摩擦轮在线
    // float fric_spd_fdb = 0.0f;         ///< 摩擦轮的当前转速
    bool is_shooter_ready = false;        ///< 发射机构是否准备就绪
    bool is_air_bottle_ready = false;     ///< 气瓶是否准备就绪
    bool bullet_feed_note = false;             ///< 是否许需要拨弹，可能存在拨弹不到位的问题(由光电门检测)
  };

  struct RefereeData {
    // chassis to gimbal
    uint8_t robot_color = 0;
    uint8_t bullet_speed_uint_8 = 155;
    bool is_rfr_offline = false;
    bool is_rfr_gimbal_power_on = false;
    bool is_rfr_shooter_power_on = false;
    // gimbal to chassis
    void setRobotColor(RobotId id)
    {
      TeamColor color = hw_rfr_ids::GetTeamColor(id);
      if (color == TeamColor::kTeamColorRed) {
        robot_color = 1;
      } else if (color == TeamColor::kTeamColorBlue) {
        robot_color = 2;
      } else {
        robot_color = 0;
      }
    };

    void setBulletSpeedUint8(float bullet_speed)
    {
      if (bullet_speed > 20.0f) {
        bullet_speed = 20.0f;
      }
      bullet_speed_uint_8 = (uint8_t)(bullet_speed * 10);
    };
    float getBulletSpeedFloat() const
    {
      if (bullet_speed_uint_8 > 200) {
        return 20.0f;
      }
      return ((float)bullet_speed_uint_8) / 10.0f;
    }
  };

  struct VisoinData {
    // Gimbal to Chassis
    bool auto_shoot_flag;
    uint8_t detected_targets;
    uint8_t vtm_x;
    uint8_t vtm_y;
  };

  GimbalChassisComm(CodePart code_part, uint32_t chassis_id, uint32_t gimbal_id)
  {
    code_part_ = code_part;
    if (code_part_ == kCodePartInChassis) {
      tx_msg_id_ = chassis_id;
      rx_msg_id_ = gimbal_id;
    } else if (code_part_ == kCodePartInGimbal) {
      tx_msg_id_ = gimbal_id;
      rx_msg_id_ = chassis_id;
    } else {
      tx_msg_id_ = 0;
      rx_msg_id_ = 0;
    }
  };
  virtual ~GimbalChassisComm() = default;

  bool encode(uint8_t tx_data[8], uint32_t tx_msg_std_id)
  {
    if (tx_msg_std_id != tx_msg_id_) {
      return false;
    }
    if (code_part_ == kCodePartInChassis) {
      encodeC2G(tx_data);
    } else if (code_part_ == kCodePartInGimbal) {
      encodeG2C(tx_data);
    } else {
      return false;
    }
    return true;
  };

  bool decode(uint8_t rx_data[8], uint32_t rx_msg_std_id)
  {
    if (rx_msg_std_id != rx_msg_id_) {
      return false;
    }
    if (code_part_ == kCodePartInChassis) {
      decodeG2C(rx_data);
    } else if (code_part_ == kCodePartInGimbal) {
      decodeC2G(rx_data);
    } else {
      return false;
    }
    return true;
  };

  // 直接访问数据
  MainBoardData& main_board_data() { return main_board_data_; }
  GimbalData& gimbal_data() { return gimbal_data_; }
  ShooterData& shooter_data() { return shooter_data_; }
  RefereeData& referee_data() { return referee_data_; }
  VisoinData& vision_data() { return vision_data_; }

  // 实用函数
  bool isGimbalBoardReady() const { return main_board_data_.gimbal_work_state == WorkState::kWorkStateWorking; }
  bool isChassisBoardReady() const { return main_board_data_.chassis_work_state == WorkState::kWorkStateWorking; }
  bool isAnyGimbalMotorReady() const { return gimbal_data_.is_any_motor_ready; }
  bool isAllGimbalMotorReady() const { return gimbal_data_.is_all_motor_ready; }
  // CHAGNE 气动没有摩擦轮结构，在此修改
  // bool isFricReady() const { return shooter_data_.is_fric_ready; }
  // bool isFricStuck() const { return shooter_data_.is_fric_stuck_; }
  // bool isAnyFricOnline() const { return shooter_data_.is_any_fric_online_; }
  bool isShooterReady() const { return shooter_data_.is_shooter_ready; }
  bool isAirBottleReady() const { return shooter_data_.is_air_bottle_ready; }
  bool isBulletNote() const { return shooter_data_.bullet_feed_note; }

  // can cfg
  void setTxMsgId(uint32_t tx_msg_id) { tx_msg_id_ = tx_msg_id; };
  void setRxMsgId(uint32_t rx_msg_id) { rx_msg_id_ = rx_msg_id; };
  uint32_t getTxMsgId() const { return tx_msg_id_; };
  uint32_t getRxMsgId() const { return rx_msg_id_; };

 private:
  void encodeG2C(uint8_t tx_data[8]);
  void decodeG2C(uint8_t rx_data[8]);
  void encodeG2C1(uint8_t tx_data[8]);
  void decodeG2C1(uint8_t rx_data[8]);
  void encodeG2C2(uint8_t tx_data[8]);
  void decodeG2C2(uint8_t rx_data[8]);
  void encodeC2G(uint8_t tx_data[8]);
  void decodeC2G(uint8_t rx_data[8]);

  CodePart code_part_ = kCodePartInChassis;

  MainBoardData main_board_data_;
  GimbalData gimbal_data_;
  ShooterData shooter_data_;
  RefereeData referee_data_;
  VisoinData vision_data_;

  size_t g2c_seq_ = 0;

  //  can cfg
  uint32_t tx_msg_id_ = 0;
  uint32_t rx_msg_id_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero
#endif /* HERO_COMPONENTS_GIMBAL_CHASSIS_COMM_HPP_ */
