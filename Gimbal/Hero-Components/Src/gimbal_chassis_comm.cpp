/** 
 *******************************************************************************
 * @file      : gimbal_chassis_comm.cpp
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
#include "gimbal_chassis_comm.hpp"

#include <cstring>
/* Private macro -------------------------------------------------------------*/

namespace hero
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

struct __attribute__((packed)) Chassis2GimbalPkg {
  // gimbal fsm
  int8_t gimbal_yaw_delta;
  int8_t gimbal_pitch_delta;
  uint16_t gimbal_work_state : 2;
  uint16_t gimbal_ctrl_mode : 2;
  uint16_t gimbal_working_mode : 2;
  uint16_t manual_ctrl_src : 2;
  uint16_t gimbal_turn_back_flag : 1;
  uint8_t mini_gimbal_use_scope_flag : 1;
  uint8_t mini_gimbal_ctrl_flag : 1;
  // main fsm
  uint16_t main_board_chassis_work_state : 2;
  // rfr
  uint16_t rfr_robot_color : 2;
  uint16_t rfr_bullet_speed : 8;
  // shooter fsm
  uint16_t shooter_work_state : 2;
  // CHANGE 气动没有摩擦轮结构，摩擦轮目标转速更改为发射信号
  // int16_t shooter_fric_spd_ref;
  uint8_t shoot_signal;
};

static_assert(sizeof(Chassis2GimbalPkg) <= 8, "Chassis2GimbalPkg size error");

struct __attribute__((packed)) Gimbal2ChassisPkg1 {
  uint8_t pkg_type_;
  uint8_t gimbal_is_any_motor_ready : 1;
  uint8_t gimbal_is_all_motor_ready : 1;
  
  // CHANGE 通过gimbal沟通chassis和shooter，需要反馈shooter的状态
  // uint8_t main_board_gimbal_work_state : 2;
  uint8_t main_board_gimbal_work_state : 2;
  uint8_t main_board_shooter_work_state : 2;

  // CHANGE 气动没有摩擦轮结构，在此修改适配气动发射结构
  // uint8_t shooter_is_fric_ready : 1;
  // uint8_t shooter_is_fric_stuck : 1;
  // uint8_t shooter_is_any_fric_online : 1;
  uint8_t shooter_is_shooter_ready : 1;
  uint8_t shooter_is_air_bottle_ready : 1;
  uint8_t shooter_bullet_feed_note : 1;

  // vision
  uint8_t vision_auto_shoot_flag : 1;
  uint8_t vision_detected_targets;
  uint8_t vision_vtm_x;
  uint8_t vision_vtm_y;
};

static_assert(sizeof(Gimbal2ChassisPkg1) <= 8, "Gimbal2ChassisPkg size error");
struct __attribute__((packed)) Gimbal2ChassisPkg2 {
  uint8_t pkg_type_;

  // CHANGE 气动没有摩擦轮结构，在此修改适配气动发射结构
  // 没有必要做比例阀的气压反馈，在shooter主控板中处理即可
  // int8_t shooter_fric_spd_fdb;

  int8_t gimbal_pitch_ref;
  int8_t gimbal_pitch_fdb;
  int8_t gimbal_yaw_ref;
  int8_t gimbal_yaw_fdb;
  int8_t mini_pitch;
};
static_assert(sizeof(Gimbal2ChassisPkg2) <= 8, "Gimbal2ChassisPkg size error");
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
/* Private function definitions ----------------------------------------------*/
void GimbalChassisComm::encodeG2C(uint8_t tx_data[8])
{
  if (g2c_seq_ % 2 == 0) {
    encodeG2C1(tx_data);
  } else {
    encodeG2C2(tx_data);
  }
  g2c_seq_++;
};

void GimbalChassisComm::decodeG2C(uint8_t rx_data[8])
{
  if (rx_data[0] == 1) {
    decodeG2C1(rx_data);
  } else if (rx_data[0] == 2) {
    decodeG2C2(rx_data);
  }
}

void GimbalChassisComm::encodeG2C1(uint8_t tx_data[8])
{
  Gimbal2ChassisPkg1 pkg = {0};
  pkg.pkg_type_ = 1;
  // main board
  pkg.main_board_gimbal_work_state = main_board_data_.gimbal_work_state;
  pkg.main_board_shooter_work_state = main_board_data_.shooter_work_state;

  // gimbal
  pkg.gimbal_is_all_motor_ready = gimbal_data_.is_all_motor_ready;
  pkg.gimbal_is_any_motor_ready = gimbal_data_.is_any_motor_ready;

  // shooter
  // CHANGE 气动没有摩擦轮结构，在此修改适配气动发射结构
  // pkg.shooter_is_fric_ready = shooter_data_.is_fric_ready;
  // pkg.shooter_is_fric_stuck = shooter_data_.is_fric_stuck_;
  // pkg.shooter_is_any_fric_online = shooter_data_.is_any_fric_online_;
  pkg.shooter_is_shooter_ready = shooter_data_.is_shooter_ready;
  pkg.shooter_is_air_bottle_ready = shooter_data_.is_air_bottle_ready;
  pkg.shooter_bullet_feed_note = shooter_data_.bullet_feed_note;

  // vision
  pkg.vision_auto_shoot_flag = vision_data_.auto_shoot_flag;
  pkg.vision_detected_targets = vision_data_.detected_targets;
  pkg.vision_vtm_x = vision_data_.vtm_x;
  pkg.vision_vtm_y = vision_data_.vtm_y;
  memcpy(tx_data, &pkg, sizeof(pkg));
}

void GimbalChassisComm::encodeG2C2(uint8_t tx_data[8])
{
  Gimbal2ChassisPkg2 pkg = {0};
  pkg.pkg_type_ = 2;

  // pkg.shooter_fric_spd_fdb = (int8_t)(shooter_data_.fric_spd_fdb * 125.0f / 800.0f);
  pkg.gimbal_pitch_fdb = (int8_t)(hello_world::AngleNormRad(gimbal_data_.pitch_fdb) / PI * 125.0f);
  pkg.gimbal_pitch_ref = (int8_t)(hello_world::AngleNormRad(gimbal_data_.pitch_ref) / PI * 125.0f);
  pkg.gimbal_yaw_fdb = (int8_t)(hello_world::AngleNormRad(gimbal_data_.yaw_fdb) / PI * 125.0f);
  pkg.gimbal_yaw_ref = (int8_t)(hello_world::AngleNormRad(gimbal_data_.yaw_ref) / PI * 125.0f);
  pkg.mini_pitch = (int8_t)(gimbal_data_.mini_pitch_ / 270 * 125.0f);
  memcpy(tx_data, &pkg, sizeof(pkg));
};
void GimbalChassisComm::decodeG2C1(uint8_t rx_data[8])
{
  Gimbal2ChassisPkg1 pkg = {0};
  memcpy(&pkg, rx_data, sizeof(pkg));
  // main board
  main_board_data_.gimbal_work_state = WorkState(pkg.main_board_gimbal_work_state);
  main_board_data_.shooter_work_state = WorkState(pkg.main_board_shooter_work_state);

  // gimbal
  gimbal_data_.is_all_motor_ready = pkg.gimbal_is_all_motor_ready;
  gimbal_data_.is_any_motor_ready = pkg.gimbal_is_any_motor_ready;

  // shooter
  // CHANGE 气动没有摩擦轮结构，在此修改适配气动发射结构
  // shooter_data_.is_fric_ready = pkg.shooter_is_fric_ready;
  // shooter_data_.is_fric_stuck_ = pkg.shooter_is_fric_stuck;
  // shooter_data_.is_any_fric_online_ = pkg.shooter_is_any_fric_online;
  shooter_data_.is_shooter_ready = pkg.shooter_is_shooter_ready;
  shooter_data_.is_air_bottle_ready = pkg.shooter_is_air_bottle_ready;
  shooter_data_.bullet_feed_note = pkg.shooter_bullet_feed_note;

  // vision
  vision_data_.auto_shoot_flag = pkg.vision_auto_shoot_flag;
  vision_data_.detected_targets = pkg.vision_detected_targets;
  vision_data_.vtm_x = pkg.vision_vtm_x;
  vision_data_.vtm_y = pkg.vision_vtm_y;
};
void GimbalChassisComm::decodeG2C2(uint8_t rx_data[8])
{
  Gimbal2ChassisPkg2 pkg = {0};
  memcpy(&pkg, rx_data, sizeof(pkg));
  // shooter_data_.fric_spd_fdb = pkg.shooter_fric_spd_fdb * 800.0f / 125.0f;
  gimbal_data_.pitch_fdb = pkg.gimbal_pitch_fdb * PI / 125.0f;
  gimbal_data_.pitch_ref = pkg.gimbal_pitch_ref * PI / 125.0f;
  gimbal_data_.yaw_fdb = pkg.gimbal_yaw_fdb * PI / 125.0f;
  gimbal_data_.yaw_ref = pkg.gimbal_yaw_ref * PI / 125.0f;
  gimbal_data_.mini_pitch_ = pkg.mini_pitch * 270 / 125.0f;
};
void GimbalChassisComm::encodeC2G(uint8_t tx_data[8])
{
  Chassis2GimbalPkg pkg = {0};
  // main board
  pkg.main_board_chassis_work_state = main_board_data_.chassis_work_state;
  // gimbal
  pkg.gimbal_yaw_delta = gimbal_data_.yaw_delta_int_8;
  pkg.gimbal_pitch_delta = gimbal_data_.pitch_delta_int_8;
  pkg.gimbal_work_state = gimbal_data_.work_state;
  pkg.gimbal_ctrl_mode = gimbal_data_.ctrl_mode;
  pkg.gimbal_working_mode = gimbal_data_.working_mode;
  pkg.manual_ctrl_src = gimbal_data_.manual_ctrl_src;
  pkg.gimbal_turn_back_flag = gimbal_data_.turn_back_flag;
  pkg.mini_gimbal_use_scope_flag = gimbal_data_.use_scope_flag;
  pkg.mini_gimbal_ctrl_flag = gimbal_data_.mini_gimbal_ctrl_flag;
  // shooter
  // CHANGE 气动没有摩擦轮结构，在此修改适配气动发射结构，主要增添shoot指令的下发
  // pkg.shooter_fric_spd_ref = (int16_t)(shooter_data_.fric_spd_ref * 10.0f);
  pkg.shooter_work_state = (uint8_t)shooter_data_.work_state;
  pkg.shoot_signal = shooter_data_.shoot_signal;

  // rfr
  pkg.rfr_robot_color = referee_data_.robot_color;
  pkg.rfr_bullet_speed = referee_data_.bullet_speed_uint_8;
  memcpy(tx_data, &pkg, sizeof(pkg));
};
void GimbalChassisComm::decodeC2G(uint8_t rx_data[8])
{
  Chassis2GimbalPkg pkg = {0};
  memcpy(&pkg, rx_data, sizeof(pkg));
  // main board
  main_board_data_.chassis_work_state = WorkState(pkg.main_board_chassis_work_state);
  // gimbal
  gimbal_data_.yaw_delta_int_8 = pkg.gimbal_yaw_delta;
  gimbal_data_.pitch_delta_int_8 = pkg.gimbal_pitch_delta;
  gimbal_data_.work_state = pkg.gimbal_work_state;
  gimbal_data_.ctrl_mode = pkg.gimbal_ctrl_mode;
  gimbal_data_.working_mode = pkg.gimbal_working_mode;
  gimbal_data_.manual_ctrl_src = pkg.manual_ctrl_src;
  gimbal_data_.turn_back_flag = pkg.gimbal_turn_back_flag;
  gimbal_data_.use_scope_flag = pkg.mini_gimbal_use_scope_flag;
  gimbal_data_.mini_gimbal_ctrl_flag = pkg.mini_gimbal_ctrl_flag;

  // shooter
  // CHANGE 气动没有摩擦轮结构，在此修改适配气动发射结构，主要增添shoot指令的下发 
  // shooter_data_.fric_spd_ref = ((float)pkg.shooter_fric_spd_ref) / 10.0f;
  shooter_data_.work_state = pkg.shooter_work_state;
  shooter_data_.shoot_signal = pkg.shoot_signal;

  // rfr
  referee_data_.robot_color = pkg.rfr_robot_color;
  referee_data_.bullet_speed_uint_8 = pkg.rfr_bullet_speed;
};

}  // namespace hero