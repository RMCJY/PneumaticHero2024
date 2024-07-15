/** 
 *******************************************************************************
 * @file      : vision.hpp
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
#ifndef HERO_COMPONENTS_VISION_HPP_
#define HERO_COMPONENTS_VISION_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "allocator.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace hero
{
/* Exported constants --------------------------------------------------------*/
const size_t kVisionRxLen = 13;  ///< 视觉接收数据长度
const size_t kVisionTxLen = 13;  ///< 视觉发送数据长度
/* Exported types ------------------------------------------------------------*/

class Vision : public hello_world::MemMgr
{
 public:
  enum DecodingState : uint8_t {
    kWaitingSof,
    kReceivingHeader,
    kReceivingTotalFrame,
    kDecodingFrame,
  };

  enum WorkState : uint8_t {
    kVwsNotWorking = 0x00,
    kVwsNoAntiTop = 0x01,
    kVwsAntiTop = 0x02,
    kVwsSmallBuff = 0x03,
    kVwsBigBuff = 0x04,
    kVwsCounterSmallBuff = 0x05,
    kVwsCounterBigBuff = 0x06,
    kVwsFarshoot = 0x07,
    kVwsReboot = 0xDD,
    kVwsDebug = 0xEE,
  };

  enum Color : uint8_t {
    kGray = 0u,    ///< 无灯条装甲板
    kBlue = 1u,    ///< 蓝色装甲板，当己方机器人为红方时自动设置
    kRed = 2u,     ///< 红色装甲板，当己方机器人为蓝方时自动设置
    kPurple = 3u,  ///< 紫色装甲板
  };

  struct TxData {
    // XYZ 固定角的轴：Z 轴 竖直向上，Y 轴水平向右，X 轴水平向前
    WorkState work_state = WorkState::kVwsNotWorking;  ///<  工作状态
    Color target_color = Color::kGray;                 ///<  目标颜色

    float roll_angle = 0.0f;   ///<  发送给视觉的 XYZ 固定角（绕 X 轴旋转角度），单位：度
    float pitch_angle = 0.0f;  ///<  发送给视觉的 XYZ 固定角（绕 Y 轴旋转角度），单位：度
    float yaw_angle = 0.0f;    ///<  发送给视觉的 XYZ 固定角（绕 Z 轴旋转角度），单位：度

    float bullet_speed = 15.5f;  ///<  发送给视觉的子弹速度，单位：m/s
  };

  struct RxData {
    bool shoot_flag;     ///<  自动射击标志位
    uint8_t target_ids;  ///<  视觉识别到的目标 id
    uint8_t target_num;  ///<  视觉识别到的目标数量
    Color target_color;  ///<  视觉识别到的目标颜色

    float pitch_angle;  ///<  接收来自视觉的 XYZ 固定角（绕 Y 轴旋转角度），单位：度
    float yaw_angle;    ///<  接收来自视觉的 XYZ 固定角（绕 Z 轴旋转角度），单位：度
    uint8_t vtm_x;      ///<  接收来自视觉的 VTM 坐标系的 X 轴坐标，单位：像素
    uint8_t vtm_y;      ///<  接收来自视觉的 VTM 坐标系的 Y 轴坐标，单位：像素
  };

  bool encode(uint8_t *data_ptr, size_t &data_len);
  bool processByte(uint8_t byte);
  bool decode(const uint8_t *data_ptr, const size_t &data_len);

  const RxData &getRxData() const { return rx_data_; };
  void resetDecodeProgress(bool keep_rx_buffer = false);
  void resetRxData();
  void resetTxData();

  bool isShootFlagHandled() const { return is_handled_shoot_flag_; };
  void setShootFlagHandled() { is_handled_shoot_flag_ = true; };
  bool getShootFlag() const { return rx_data_.shoot_flag && (is_handled_shoot_flag_ == false); };
  float getAngRefPitch() const { return rx_data_.pitch_angle; };
  float getAngRefYaw() const { return rx_data_.yaw_angle; };

  bool setBulletSpeed(float bullet_speed, float beta = 0.9)
  {
    if (bullet_speed == last_bullet_speed_) {
      return false;
    }
    tx_data_.bullet_speed = (1 - beta) * tx_data_.bullet_speed + beta * bullet_speed;
    last_bullet_speed_ = bullet_speed;
    return true;
  };

  void setWorkState(WorkState work_state) { tx_data_.work_state = work_state; };

  void setTargetColor(Color color) { tx_data_.target_color = color; };

  void setAngle(float roll, float pitch, float yaw)
  {
    tx_data_.roll_angle = roll;
    tx_data_.pitch_angle = pitch;
    tx_data_.yaw_angle = yaw;
  };

  bool isTargetDetected() const { return rx_data_.target_ids != 0; };

 private:
  // 发给视觉的数据
  TxData tx_data_;
  // 视觉发送的数据
  RxData rx_data_;

  // 辅助变量
  bool is_handled_shoot_flag_ = false;          ///<  自动射击标志位是否已处理
  uint8_t rx_data_buffer_idx_ = 0;              ///<  接收来自视觉的数据索引
  uint8_t rx_data_buffer_[kVisionRxLen] = {0};  ///<  接收来自视觉的数据缓存
  uint8_t tx_data_buffer_[kVisionTxLen] = {0};  ///<  发给视觉的数据缓存，debug 用

  DecodingState decoding_state_ = DecodingState::kWaitingSof;  ///<  解码状态

  float last_bullet_speed_ = 0.0f;  ///<  上一次发送的子弹速度，用于判断是否需要更新子弹速度
  uint32_t last_dec_tick_ = 0;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero
#endif /* HERO_COMPONENTS_VISION_HPP_ */
