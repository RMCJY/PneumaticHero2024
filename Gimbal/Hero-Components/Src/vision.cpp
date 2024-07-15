/** 
 *******************************************************************************
 * @file      : vision.cpp
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
#include "vision.hpp"

#include <cmath>
#include <cstring>
#include "tick.hpp"
/* Private macro -------------------------------------------------------------*/
namespace hero
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

/**
 * 将数据编码为可传输的格式。
 * @param data_ptr 指向要编码数据的指针。
 * @param data_len 编码后数据长度的引用。
 * @return 返回true表示编码成功，否则返回false。
 */
bool Vision::encode(uint8_t *data_ptr, size_t &data_len)
{
  // TODO: 根据实际情况改写数据编码逻辑
  // 初始化传输数据缓冲区的前两个字节为固定值
  tx_data_buffer_[0] = 0x3f;  // 固定值，可能用于标识数据包的开始
  tx_data_buffer_[1] = 0x4f;  // 固定值，可能用于标识数据包的类型

  // 将工作状态编码为一个字节
  tx_data_buffer_[2] = (uint8_t)tx_data_.work_state;

  // 将子弹速度乘以100并编码为两个字节
  int16_t temp = 0;
  temp = tx_data_.bullet_speed * 100;
  tx_data_buffer_[3] = temp >> 8;  // 高字节
  tx_data_buffer_[4] = temp;       // 低字节

  // 将横滚角乘以18000/π并编码为两个字节
  temp = tx_data_.roll_angle * 100.0f / M_PI * 180.0f;
  tx_data_buffer_[5] = temp >> 8;  // 高字节
  tx_data_buffer_[6] = temp;       // 低字节

  temp = tx_data_.pitch_angle * 100.0f / M_PI * 180.0f;
  tx_data_buffer_[7] = temp >> 8;  // 高字节
  tx_data_buffer_[8] = temp;       // 低字节

  // 将偏航角乘以18000/π并编码为两个字节
  temp = tx_data_.yaw_angle * 100.0f / M_PI * 180.0f;
  tx_data_buffer_[9] = temp >> 8;  // 高字节
  tx_data_buffer_[10] = temp;      // 低字节

  // 将目标颜色编码为一个字节的低2位
  tx_data_buffer_[11] = ((uint8_t)tx_data_.target_color) & 0b11;

  // 计算校验位，将数据缓冲区的第3个字节到第12个字节异或运算得到校验位
  tx_data_buffer_[12] = tx_data_buffer_[2];
  for (size_t i = 3; i < 12; i++) {
    tx_data_buffer_[12] ^= tx_data_buffer_[i];
  }

  // 拷贝编码后的数据到发送数据缓冲区
  memcpy(data_ptr, tx_data_buffer_, kVisionTxLen);
  // 设置编码后数据的长度
  data_len = kVisionTxLen;

  // 返回true表示编码成功
  return true;
}

/**
 * @brief 对给定的数据进行解码。
 * 
 * @param data_ptr 指向需要解码的数据的指针。
 * @param data_len 需要解码的数据的长度。
 * @return bool 返回解码过程是否成功。如果成功，返回true；否则返回false。
 */
bool Vision::decode(const uint8_t *data_ptr, const size_t &data_len)
{
  bool process_result = false;
  for (size_t i = 0; i < data_len; i++) {
    // 遍历数据，对每个字节进行处理，并更新解码结果
    process_result |= processByte(data_ptr[i]);
  }
  // 返回解码结果
  return process_result;
};

/**
 * @brief 重置解码进度。
 * 
 * 对解码状态进行重置，清空接收数据缓冲区，并重置其索引。
 */
void Vision::resetDecodeProgress(bool keep_rx_buffer)
{
  // 重置解码状态为等待开始帧
  decoding_state_ = DecodingState::kWaitingSof;
  // 使用 0 填充接收数据缓冲区
  if (!keep_rx_buffer) {
    memset(rx_data_buffer_, 0, sizeof(rx_data_buffer_));
  }

  // 重置接收数据缓冲区的当前索引
  rx_data_buffer_idx_ = 0;
};
/** 
 * @brief 重置接收数据
 */
void Vision::resetRxData()
{
  rx_data_.shoot_flag = false;
  rx_data_.target_ids = 0;
  rx_data_.target_num = 0;
  rx_data_.target_color = Color::kGray;
  rx_data_.pitch_angle = 0.0f;
  rx_data_.yaw_angle = 0.0f;
};

/** 
 * @brief 重置发送数据
 */
void Vision::resetTxData()
{
  tx_data_.work_state = WorkState::kVwsNotWorking;
  tx_data_.target_color = Color::kGray;
  tx_data_.roll_angle = 0.0f;
  tx_data_.pitch_angle = 0.0f;
  tx_data_.yaw_angle = 0.0f;
  tx_data_.bullet_speed = 15.5f;
};

/**
 * @brief 处理一个字节的数据。
 * 
 * 该函数根据当前的解码状态(decoding_state_)来解码输入的字节。解码的过程包括接收起始位，接收头部，以及接收固定长度的数据帧。
 * 数据帧接收完毕后，进行校验，并解析其中的数据信息，包含角度、目标ID、颜色等信息。
 * 在解码过程中，任何步骤出错（包括校验失败和数据帧格式不符合预期）都会重置解码进度，并返回false表示处理失败。
 * 如果字节成功解码，函数将返回true。
 * 该函数只处理一个字节的数据，所以会被连续地调用来处理多个字节的数据。
 * 
 * @param byte 需要处理的字节。
 * @return bool 返回处理的结果。如果成功，返回true；否则返回false。
 */
bool Vision::processByte(uint8_t byte)
{
  // TODO: 之后需要检查此处的状态机是否有误
  if (decoding_state_ == DecodingState::kWaitingSof) {
    // 等待起始位
    if (byte == 0xAA) {
      rx_data_buffer_[rx_data_buffer_idx_++] = 0xAA;
      decoding_state_ = DecodingState::kReceivingHeader;
    } else {
      resetDecodeProgress();
      return false;
    }
  } else if (decoding_state_ == DecodingState::kReceivingHeader) {
    // 接收帧头
    if (byte == 0xBB && rx_data_buffer_idx_ == 1) {
      rx_data_buffer_[rx_data_buffer_idx_++] = 0xBB;
      decoding_state_ = DecodingState::kReceivingHeader;
    } else if (byte == 0xCC && rx_data_buffer_idx_ == 2) {
      rx_data_buffer_[rx_data_buffer_idx_++] = 0xCC;
      decoding_state_ = DecodingState::kReceivingTotalFrame;
    } else {
      resetDecodeProgress();
      return false;
    }
  } else if (decoding_state_ == DecodingState::kReceivingTotalFrame) {
    // 接收固定长度的数据
    if (rx_data_buffer_idx_ < kVisionRxLen) {
      rx_data_buffer_[rx_data_buffer_idx_++] = byte;
    }
    if (rx_data_buffer_idx_ == kVisionRxLen) {
      // 接收完所有数据，开始校验
      // 帧尾固定数据位：0xFF
      if (rx_data_buffer_[12] != 0xFF) {
        resetDecodeProgress();
        return false;
      }
      // 校验位：rx_data_buffer_[9]
      uint8_t checksum = rx_data_buffer_[3];
      for (size_t i = 4; i < 11; i++) {
        checksum ^= rx_data_buffer_[i];
      }
      if (checksum != rx_data_buffer_[11]) {
        resetDecodeProgress();
        return false;
      }

      int16_t temp_int16 = 0;
      // 解析数据
      // TODO: 这里需要根据实际情况改写数据编码
      temp_int16 = (int16_t)((rx_data_buffer_[3] << 8) | rx_data_buffer_[4]);
      rx_data_.pitch_angle = ((float)temp_int16) * M_PI / 180.0f / 100.0f;

      temp_int16 = (int16_t)((rx_data_buffer_[5] << 8) | rx_data_buffer_[6]);
      rx_data_.yaw_angle = ((float)temp_int16) * M_PI / 180.0f / 100.0f;

      rx_data_.target_ids = (uint8_t)(rx_data_buffer_[7]);
      rx_data_.target_num = (uint8_t)(rx_data_buffer_[8] >> 4);
      rx_data_.target_color = (Color)((rx_data_buffer_[8]) >> 2 & 0x03);
      rx_data_.shoot_flag = (bool)(rx_data_buffer_[8] & 0x01);
      rx_data_.vtm_x = rx_data_buffer_[9];
      rx_data_.vtm_y = rx_data_buffer_[10];

      last_dec_tick_ = hello_world::tick::GetTickMs();

      resetDecodeProgress(true);
    }
  } else {
    resetDecodeProgress();
    return false;
  }
  return true;
};

/* Private function definitions ----------------------------------------------*/
}  // namespace hero
