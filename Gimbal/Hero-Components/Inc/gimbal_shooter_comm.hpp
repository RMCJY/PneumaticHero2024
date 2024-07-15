#ifndef HERO_COMPONENTS_GIMBAL_SHOOTER_COMM_HPP_
#define HERO_COMPONENTS_GIMBAL_SHOOTER_COMM_HPP_

#include <cstdint>

#include "allocator.hpp"
#include "base.hpp"

namespace hero
{

class GimbalShooterComm : public hello_world::MemMgr
{
public:
  /** 工作模式 */
  enum WorkState : uint8_t {
    kWorkStateDead,
    kWorkStateResurrection,
    kWorkStateWorking,
    kWorkStateNum,
  };

  struct MainBoardData{
    // gimbal to shooter
    WorkState gimbal_work_state = WorkState::kWorkStateDead;
    // shooter to gimbal
    WorkState shooter_work_state = WorkState::kWorkStateDead;
  };

  struct ShooterData{
    // shooter to gimbal
    bool is_shooter_ready = false;      //< 发射机构是否就绪，气压异常和正在发射状态时候都为false
    bool is_air_bottle_ready = false;   //< 气瓶是否就绪
    bool bullet_feed_note = false;           //< 是否许需要拨弹，可能存在拨弹不到位的问题

    // gimbal to shooter
    bool is_rfr_shooter_power_on = false;
    bool shoot_signal = false;           //< 是否进行发射
    uint8_t bullet_speed_ref_uint_8 = 0;         //< 弹速设定值
    uint8_t bullet_speed_fdb_uint_8 = 0;    //< 裁判系统测得弹速

    void setBulletSpeedFdbUint8(float bullet_speed)
    {
      if (bullet_speed > 20.0f) {
        bullet_speed = 20.0f;
      }
      bullet_speed_fdb_uint_8 = (uint8_t)(bullet_speed * 10);
    };
    float getBulletSpeedFloat() const
    {
      if (bullet_speed_fdb_uint_8 > 200) {
        return 20.0f;
      }
      return ((float)bullet_speed_fdb_uint_8) / 10.0f;
    }
  };

  GimbalShooterComm(){};
  ~GimbalShooterComm(){};

  // 直接访问数据
  MainBoardData& main_board_data() { return main_board_data_; }
  ShooterData& shooter_data() { return shooter_data_; }

  void decodeS2G(uint8_t rx_data[8]);
  void encodeG2S(uint8_t tx_data[8]);

  // 实用函数
  bool isShooterBoardReady() const { return main_board_data_.shooter_work_state; };
  bool isShooterReady() const { return shooter_data_.is_shooter_ready; };     // 带board的是检测通行状态，不带board的是检测发弹状态
  bool isAirBottleReady() const { return shooter_data_.is_air_bottle_ready; };
  bool isBulletFeedNote() const { return shooter_data_.bullet_feed_note; };  
  bool getShootSignal() const { return shooter_data_.shoot_signal; };


  // can cfg
  void setTxMsgId(uint32_t tx_msg_id) { tx_msg_id_ = tx_msg_id; };
  void setRxMsgId(uint32_t rx_msg_id) { rx_msg_id_ = rx_msg_id; };
  uint32_t getTxMsgId() const { return tx_msg_id_; };
  uint32_t getRxMsgId() const { return rx_msg_id_; };

  private:
  MainBoardData main_board_data_;
  ShooterData shooter_data_;

  // can cfg 
  uint32_t tx_msg_id_ = 0;
  uint32_t rx_msg_id_ = 0;
};
}
#endif