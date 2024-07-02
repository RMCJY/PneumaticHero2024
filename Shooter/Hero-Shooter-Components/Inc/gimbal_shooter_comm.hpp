/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_SHOOTER_COMM_HPP_
#define GIMBAL_SHOOTER_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "allocator.hpp"
#include "base.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace hero
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class GimbalShooterComm : public hello_world::MemMang
{
 public:
  enum CodePart : uint8_t{
    kCodePartInGimbal = 1,
    kCodePartInShooter = 2,
  };

  /** 工作模式 */
  enum WorkState : uint8_t {
    kDead,
    kResurrection,
    kWorking,
    kWorkStateNum,
  };

  struct MainBoardData{
    // gimbal to shooter
    WorkState gimbal_work_state = WorkState::kDead;     ///< 云台主控工作状态
    // shooter to gimbal
    WorkState shooter_work_state = WorkState::kDead;    ///< 发射主控工作状态
  };

  struct GimbalData{
    // gimbal to shooter
    bool shoot_command = false;       ///< 云台主控是否下发发射指令
  };
  
  struct ShooterData{
    // shooter to gimbal
    bool is_shooter_ready = false;      ///< 发射机构是否处于准备状态，发射机构离线或者正在发射时，都不处于准备状态
    // bool is_shooter_stuck = false;      ///< 发射机构是否卡弹，这句话没用，没有枪口处的光电门，无法监测到
    bool is_shooter_pill_ready = false;   ///< 在发射机构并不处于准备状态的情况下，判断弹丸是否到位，判断是否出现供弹的卡弹
    bool is_airpre_ready = false;      ///< 气室气压是否到达目标值，卡弹也有可能是气压不够导致的
    bool is_air_bottle_ready = false;  ///< 气瓶是否处于欠压状态，若欠压则气室气压将无法到达设定这
    uint8_t proportional_valve_airpre_ref = 0;  ///< 比例阀控制气压
  };

  GimbalShooterComm(CodePart code_part, uint32_t gimbal_id, uint32_t shooter_id)
  {
    code_part_ = code_part;
    if(code_part_ == kCodePartInGimbal)
    {
        tx_msg_id_ = gimbal_id;
        rx_msg_id_ = shooter_id;
    }
    else if(code_part_ == kCodePartInShooter)
    {
        tx_msg_id_ = shooter_id;
        rx_msg_id_ = gimbal_id;
    }
    else
    {
        tx_msg_id_ = 0;
        rx_msg_id_ = 0;
    }
  };
  virtual ~GimbalShooterComm() = default;

  bool encode(uint8_t tx_data[8], uint32_t tx_msg_std_id)
  {
    if(tx_msg_std_id != tx_msg_id_)
    {
        return false;
    }
    if(code_part_ == kCodePartInGimbal)
    {
        encodeG2S(tx_data);
    }
    else if(code_part_ == kCodePartInShooter)
    {
        encodeS2G(tx_data);
    }
    else
    {
        return false;
    }
    return true;
  };

  bool decode(uint8_t rx_data[8], uint32_t rx_msg_std_id)
  {
    if(rx_msg_std_id != rx_msg_id_)
    {
        return false;
    }
    if(code_part_ == kCodePartInGimbal)
    {
        decodeS2G(rx_data);
    }
    else if(code_part_ == kCodePartInShooter)
    {
        decodeG2S(rx_data);
    }
    else
    {
        return false;
    }
    return true;
  };

  // 直接访问数据
  MainBoardData& main_board_data() { return main_board_data_; }
  GimbalData& gimbal_data() { return gimbal_data_; }
  ShooterData& shooter_data() { return shooter_data_; }

  // 实用函数
  bool isGimbalBoardReady() const { return main_board_data_.gimbal_work_state == WorkState::kWorking; }
  bool isShooterBoardReady() const { return main_board_data_.shooter_work_state == WorkState::kWorking; }  
  bool isShooterReady() const { return shooter_data_.is_shooter_ready; };
  bool isShooterPillReady() const { return shooter_data_.is_shooter_pill_ready; };
  bool isAirpreReady() const {return shooter_data_.is_airpre_ready; };

  // can cfg
  void setTxMsgId(uint32_t tx_msg_id) { tx_msg_id_ = tx_msg_id; };
  void setRxMsgId(uint32_t rx_msg_id) { rx_msg_id_ = rx_msg_id; };
  uint32_t getTxMsgId() const { return tx_msg_id_; };
  uint32_t getRxMsgId() const { return rx_msg_id_; };

 private:
  void encodeG2S(uint8_t tx_data[8]);
  void decodeG2S(uint8_t rx_data[8]);
  void encodeS2G(uint8_t tx_data[8]);
  void decodeS2G(uint8_t rx_data[8]);

  CodePart code_part_ = kCodePartInShooter;

  MainBoardData main_board_data_;
  GimbalData gimbal_data_;
  ShooterData shooter_data_;

  //  can cfg
  // #TODO还需要设置
  uint32_t tx_msg_id_ = 0;
  uint32_t rx_msg_id_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero
#endif /* GIMBAL_SHOOTER_COMM_HPP_ */