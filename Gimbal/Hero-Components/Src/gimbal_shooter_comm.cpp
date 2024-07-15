#include "gimbal_shooter_comm.hpp"

#include <cstring>

namespace hero
{
  struct __attribute__((packed)) Gimbal2ShooterPkg{
    uint8_t is_rfr_shooter_power_on;
    uint8_t shoot_signal;
    uint8_t bullet_speed_fdb_uint_8;
    uint8_t bullet_speed_ref_uint_8;
  };

  struct __attribute__((packed)) Shooter2GimbalPkg{
    uint8_t is_shooter_ready;
    uint8_t is_air_bottle_ready;
    uint8_t bullet_feed_note;
  };

  void GimbalShooterComm::decodeS2G(uint8_t rx_data[8])
  {
    Shooter2GimbalPkg pkg = {0};
    std::memcpy(&pkg, rx_data, sizeof(pkg));
    shooter_data_.is_shooter_ready = (bool)pkg.is_shooter_ready;
    shooter_data_.is_air_bottle_ready = (bool)pkg.is_air_bottle_ready;
    shooter_data_.bullet_feed_note = (bool)pkg.bullet_feed_note;
  }

  void GimbalShooterComm::encodeG2S(uint8_t tx_data[8])
  {
    Gimbal2ShooterPkg pkg = {0};
    pkg.is_rfr_shooter_power_on = (uint8_t)shooter_data_.is_rfr_shooter_power_on;
    pkg.shoot_signal = (uint8_t)shooter_data_.shoot_signal;
    pkg.bullet_speed_ref_uint_8 = (uint8_t)shooter_data_.bullet_speed_ref_uint_8;
    pkg.bullet_speed_fdb_uint_8 = (uint8_t)shooter_data_.bullet_speed_fdb_uint_8;
    std::memcpy(tx_data, &pkg, sizeof(pkg));
  }
}