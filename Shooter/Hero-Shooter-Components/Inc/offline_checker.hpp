/*
 * @Author: RMCJY 1409947012@qq.com
 * @Date: 2024-05-25 22:07:20
 * @LastEditors: RMCJY 1409947012@qq.com
 * @LastEditTime: 2024-06-30 15:38:13
 * @FilePath: \Shooter\Hero-Shooter-Components\Inc\offline_checker.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HERO_SHOOTER_COMPONENTS_OFFLINE_CHECKER_HPP_
#define HERO_SHOOTER_COMPONENTS_OFFLINE_CHECKER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <list>

#include "allocator.hpp"
#include "assert.hpp"
#include "tick.hpp"

namespace hero
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class OfflineChecker
{
 public:
  OfflineChecker(uint32_t offline_threshold_interval = 1000)
  {
    last_communicate_tick_ = 0;
    offline_threshold_interval_ = offline_threshold_interval;
  }

  bool isOffline() const { return (getCurrentTick() - last_communicate_tick_) > offline_threshold_interval_; }
  bool isOnline() const { return (getCurrentTick() - last_communicate_tick_) <= offline_threshold_interval_; }

  void update() { last_communicate_tick_ = getCurrentTick(); }

 private:
  uint32_t getCurrentTick() const { return hello_world::tick::GetTickMs(); };
  uint32_t last_communicate_tick_ = 0;          ///< 上次通讯时间
  uint32_t offline_threshold_interval_ = 1000;  ///< 离线阈值时间间隔
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero
#endif /* HERO_SHOOTER_COMPONENTS_OFFLINE_CHECKER_HPP_ */