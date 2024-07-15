/** 
 *******************************************************************************
 * @file      : offline_checker.hpp
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
#ifndef HERO_COMPONENTS_OFFLINE_CHECKER_HPP_
#define HERO_COMPONENTS_OFFLINE_CHECKER_HPP_

/* Includes ------------------------------------------------------------------*/
#include <list>

#include "allocator.hpp"
#include "assert.hpp"
#include "tick.hpp"
/* Exported macro ------------------------------------------------------------*/
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
#endif /* HERO_COMPONENTS_OFFLINE_CHECKER_HPP_ */
