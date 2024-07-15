/** 
 *******************************************************************************
 * @file      : hero_imu.cpp
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
#include "hero_imu.hpp"

#include "spi.h"
/* Private macro -------------------------------------------------------------*/

namespace hero
{
/* Private constants ---------------------------------------------------------*/
const hw_imu::BMI088HWConfig kBmi088DefaultHWConfig = {
    .hspi = &hspi1,
    .acc_cs_port = GPIOA,
    .acc_cs_pin = GPIO_PIN_4,
    .gyro_cs_port = GPIOB,
    .gyro_cs_pin = GPIO_PIN_0,
};

const float kImuRotMatFlatten[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};

const hw_imu::BMI088Config kBmi088DefaultConfig = {
    .acc_range = hw_imu::kBMI088AccRange3G,
    .acc_odr = hw_imu::kBMI088AccOdr1600,
    .acc_osr = hw_imu::kBMI088AccOsr4,
    .gyro_range = hw_imu::kBMI088GyroRange1000Dps,
    .gyro_odr_fbw = hw_imu::kBMI088GyroOdrFbw1000_116,
};
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

bool Imu::initHardware()
{
  if (bmi088_ptr_ == nullptr) {
    status_ = ImuStatus::kPointerUninitErr;
    return false;
  }
  while (bmi088_ptr_->init() != hw_imu::kBMI088ErrStateNoErr) {
    status_ = ImuStatus::kInitHardwareFailedErr;
  }
  status_ = ImuStatus::kWaitCalcOffset;
  return true;
};

bool Imu::update()
{
  if (status_ == ImuStatus::kNotInitHardware) {
    if (!initHardware()) {
      return false;
    }
  }

  if (status_ >= ImuStatus::kInitHardwareFailedErr) {
    return false;
  }

  getRawData();

  if (status_ == ImuStatus::kWaitCalcOffset || status_ == ImuStatus::kCalcingOffset) {
    calcOffset();
  }

  updateAccGyro();

  updateMahony();
  return true;
};

void Imu::getRawData() { bmi088_ptr_->getData(raw_acc_.data, raw_gyro_.data, &temp_); };

void Imu::calcOffset()
{
  if (offset_count_ < offset_max_count_) {
    float threshold = fabs(gyro_threshold_);

    for (size_t i = 0; i < 3; i++) {
      if (raw_gyro_.data[i] > threshold || raw_gyro_.data[i] < -threshold) {
        continue;
      }
      gyro_offset_.data[i] += ((raw_gyro_.data[i] - gyro_offset_.data[i]) / (float)(offset_count_ + 1));
    }
    status_ = ImuStatus::kCalcingOffset;
    offset_count_++;
  } else {
    status_ = ImuStatus::kNormWorking;
  }
};

void Imu::updateAccGyro()
{
  float acc_threshold = fabs(acc_threshold_);
  for (size_t i = 0; i < 3; i++) {
    if (raw_acc_.data[i] > acc_threshold) {
      acc_.data[i] = acc_threshold;
    } else if (raw_acc_.data[i] < -acc_threshold) {
      acc_.data[i] = -acc_threshold;
    } else {
      acc_.data[i] = raw_acc_.data[i];
    }
  }

  for (size_t i = 0; i < 3; i++) {
    gyro_.data[i] = raw_gyro_.data[i] - gyro_offset_.data[i];
  }
};

bool Imu::updateMahony()
{
  if (ahrs_ptr_ == nullptr) {
    status_ = ImuStatus::kPointerUninitErr;
    return false;
  }
  ahrs_ptr_->update(acc_.data, gyro_.data);
  ahrs_ptr_->getEulerAngle(ang_.data);
  return true;
};

/* Private function prototypes -----------------------------------------------*/
}  // namespace hero