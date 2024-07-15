/** 
 *******************************************************************************
 * @file      : hero_imu.hpp
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
#ifndef HERO_COMPONENTS_HERO_IMU_HPP_
#define HERO_COMPONENTS_HERO_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"
#include "mahony.hpp"
#include "system.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace hw_imu = hello_world::imu;
namespace hw_ahrs = hello_world::ahrs;
namespace hero
{
/* Exported constants --------------------------------------------------------*/
extern const hw_imu::BMI088HWConfig kBmi088DefaultHWConfig;
extern const float kImuRotMatFlatten[9];
extern const hw_imu::BMI088Config kBmi088DefaultConfig;

/* Exported types ------------------------------------------------------------*/
union Imu3AxisData {
  struct {
    float x;
    float y;
    float z;
  };
  float data[3];
};

struct ImuInitParams {
  size_t offset_max_count = 1000;
  float acc_threshold = 10.0f;
  float gyro_threshold = 0.1f;
  float samp_freq = 1000.0f;
  float kp = 1.0f;
  float ki = 0.0f;
  const hw_imu::BMI088HWConfig *bmi088_hw_config_ptr = &kBmi088DefaultHWConfig;
  const float *rot_mat_ptr = kImuRotMatFlatten;  // TODO 可能需要修改
  const hw_imu::BMI088Config *bmi088_config_ptr = &kBmi088DefaultConfig;
};

class Imu : public hello_world::MemMgr
{
 public:
  enum ImuStatus {
    kNotInitHardware,
    kWaitCalcOffset,
    kCalcingOffset,
    kNormWorking,
    kInitHardwareFailedErr,
    kPointerUninitErr,
    kUnkonwnErr,
  };

  explicit Imu(const ImuInitParams &params = ImuInitParams())
  {
    offset_max_count_ = params.offset_max_count;
    acc_threshold_ = fabs(params.acc_threshold);
    gyro_threshold_ = fabs(params.gyro_threshold);
    bmi088_ptr_ = new hw_imu::BMI088(*params.bmi088_hw_config_ptr, params.rot_mat_ptr, *params.bmi088_config_ptr);
    ahrs_ptr_ = new Mahony(params.samp_freq, params.kp, params.ki);
  };
  ~Imu()
  {
    if (bmi088_ptr_) {
      delete bmi088_ptr_;
      bmi088_ptr_ = nullptr;
    }
    if (ahrs_ptr_) {
      delete ahrs_ptr_;
      ahrs_ptr_ = nullptr;
    }
  };

  bool initHardware();

  bool update();

  void resetOffset()
  {
    status_ = ImuStatus::kWaitCalcOffset;
    memset(gyro_offset_.data, 0, sizeof(gyro_offset_.data));
    offset_count_ = 0;
  };

  bool isNormWorking() { return status_ == ImuStatus::kNormWorking; };

  float getAngRoll() { return ang_.x; };
  float getAngPitch() { return ang_.y; };
  float getAngYaw() { return ang_.z; };

  float getGyroRoll() { return gyro_.x; };
  float getGyroPitch() { return gyro_.y; };
  float getGyroYaw() { return gyro_.z; };

 private:
  typedef hello_world::imu::BMI088 BMI088;
  typedef hello_world::ahrs::Mahony Mahony;

  void getRawData();

  void calcOffset();

  void updateAccGyro();

  bool updateMahony();

  BMI088 *bmi088_ptr_ = nullptr;  ///< 硬件接口
  Mahony *ahrs_ptr_ = nullptr;    ///< 姿态算法

  // 抽象数据，与硬件无关
  Imu3AxisData raw_acc_ = {0};   ///< 原始加速度，单位：m/s^2
  Imu3AxisData raw_gyro_ = {0};  ///< 原始角速度，单位：rad/s
  float temp_ = 0.0f;            ///< 温度，单位：℃

  float acc_threshold_ = 10;        ///< 加速度阈值，用于过滤短时撞击，单位：m/s^2
  float gyro_threshold_ = 0.1;      ///< 角速度阈值，用于去除异常数据，计算零飘时使用，单位：rad/s
  Imu3AxisData gyro_offset_ = {0};  ///< 角速度零飘，单位：rad/s

  Imu3AxisData acc_ = {0};   ///< 加速度，供 mahony 算法使用，单位：m/s^2
  Imu3AxisData gyro_ = {0};  ///< 角速度，供 mahony 算法和其他外部程序使用，单位：rad/s

  Imu3AxisData ang_ = {0};  ///< 姿态角度，由 mahony 算法计算，供外部程序使用，单位：rad

  ImuStatus status_ = ImuStatus::kNotInitHardware;  ///< IMU 工作状态
  size_t offset_count_ = 0;                         ///< 计数器，用于计算角速度零飘
  size_t offset_max_count_ = 100;                   ///< 最大计数器值，用于计算角速度零飘
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hero
#endif /* HERO_COMPONENTS_HERO_IMU_HPP_ */
