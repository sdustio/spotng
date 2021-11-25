#pragma once

#include "sdquadx/estimate.h"
#include "sdquadx/sensor.h"
#include "sdquadx/interface.h"

namespace sdquadx::estimate {
/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 * 得到四元数、旋转矩阵、角速度(身体与世界)、
 * rpy，加速度(世界，身体)从vector nav IMU数据
 * 实际用
 */
class Orientation : public Estimator {
 public:
  explicit Orientation(interface::Imu::ConstSharedPtr const &itf);
  bool RunOnce(State &ret) override;

 private:
  interface::Imu::ConstSharedPtr const itf_;
  bool b_first_visit_ = true;
  SdVector4f ori_ini_inv_;
  sensor::ImuData imu_;
};

}  // namespace sdquadx::estimate
