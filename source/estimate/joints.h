#pragma once

#include <array>

#include "sdengine/consts.h"
#include "sdengine/estimate.h"
#include "sdengine/interface.h"
#include "sdengine/sensor.h"

namespace sdengine::estimate {
/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 * 得到四元数、旋转矩阵、角速度(身体与世界)、
 * rpy，加速度(世界，身体)从vector nav IMU数据
 * 实际用
 */
class Joints : public Estimator {
 public:
  explicit Joints(interface::Leg::ConstSharedPtr const &itf);
  bool RunOnce(State &ret) override;

 private:
  bool InterfaceValid();
  interface::Leg::ConstSharedPtr const itf_;
  sensor::LegDatas legdatas_;
};

}  // namespace sdengine::estimate
