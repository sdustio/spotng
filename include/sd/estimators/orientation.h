#pragma once

#include "sd/robot/interface.h"
#include "sd/estimators/state_est.h"

namespace sd::estimators
{
  /*!
  * Get quaternion, rotation matrix, angular velocity (body and world),
  * rpy, acceleration (world, body) from vector nav IMU
  * 得到四元数、旋转矩阵、角速度(身体与世界)、
  * rpy，加速度(世界，身体)从vector nav IMU数据
  * 实际用
  */
  class Orientation
  {
  public:
    bool Run(StateEst& ret, const robot::IMUData& imu);

  private:
    dynamics::Quat _ori_ini_inv;
    bool _b_first_visit = true;
  };

}
