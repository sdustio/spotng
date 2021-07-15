#pragma once

#include <memory>

#include "sdrobot/dynamics/kinematics.h"

namespace sdrobot::est
{
  struct StateData
  {
    Vector4 contact_estimate;        //接触估计
    Vector3 position;                //位置
    Vector3 rpy;                     //欧拉角
    dynamics::Quat orientation;       //四元数
    dynamics::RotMat rot_body;        //旋转矩阵
    Vector3 v_body, v_world;         //速度
    Vector3 omega_body, omega_world; //角速度
    Vector3 a_body, a_world;         //加速度，世界坐标下加速度
  };

  class StateEst
  {
  public:
    const StateData &GetData() const { return data_; }
    StateData &GetDataForUpdate() { return data_; }

  private:
    StateData data_;
  };

  using StateEstPtr = std::shared_ptr<StateEst>;

}
