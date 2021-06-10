#pragma once

#include "sd/dynamics/kinematics.h"

namespace sd::estimators
{
  struct StateEst
  {
    Vector4d contact_estimate;        //接触估计
    Vector3d position;                //位置
    Vector3d rpy;                     //欧拉角
    dynamics::Quat orientation;                 //四元数
    dynamics::RotMat rot_body;                  //旋转矩阵
    Vector3d v_body, v_world;         //速度
    Vector3d omega_body, omega_world; //角速度
    Vector3d a_body, a_world;         //加速度，世界坐标下加速度
  };

}
