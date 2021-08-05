#pragma once

#include "eigen.h"

namespace sdrobot::dynamics
{
  enum class SDROBOT_EXPORT JointType
  {
    Prismatic,
    Revolute,
    FloatingBase,
    Nothing
  };

  enum class SDROBOT_EXPORT CoordinateAxis
  {
    X,
    Y,
    Z
  };

  // Rotation Matrix
  using RotMat = Matrix3;

  // Quaternions
  using Quat = Vector4;

  // Spatial Mass Matrix
  using SpatialInertia = Matrix6;

  // Spatial Vector (6x1, all subspaces)
  using SpatialVec = Vector6;

  // Spatial Transform (6x6)
  using SpatialXform = Matrix6;

  // 10x1 Vector
  using MassProperties = Eigen::Matrix<fpt_t, 10, 1>;

  using RotationalInertia = Matrix3; //InertiaMat

  using PseudoRotationalInertia = Matrix4; //PseudoInertiaMat

}
