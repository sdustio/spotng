#pragma once

#include "sdrobot/types.h"

namespace sdrobot::dynamics
{
  enum class JointType
  {
    Prismatic,
    Revolute,
    FloatingBase,
    Nothing
  };

  enum class CoordinateAxis
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
  using MassProperties = Vector10;

  using InertiaMat = Matrix3;

  using PseudoInertiaMat = Matrix4;

  // Dynamically sized matrix with spatial vector columns
  using SpatialVecXd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  // Dynamically sized matrix with cartesian vector columns
  using CartesianVecXd = Matrix3X;
}
