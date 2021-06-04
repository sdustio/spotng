#pragma once

#include "sd/types.h"

namespace sd::dynamics
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
  using RotMat = Matrix3d;

  // Quaternions
  using Quat = Vector4d;

  // Spatial Mass Matrix
  using SpatialInertia = Matrix6d;

  // Spatial Vector (6x1, all subspaces)
  using SpatialVec = Vector6d;

  // Spatial Transform (6x6)
  using SpatialXform = Matrix6d;

  // 10x1 Vector
  using MassProperties = Vector10d;

  using InertiaMat = Matrix3d;

  using PseudoInertiaMat = Matrix4d;

  // Dynamically sized matrix with spatial vector columns
  using SpatialVecXd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  // Dynamically sized matrix with cartesian vector columns
  using CartesianVecXd = Matrix3Xd;
}
