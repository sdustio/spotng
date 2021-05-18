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
  template <typename T>
  using RotMat = Mat3<T>;

  // Quaternions
  template <typename T>
  using Quat = Vec4<T>;

  // Spatial Mass Matrix
  template <typename T>
  using SpatialInertia = Mat6<T>;

  // Spatial Vector (6x1, all subspaces)
  template <typename T>
  using SVec = Vec6<T>;

  // Spatial Transform (6x6)
  template <typename T>
  using SXform = Mat6<T>;

  // 10x1 Vector
  template <typename T>
  using MassProperties = Vec10<T>;

  template <typename T>
  using InertiaMat = Mat3<T>;

  template <typename T>
  using PseudoInertiaMat = Mat4<T>;
}
