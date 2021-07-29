#pragma once

#include "dynamics/types.h"

namespace sdrobot::dynamics
{
  /*!
   * Construct spatial inertia from mass, center of mass, and 3x3 rotational inertia
   * 从质量、质心和3 * 3的转动惯性来构造空间惯量
   */
  bool BuildSpatialInertia(Eigen::Ref<SpatialInertia> ret, double mass, Eigen::Ref<Vector3 const> const &com, Eigen::Ref<InertiaMat const> const &inertia);
}
