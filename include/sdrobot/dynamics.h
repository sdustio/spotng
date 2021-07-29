#pragma once

#include "sdrobt/export.h"

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
} // namespace sdrobot::dynamics
