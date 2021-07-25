#pragma once

#include "sdrobot/types.h"
#include "sdrobot/geometry.h"

namespace sdrobot::sensor
{
  struct SDROBOT_EXPORT ImuData
  {
    Array3f acc;
    Array3f gyro;
    geometry::Quaternions quat;
  };
}
