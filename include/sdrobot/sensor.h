#pragma once

#include "sdrobot/types.h"

namespace sdrobot::sensor
{
  struct SDROBOT_EXPORT ImuData
  {
    Array3f acc = {};
    Array3f gyro = {};
    Array4f quat = {};
  };
}
