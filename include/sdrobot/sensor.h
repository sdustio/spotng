#pragma once

#include "sdrobot/types.h"

namespace sdrobot::sensor
{
  struct SDROBOT_EXPORT ImuData
  {
    SdArray3f acc = {};
    SdArray3f gyro = {};
    SdArray4f quat = {};
  };
}
