#pragma once

#include "sdrobot/types.h"

namespace sdrobot::sensor {
struct SDROBOT_EXPORT ImuData {
  SdVector3f acc = {};
  SdVector3f gyro = {};
  SdVector4f quat = {};  // w, x, y, z
};
}  // namespace sdrobot::sensor
