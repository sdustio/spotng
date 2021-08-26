#pragma once

#include "sdquadx/types.h"

namespace sdquadx::sensor {
struct SDQUADX_EXPORT ImuData {
  SdVector3f acc = {};
  SdVector3f gyro = {};
  SdVector4f quat = {};  // w, x, y, z
};
}  // namespace sdquadx::sensor
