#pragma once

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx::sensor {
struct SDQUADX_EXPORT ImuData {
  SdVector3f acc = {};
  SdVector3f gyro = {};
  // w.r.t ENU (magnetometer not required. so, EN are no need to be aligned to world)
  // w, x, y, z;
  SdVector4f quat = {1.};
};

struct SDQUADX_EXPORT LegData {
  SdVector3f q = {};
  SdVector3f qd = {};
};
using LegDatas = std::array<LegData, consts::model::kNumLeg>;

}  // namespace sdquadx::sensor
