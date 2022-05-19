#pragma once

#include "sdengine/consts.h"
#include "sdengine/types.h"

namespace sdengine::sensor {
struct SDENGINE_EXPORT ImuData {
  SdVector3f acc = {};
  SdVector3f gyro = {};
  // w.r.t ENU (magnetometer not required. so, EN are no need to be aligned to world)
  // w, x, y, z;
  SdVector4f quat = {1.};
};

struct SDENGINE_EXPORT LegData {
  SdVector3f q = {};
  SdVector3f qd = {};
};
using LegDatas = std::array<LegData, consts::model::kNumLeg>;

}  // namespace sdengine::sensor
