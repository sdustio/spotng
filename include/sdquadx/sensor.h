#pragma once

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx::sensor {
struct SDQUADX_EXPORT ImuData {
  SdVector3f acc = {};
  SdVector3f gyro = {};
  SdVector4f quat = {1.};  // w, x, y, z
};

struct SDQUADX_EXPORT LegData {
  SdVector3f q = {};
  SdVector3f qd = {};
};
using LegDatas = std::array<LegData, consts::model::kNumLeg>;

}  // namespace sdquadx::sensor
