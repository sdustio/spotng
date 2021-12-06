#pragma once

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx::sensor {
struct SDQUADX_EXPORT ImuData {
  SdVector3f acc = {};
  SdVector3f gyro = {};
  SdVector4f quat = {};  // w, x, y, z
};

struct SDQUADX_EXPORT LegData {
  SdVector3f q = {};   // 关节角度
  SdVector3f qd = {};  // 关节角速度
};
using LegDatas = std::array<LegData, consts::model::kNumLeg>;

}  // namespace sdquadx::sensor
