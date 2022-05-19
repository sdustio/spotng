#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "sdengine/consts.h"
#include "sdengine/sensor.h"
#include "sdengine/types.h"

namespace sdengine::interface {

struct SDENGINE_EXPORT LegCmd {
  SdVector3f tau = {};
  SdVector3f q_des = {};
  SdVector3f qd_des = {};
  SdVector3f kp = {};
  SdVector3f kd = {};
};

using LegCmds = std::array<LegCmd, consts::model::kNumLeg>;

class SDENGINE_EXPORT Leg {
 public:
  using Ptr = std::unique_ptr<Leg>;
  using SharedPtr = std::shared_ptr<Leg>;
  using ConstSharedPtr = std::shared_ptr<Leg const>;

  virtual ~Leg() = default;
  virtual bool ReadTo(sensor::LegDatas &data) const = 0;
  virtual bool WriteFrom(LegCmds const &cmds) = 0;
};

class SDENGINE_EXPORT Imu {
 public:
  using Ptr = std::unique_ptr<Imu>;
  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<Imu const>;

  virtual ~Imu() = default;
  virtual bool ReadTo(sensor::ImuData &data) const = 0;
};
}  // namespace sdengine::interface
