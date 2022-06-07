#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "spotng/consts.h"
#include "spotng/sensor.h"
#include "spotng/types.h"

namespace spotng::interface {

struct SPOTNG_EXPORT LegCmd {
  SdVector3f tau = {};
  SdVector3f q_des = {};
  SdVector3f qd_des = {};
  SdVector3f kp = {};
  SdVector3f kd = {};
};

using LegCmds = std::array<LegCmd, consts::model::kNumLeg>;

class SPOTNG_EXPORT Leg {
 public:
  using Ptr = std::unique_ptr<Leg>;
  using SharedPtr = std::shared_ptr<Leg>;
  using ConstSharedPtr = std::shared_ptr<Leg const>;

  virtual ~Leg() = default;
  virtual bool ReadTo(sensor::LegDatas &data) const = 0;
  virtual bool WriteFrom(LegCmds const &cmds) = 0;
};

class SPOTNG_EXPORT Imu {
 public:
  using Ptr = std::unique_ptr<Imu>;
  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<Imu const>;

  virtual ~Imu() = default;
  virtual bool ReadTo(sensor::ImuData &data) const = 0;
};
}  // namespace spotng::interface
