#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "sdquadx/consts.h"
#include "sdquadx/sensor.h"
#include "sdquadx/types.h"

namespace sdquadx::interface {

struct SDQUADX_EXPORT LegCmd {
  SdVector3f tau = {};
  SdVector3f q_des = {};
  SdVector3f qd_des = {};
  SdVector3f kp = {};
  SdVector3f kd = {};
};

using LegCmds = std::array<interface::LegCmd, consts::model::kNumLeg>;

class SDQUADX_EXPORT Leg {
 public:
  using Ptr = std::unique_ptr<Leg>;
  using SharedPtr = std::shared_ptr<Leg>;
  using ConstSharedPtr = std::shared_ptr<Leg const>;

  virtual ~Leg() = default;
  virtual bool ReadTo(sensor::LegDatas &data) const = 0;
  virtual bool WriteFrom(LegCmds const &cmds) = 0;
};

class SDQUADX_EXPORT Imu {
 public:
  using Ptr = std::unique_ptr<Imu>;
  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<Imu const>;

  virtual ~Imu() = default;
  virtual bool ReadTo(sensor::ImuData &data) const = 0;
};
}  // namespace sdquadx::interface
