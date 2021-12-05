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
  SdVector3f kp_joint = {};
  SdVector3f kd_joint = {};
};

using LegCmds = std::array<interface::LegCmd, consts::model::kNumLeg>;

class SDQUADX_EXPORT Leg {
 public:
  using Ptr = std::unique_ptr<Leg>;
  using SharedPtr = std::shared_ptr<Leg>;
  using ConstSharedPtr = std::shared_ptr<Leg const>;

  virtual ~Leg() = default;
  virtual bool SendLegData(sensor::LegData &data, int const leg) const = 0;
  virtual bool ReceiveLegCmds(LegCmds const &cmds) = 0;
  virtual bool RunOnce() = 0;  // return true if ok
};

class SDQUADX_EXPORT Imu {
 public:
  using Ptr = std::unique_ptr<Imu>;
  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<Imu const>;

  virtual ~Imu() = default;
  virtual bool SendImuData(sensor::ImuData &data) const = 0;
  virtual bool RunOnce() = 0;  // return true if ok
};
}  // namespace sdquadx::interface
