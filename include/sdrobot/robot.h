#pragma once

#include <memory>

#include "sdrobot/drive.h"
#include "sdrobot/interface.h"
#include "sdrobot/options.h"
#include "sdrobot/sensor.h"

namespace sdrobot {
class SDROBOT_EXPORT Robot {
 public:
  using Ptr = std::unique_ptr<Robot>;
  using SharedPtr = std::shared_ptr<Robot>;
  using ConstSharedPtr = std::shared_ptr<Robot const>;

  static bool Build(Ptr &ret, Options const &opts,
                    interface::ActuatorInterface::SharedPtr const &act_itf);
  static bool Build(SharedPtr &ret, Options const &opts,
                    interface::ActuatorInterface::SharedPtr const &act_itf);

  virtual ~Robot() = default;

  virtual bool UpdateImu(sensor::ImuData const &imu) = 0;
  virtual bool UpdateDriveTwist(drive::Twist const &twist) = 0;
  virtual bool UpdateDriveState(drive::State const &state) = 0;
  virtual bool UpdateDriveGait(drive::Gait const &gait) = 0;
  virtual bool UpdateDriveStepHeight(fpt_t const height) = 0;

  virtual bool RunOnce() = 0;
};

}  // namespace sdrobot
