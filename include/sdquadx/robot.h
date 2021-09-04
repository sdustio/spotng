#pragma once

#include <memory>

#include "sdquadx/drive.h"
#include "sdquadx/interface.h"
#include "sdquadx/options.h"
#include "sdquadx/sensor.h"

namespace sdquadx {
class SDQUADX_EXPORT RobotCtrl {
 public:
  using Ptr = std::unique_ptr<RobotCtrl>;
  using SharedPtr = std::shared_ptr<RobotCtrl>;
  using ConstSharedPtr = std::shared_ptr<RobotCtrl const>;

  static bool Build(Ptr &ret, Options::SharedPtr const &opts, interface::ActuatorInterface::SharedPtr const &act_itf);
  static bool Build(SharedPtr &ret, Options::SharedPtr const &opts, interface::ActuatorInterface::SharedPtr const &act_itf);

  virtual ~RobotCtrl() = default;

  virtual bool UpdateImu(sensor::ImuData const &imu) = 0;
  virtual bool UpdateDriveTwist(drive::Twist const &twist) = 0;
  virtual bool UpdateDriveVarPose(drive::VarPose const &varpose) = 0;
  virtual bool UpdateDriveState(drive::State const &state) = 0;
  virtual bool UpdateDriveGait(drive::Gait const &gait) = 0;
  virtual bool UpdateDriveStepHeight(fpt_t const height) = 0;

  virtual bool RunOnce() = 0;
};

}  // namespace sdquadx
