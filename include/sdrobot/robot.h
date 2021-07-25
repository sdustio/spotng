#pragma once

#include "sdrobot/export.h"
#include "sdrobot/options.h"
#include "sdrobot/drive.h"
#include "sdrobot/sensor.h"
#include "sdrobot/interface.h"

namespace sdrobot
{
  class SDROBOT_EXPORT Robot
  {
    public:
    virtual ~Robot() = default;
    bool UpdateImu(sensor::ImuData const &imu);
    bool UpdateDriveCmd(drive::DriveCmd const &dcmd);
  };

  using RobotPtr = std::unique_ptr<Robot>;

  RobotPtr SDROBOT_EXPORT BuildRobot(Options const &opts, interface::ActuatorInterfacePtr const &act_itf);
} // namespace sdrobot
