#pragma once

#include "sdrobot/options.h"
#include "sdrobot/drive.h"
#include "sdrobot/sensor.h"
#include "sdrobot/interface.h"

namespace sdrobot
{
  class SDROBOT_EXPORT Robot
  {
  public:
    using Ptr = std::unique_ptr<Robot>;
    using SharedPtr = std::shared_ptr<Robot>;

    static Ptr Build(Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf);
    static SharedPtr BuildShared(Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf);

    bool UpdateImu(sensor::ImuData const &imu);
    bool UpdateDriveCmd(drive::DriveCmd const &dcmd);
    bool RunOnce();
  };

} // namespace sdrobot
