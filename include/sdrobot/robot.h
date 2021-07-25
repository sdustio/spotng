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

    virtual ~Robot() = default;

    virtual bool UpdateImu(sensor::ImuData const &imu) = 0;
    virtual bool UpdateDriveCmd(drive::DriveCmd const &dcmd) = 0;
    virtual bool RunOnce() = 0;
  };

} // namespace sdrobot
