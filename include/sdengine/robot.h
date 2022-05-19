#pragma once

#include <memory>

#include "sdengine/drive.h"
#include "sdengine/estimate.h"
#include "sdengine/interface.h"
#include "sdengine/model.h"
#include "sdengine/options.h"

namespace sdengine {
class SDENGINE_EXPORT RobotCtrl {
 public:
  using Ptr = std::unique_ptr<RobotCtrl>;
  using SharedPtr = std::shared_ptr<RobotCtrl>;
  using ConstSharedPtr = std::shared_ptr<RobotCtrl const>;

  static bool Build(Ptr &ret, Options::SharedPtr const &opts, interface::Leg::SharedPtr const &leg_itf,
                    interface::Imu::SharedPtr const &imu_itf);
  static bool Build(SharedPtr &ret, Options::SharedPtr const &opts, interface::Leg::SharedPtr const &leg_itf,
                    interface::Imu::SharedPtr const &imu_itf);

  virtual ~RobotCtrl() = default;

  virtual drive::DriveCtrl::SharedPtr const &GetDriveCtrl() = 0;
  virtual estimate::State const &GetEstimatState() const = 0;
  virtual model::DynamicsData const &GetDynamicsData() const = 0;

  virtual bool RunOnce() = 0;
};

}  // namespace sdengine
