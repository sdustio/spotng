#include "sdrobot/drive.h"
#include "sdrobot/estimate.h"
#include "sdrobot/fsm.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"
#include "sdrobot/robot.h"

namespace sdrobot {
class RobotImpl : public Robot {
 public:
  RobotImpl(Options const &opts,
            interface::ActuatorInterface::SharedPtr const &act_itf);
  bool UpdateImu(sensor::ImuData const &imu) override;
  bool UpdateDriveTwist(drive::Twist const &twist) override;
  bool UpdateDriveState(drive::State const &state) override;
  bool UpdateDriveGait(drive::Gait const &gait) override;
  bool UpdateDriveStepHeight(fpt_t const height) override;
  bool RunOnce() override;

 private:
  Options const opts_;
  model::Quadruped::SharedPtr mquad_;
  leg::LegCtrl::SharedPtr legctrl_;
  leg::JPosInit::SharedPtr jposinit_;
  drive::DriveCtrl::SharedPtr drivectrl_;
  estimate::EstimateCtrl::SharedPtr estctrl_;
  fsm::FiniteStateMachine::SharedPtr fsm_;
};

}  // namespace sdrobot
