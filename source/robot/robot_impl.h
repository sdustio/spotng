#include "sdquadx/robot.h"

#include "sdquadx/drive.h"
#include "sdquadx/estimate.h"
#include "sdquadx/fsm.h"
#include "sdquadx/model.h"
#include "sdquadx/interface.h"

namespace sdquadx {
class RobotCtrlImpl : public RobotCtrl {
 public:
  RobotCtrlImpl(Options::SharedPtr const &opts, interface::Leg::SharedPtr const &leg_itf, interface::Imu::ConstSharedPtr const &imu_itf);
  bool UpdateDriveTwist(drive::Twist const &twist) override;
  bool UpdateDrivePose(drive::Pose const &varpose) override;
  bool UpdateDriveState(drive::State const &state) override;
  bool UpdateDriveGait(drive::Gait const &gait) override;
  bool UpdateDriveStepHeight(fpt_t const height) override;
  bool RunOnce() override;

 private:
  bool ParseOptions(Options::SharedPtr const &opts);

  Options::ConstSharedPtr opts_;

  model::Quadruped::SharedPtr mquad_;
  drive::DriveCtrl::SharedPtr drivectrl_;
  estimate::EstimateCtrl::SharedPtr estctrl_;
  fsm::FiniteStateMachine::SharedPtr fsm_;

};

}  // namespace sdquadx
