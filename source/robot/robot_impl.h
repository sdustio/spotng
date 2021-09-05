#include "sdquadx/drive.h"
#include "sdquadx/estimate.h"
#include "sdquadx/fsm.h"
#include "sdquadx/leg.h"
#include "sdquadx/model.h"
#include "sdquadx/robot.h"

namespace sdquadx {
class RobotCtrlImpl : public RobotCtrl {
 public:
  RobotCtrlImpl(Options::SharedPtr const &opts, interface::ActuatorInterface::SharedPtr const &act_itf);
  bool UpdateImu(sensor::ImuData const &imu) override;
  bool UpdateDriveTwist(drive::Twist const &twist) override;
  bool UpdateDriveVarPose(drive::VarPose const &varpose) override;
  bool UpdateDriveState(drive::State const &state) override;
  bool UpdateDriveGait(drive::Gait const &gait) override;
  bool UpdateDriveStepHeight(fpt_t const height) override;
  bool RunOnce() override;

 private:
  bool ParseOptions(Options::SharedPtr const &opts);

  Options::ConstSharedPtr opts_;
  model::Quadruped::SharedPtr mquad_;
  leg::LegCtrl::SharedPtr legctrl_;
  drive::DriveCtrl::SharedPtr drivectrl_;
  estimate::EstimateCtrl::SharedPtr estctrl_;
  fsm::FiniteStateMachine::SharedPtr fsm_;
};

}  // namespace sdquadx
