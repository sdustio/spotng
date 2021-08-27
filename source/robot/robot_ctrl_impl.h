#include "sdquadx/drive.h"
#include "sdquadx/estimate.h"
#include "sdquadx/fsm.h"
#include "sdquadx/leg.h"
#include "sdquadx/model.h"
#include "sdquadx/robot.h"

namespace sdquadx::robot {
class RobotCtrlImpl : public RobotCtrl {
 public:
  RobotCtrlImpl(Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf);
  bool UpdateImu(sensor::ImuData const &imu) override;
  bool UpdateDriveTwist(drive::Twist const &twist) override;
  bool UpdateDriveVarPos(drive::VarPos const &varpos) override;
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

}  // namespace sdquadx::robot
