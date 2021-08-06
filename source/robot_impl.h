#include "sdrobot/robot.h"
#include "sdrobot/model.h"
#include "sdrobot/leg.h"
#include "sdrobot/drive.h"
#include "sdrobot/estimate.h"
#include "sdrobot/fsm.h"


namespace sdrobot
{
  class RobotImpl : public Robot
  {
  public:
    RobotImpl(Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf);
    bool UpdateImu(sensor::ImuData const &imu) override;
    bool UpdateDriveCmd(drive::DriveCmd const &dcmd) override;
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

}
