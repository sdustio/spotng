#include "sdquadx/fsm.h"
#include "sdquadx/interface.h"
#include "sdquadx/model.h"
#include "sdquadx/robot.h"

namespace sdquadx {
class RobotCtrlImpl : public RobotCtrl {
 public:
  RobotCtrlImpl(Options::SharedPtr const &opts, interface::Leg::SharedPtr const &leg_itf,
                interface::Imu::ConstSharedPtr const &imu_itf);

  drive::DriveCtrl::SharedPtr const &GetDriveCtrl() override;
  estimate::State const &GetEstimatState() const override;
  model::DynamicsData const &GetDynamicsData() const override;

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
