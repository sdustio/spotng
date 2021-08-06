#include "robot_impl.h"
#include "model/quadruped_impl.h"
#include "leg/jpos_init_impl.h"
#include "leg/leg_ctrl_impl.h"
#include "drive/drive_ctrl_impl.h"
#include "estimate/estimate_ctrl_impl.h"
#include "estimate/orientation.h"
#include "estimate/pos_vel.h"
#include "fsm/impl.h"

namespace sdrobot
{

  RobotImpl::RobotImpl(Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf) : opts_(opts)
  {
    mquad_ = std::make_shared<model::QuadrupedImpl>();
    mquad_->ComputeFloatBaseModel(opts.gravity);

    legctrl_ = std::make_shared<leg::LegCtrlImpl>(act_itf);
    jposinit_ = std::make_shared<leg::JPosInitImpl>(opts.ctrl_dt_sec, opts.jpos_init_sec);

    drivectrl_ = std::make_shared<drive::DriveCtrlImpl>(opts.drive_mode, opts.ctrl_dt_sec);

    estctrl_ = std::make_shared<estimate::EstimateCtrlImpl>();
    estctrl_->AddEstimator("posvel", std::make_shared<estimate::PosVel>(opts.ctrl_dt_sec, opts.gravity, legctrl_, mquad_));
    estctrl_->AddEstimator("ori", std::make_shared<estimate::Orientation>());

    fsm_ = std::make_shared<fsm::FiniteStateMachineImpl>(opts, legctrl_, mquad_, drivectrl_, estctrl_);
  }

  bool RobotImpl::UpdateImu(sensor::ImuData const &imu)
  {
    std::dynamic_pointer_cast<estimate::Orientation>(estctrl_->GetEstimator("ori"))->Update(imu);
    return true;
  }

  bool RobotImpl::UpdateDriveCmd(drive::DriveCmd const &dcmd)
  {
    drivectrl_->UpdateDriveCmd(dcmd);
    return true;
  }

  bool RobotImpl::RunOnce()
  {
    return true;
  }

  bool Robot::Build(Ptr &ret, Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf)
  {
    ret = std::make_unique<RobotImpl>(opts, act_itf);
    return true;
  }

  bool Robot::Build(SharedPtr &ret, Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf)
  {
    ret = std::make_shared<RobotImpl>(opts, act_itf);
    return true;
  }
}
