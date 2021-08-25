#include "robot/robot_impl.h"

#include <memory>

#include "drive/drive_ctrl_impl.h"
#include "estimate/contact.h"
#include "estimate/estimate_ctrl_impl.h"
#include "estimate/orientation.h"
#include "estimate/pos_vel.h"
#include "fsm/impl.h"
#include "leg/jpos_init_impl.h"
#include "leg/leg_ctrl_impl.h"
#include "model/quadruped_impl.h"

namespace sdrobot {

RobotImpl::RobotImpl(Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf)
    : opts_(opts),
      mquad_(std::make_shared<model::QuadrupedImpl>()),
      legctrl_(std::make_shared<leg::LegCtrlImpl>(act_itf)),
      jposinit_(std::make_shared<leg::JPosInitImpl>(opts.ctrl_dt_sec, opts.jpos_init_sec)),
      drivectrl_(std::make_shared<drive::DriveCtrlImpl>(opts.drive_mode, opts.ctrl_dt_sec)),
      estctrl_(std::make_shared<estimate::EstimateCtrlImpl>()) {
  mquad_->ComputeFloatBaseModel(opts.gravity);

  estctrl_->AddEstimator("posvel",
                         std::make_shared<estimate::PosVel>(opts.ctrl_dt_sec, opts.gravity, legctrl_, mquad_));
  estctrl_->AddEstimator("ori", std::make_shared<estimate::Orientation>());
  auto est_contact = std::make_shared<estimate::Contact>();
  est_contact->UpdateContact({0.5, 0.5, 0.5, 0.5});
  estctrl_->AddEstimator("contact", est_contact);

  fsm_ = std::make_shared<fsm::FiniteStateMachineImpl>(opts, legctrl_, mquad_, drivectrl_, estctrl_);
}

bool RobotImpl::UpdateImu(sensor::ImuData const &imu) {
  std::dynamic_pointer_cast<estimate::Orientation>(estctrl_->GetEstimator("ori"))->UpdateImu(imu);
  return true;
}

bool RobotImpl::UpdateDriveTwist(drive::Twist const &twist) {
  drivectrl_->UpdateTwist(twist);
  return true;
}

bool RobotImpl::UpdateDriveState(drive::State const &state) {
  drivectrl_->UpdateState(state);
  return true;
}

bool RobotImpl::UpdateDriveGait(drive::Gait const &gait) {
  drivectrl_->UpdateGait(gait);
  return true;
}

bool RobotImpl::UpdateDriveStepHeight(fpt_t const height) {
  drivectrl_->UpdateStepHeight(height);
  return true;
}

bool RobotImpl::RunOnce() {
  legctrl_->UpdateDatasFromActuatorInterface();
  legctrl_->ZeroCmd();

  estctrl_->RunOnce();

  if (jposinit_->IsInitialized(legctrl_)) {
    drivectrl_->CmdtoDesData();
    fsm_->RunOnce();
  }

  legctrl_->SendCmdsToActuatorInterface();

  return true;
}

bool Robot::Build(Ptr &ret, Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf) {
  ret = std::make_unique<RobotImpl>(opts, act_itf);
  return true;
}

bool Robot::Build(SharedPtr &ret, Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf) {
  ret = std::make_shared<RobotImpl>(opts, act_itf);
  return true;
}
}  // namespace sdrobot
