#include "robot/robot_ctrl_impl.h"

#include <memory>
#include <string>
#include <unordered_map>

#include "drive/drive_ctrl_impl.h"
#include "estimate/contact.h"
#include "estimate/estimate_ctrl_impl.h"
#include "estimate/orientation.h"
#include "estimate/pos_vel.h"
#include "fsm/impl.h"
#include "leg/jpos_init_impl.h"
#include "leg/leg_ctrl_impl.h"
#include "model/quadruped_impl.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace sdquadx {

RobotCtrlImpl::RobotCtrlImpl(Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf)
    : opts_(opts) {
  ParseOptions();

  mquad_ = std::make_shared<model::QuadrupedImpl>();
  mquad_->ComputeFloatBaseModel(opts_.gravity);

  legctrl_ = std::make_shared<leg::LegCtrlImpl>(act_itf);

  jposinit_ = std::make_shared<leg::JPosInitImpl>(opts_.ctrl_sec, opts_.jpos_init_sec, opts.kp_joint, opts_.kd_joint,
                                                  opts_.init_jpos);

  drivectrl_ = std::make_shared<drive::DriveCtrlImpl>(opts_.drive_mode, opts_.ctrl_sec);

  estctrl_ = std::make_shared<estimate::EstimateCtrlImpl>();
  estctrl_->AddEstimator("posvel", std::make_shared<estimate::PosVel>(opts_.ctrl_sec, opts_.gravity, legctrl_, mquad_));
  estctrl_->AddEstimator("ori", std::make_shared<estimate::Orientation>());
  auto est_contact = std::make_shared<estimate::Contact>();
  est_contact->UpdateContact({0.5, 0.5, 0.5, 0.5});
  estctrl_->AddEstimator("contact", est_contact);

  fsm_ = std::make_shared<fsm::FiniteStateMachineImpl>(opts, legctrl_, mquad_, drivectrl_, estctrl_);
}

bool RobotCtrlImpl::UpdateImu(sensor::ImuData const &imu) {
  return std::dynamic_pointer_cast<estimate::Orientation>(estctrl_->GetEstimator("ori"))->UpdateImu(imu);
}

bool RobotCtrlImpl::UpdateDriveTwist(drive::Twist const &twist) { return drivectrl_->UpdateTwist(twist); }

bool RobotCtrlImpl::UpdateDriveVarPose(drive::VarPose const &varpose) { return drivectrl_->UpdateVarPose(varpose); }

bool RobotCtrlImpl::UpdateDriveState(drive::State const &state) { return drivectrl_->UpdateState(state); }

bool RobotCtrlImpl::UpdateDriveGait(drive::Gait const &gait) { return drivectrl_->UpdateGait(gait); }

bool RobotCtrlImpl::UpdateDriveStepHeight(fpt_t const height) { return drivectrl_->UpdateStepHeight(height); }

bool RobotCtrlImpl::RunOnce() {
  legctrl_->UpdateDatasFromActuatorInterface();

  estctrl_->RunOnce();

  if (jposinit_->IsInitialized(legctrl_)) {
    drivectrl_->CmdtoDesData();
    fsm_->RunOnce();
  }

  return legctrl_->SendCmdsToActuatorInterface();
}

bool RobotCtrlImpl::ParseOptions() {
  std::shared_ptr<spdlog::logger> logger;
  auto logt = opts_.log_target;
  if (logt == "console") {
    logger = spdlog::stdout_color_mt("sdlogger");
  } else if (logt == "file") {
    auto fn = opts_.log_filename;
    logger = spdlog::rotating_logger_mt("sdlogger", fn, 1048576, 3);  // max size 1mb
  }

  std::unordered_map<std::string, spdlog::level::level_enum> loglevelmap = {{"debug", spdlog::level::debug},
                                                                            {"info", spdlog::level::info},
                                                                            {"warn", spdlog::level::warn},
                                                                            {"err", spdlog::level::err},
                                                                            {"critical", spdlog::level::critical}};
  logger->set_level(loglevelmap[opts_.log_level]);

  spdlog::set_default_logger(logger);

  return true;
}

bool RobotCtrl::Build(Ptr &ret, Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf) {
  ret = std::make_unique<RobotCtrlImpl>(opts, act_itf);
  return true;
}

bool RobotCtrl::Build(SharedPtr &ret, Options const &opts, interface::ActuatorInterface::SharedPtr const &act_itf) {
  ret = std::make_shared<RobotCtrlImpl>(opts, act_itf);
  return true;
}
}  // namespace sdquadx
