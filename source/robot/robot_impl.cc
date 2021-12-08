#include "robot/robot_impl.h"

#include <memory>
#include <string>
#include <unordered_map>

#include "drive/drive_ctrl_impl.h"
#include "estimate/contact.h"
#include "estimate/impl.h"
#include "estimate/joints.h"
#include "estimate/orientation.h"
#include "estimate/pos_vel.h"
#include "fsm/impl.h"
#include "model/quadruped_impl.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace sdquadx {

RobotCtrlImpl::RobotCtrlImpl(Options::SharedPtr const &opts, interface::Leg::SharedPtr const &leg_itf,
                             interface::Imu::ConstSharedPtr const &imu_itf) {
  ParseOptions(opts);

  mquad_ = std::make_shared<model::QuadrupedImpl>(opts);

  drivectrl_ = std::make_shared<drive::DriveCtrlImpl>(opts->ctrl_sec);

  estctrl_ = std::make_shared<estimate::EstimateCtrlImpl>();
  estctrl_->AddEstimator("joints", std::make_shared<estimate::Joints>(leg_itf));
  estctrl_->AddEstimator("contact", std::make_shared<estimate::Contact>());
  estctrl_->AddEstimator("ori", std::make_shared<estimate::Orientation>(imu_itf));
  estctrl_->AddEstimator("posvel", std::make_shared<estimate::PosVel>(opts));

  fsm_ = std::make_shared<fsm::FiniteStateMachineImpl>(opts, leg_itf, mquad_, drivectrl_, estctrl_);
}

drive::DriveCtrl::SharedPtr const &RobotCtrlImpl::GetDriveCtrl() { return drivectrl_; }
estimate::State const &RobotCtrlImpl::GetEstimatState() const { return estctrl_->GetEstState(); }
model::DynamicsData const &RobotCtrlImpl::GetDynamicsData() const { return mquad_->GetDynamicsData(); }

bool RobotCtrlImpl::RunOnce() {
  estctrl_->RunOnce();

  if (!estctrl_->GetEstState().success) return false;

  drivectrl_->CmdtoDesData();
  return fsm_->RunOnce();
}

bool RobotCtrlImpl::ParseOptions(Options::SharedPtr const &opts) {
  std::shared_ptr<spdlog::logger> logger;
  auto logt = opts->log_target;
  if (logt == "console") {
    logger = spdlog::stdout_color_mt("sdlogger");
  } else if (logt == "file") {
    auto fn = opts->log_filename;
    logger = spdlog::rotating_logger_mt("sdlogger", fn, 1073741824, 3);  // max size 1GiB
  }

  std::unordered_map<std::string, spdlog::level::level_enum> loglevelmap = {{"debug", spdlog::level::debug},
                                                                            {"info", spdlog::level::info},
                                                                            {"warn", spdlog::level::warn},
                                                                            {"err", spdlog::level::err},
                                                                            {"critical", spdlog::level::critical}};
  logger->set_level(loglevelmap[opts->log_level]);

  spdlog::set_default_logger(logger);

  opts_ = opts;

  return true;
}

bool RobotCtrl::Build(Ptr &ret, Options::SharedPtr const &opts, interface::Leg::SharedPtr const &leg_itf,
                      interface::Imu::SharedPtr const &imu_itf) {
  ret = std::make_unique<RobotCtrlImpl>(opts, leg_itf, imu_itf);
  return true;
}

bool RobotCtrl::Build(SharedPtr &ret, Options::SharedPtr const &opts, interface::Leg::SharedPtr const &leg_itf,
                      interface::Imu::SharedPtr const &imu_itf) {
  ret = std::make_shared<RobotCtrlImpl>(opts, leg_itf, imu_itf);
  return true;
}
}  // namespace sdquadx
