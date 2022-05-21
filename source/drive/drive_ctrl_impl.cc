#include "drive/drive_ctrl_impl.h"

#include <array>

#include "math/interpolate.h"
#include "math/utils.h"
#include "forax/consts.h"

namespace forax::drive {

namespace params {
constexpr fpt_t const kFilter = 0.1;
constexpr std::array<fpt_t, 5> const kGiatScale = {1., 0.8, 1.25, 0.6, 1.};
}  // namespace params

DriveCtrlImpl::DriveCtrlImpl(Options::ConstSharedPtr const &opts) : opts_(opts) {}

bool DriveCtrlImpl::CmdtoDesData() {
  lvel_ = {twist_.lvel_x, twist_.lvel_y, 0.};

  avel_ = {0., 0., twist_.avel_z};

  pos_ = {0., 0., pose_.height};

  rpy_ = {pose_.roll, pose_.pitch, pose_.yaw};

  if (mode_ == Mode::Auto) {
    if (state_ == State::Locomotion && ZeroVel()) {
      state_ = State::BalanceStand;
    } else if (state_ == State::BalanceStand && !ZeroVel()) {
      state_ = State::Locomotion;
    }
  }

  return true;
}

bool DriveCtrlImpl::UpdateTwist(Twist const &twist) {
  math::InterpolateLinear(twist_.lvel_x, twist_.lvel_x, twist.lvel_x, params::kFilter);
  math::InterpolateLinear(twist_.lvel_y, twist_.lvel_y, twist.lvel_y, params::kFilter);
  math::InterpolateLinear(twist_.avel_z, twist_.avel_z, twist.avel_z, params::kFilter);

  auto gt = static_cast<std::size_t>(gait_);

  twist_.lvel_x = math::LimitV(twist_.lvel_x, (params::kGiatScale[gt] * opts_->ctrl.max_trot_lvel_x),
                               (params::kGiatScale[gt] * opts_->ctrl.min_trot_lvel_x));
  twist_.lvel_y = math::LimitV(twist_.lvel_y, (params::kGiatScale[gt] * opts_->ctrl.max_trot_lvel_y),
                               (params::kGiatScale[gt] * -opts_->ctrl.max_trot_lvel_y));
  twist_.avel_z = math::LimitV(twist_.avel_z, (params::kGiatScale[gt] * opts_->ctrl.max_trot_avel_z),
                               (params::kGiatScale[gt] * -opts_->ctrl.max_trot_avel_z));

  return true;
}

bool DriveCtrlImpl::UpdatePose(Pose const &pose) {
  pose_ = pose;

  return true;
}

bool DriveCtrlImpl::UpdateMode(Mode const &mode) {
  mode_ = mode;
  return true;
}

bool DriveCtrlImpl::UpdateState(State const &state) {
  state_ = state;
  return true;
}

bool DriveCtrlImpl::UpdateGait(Gait const &gait) {
  gait_ = gait;
  return true;
}

bool DriveCtrlImpl::UpdateStepHeight(fpt_t const height) {
  step_height_ = height;
  return true;
}

bool DriveCtrlImpl::ZeroVel() {
  bool ret = true;
  ret = ret && (ToConstEigenTp(avel_).norm() < 0.0001);
  ret = ret && (ToConstEigenTp(lvel_).norm() < 0.0001);
  return ret;
}

fpt_t DriveCtrlImpl::GetStepHeight() const { return step_height_; }
State DriveCtrlImpl::GetState() const { return state_; }
Gait DriveCtrlImpl::GetGait() const { return gait_; }

SdVector3f const &DriveCtrlImpl::GetPosDes() const { return pos_; }
SdVector3f const &DriveCtrlImpl::GetRpyDes() const { return rpy_; }
SdVector3f const &DriveCtrlImpl::GetLvelDes() const { return lvel_; }
SdVector3f const &DriveCtrlImpl::GetAvelDes() const { return avel_; }

}  // namespace forax::drive
