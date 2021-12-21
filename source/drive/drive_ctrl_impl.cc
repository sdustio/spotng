#include "drive/drive_ctrl_impl.h"

#include <array>

#include "math/interpolate.h"
#include "math/utils.h"
#include "sdquadx/consts.h"

namespace sdquadx::drive {

namespace params {
constexpr fpt_t const kFilter = 0.1;
constexpr std::array<fpt_t, 5> const kGiatScale = {1., 0.8, 1.25, 0.6, 1.};
}  // namespace params

DriveCtrlImpl::DriveCtrlImpl(Options::ConstSharedPtr const &opts) : opts_(opts) {}

bool DriveCtrlImpl::CmdtoDesData() {
  lvel_ = {twist_.lvel_x, twist_.lvel_y, 0};

  avel_ = {0., 0, twist_.avel_z};

  pos_ = {0., 0., pose_.height};

  rpy_ = {pose_.roll, pose_.pitch, pose_.yaw};

  // TODO(Michael Ding) 根据 Drive Mode 进行参数修正。比如 自动档无视 state，
  // state 和 move 是否冲突等等
  return true;
}

bool DriveCtrlImpl::UpdateTwist(Twist const &twist) {
  math::InterpolateLinear(twist_.lvel_x, twist_.lvel_x, twist.lvel_x, params::kFilter);
  math::InterpolateLinear(twist_.lvel_y, twist_.lvel_y, twist.lvel_y, params::kFilter);
  math::InterpolateLinear(twist_.avel_z, twist_.avel_z, twist.avel_z, params::kFilter);

  auto gt = static_cast<size_t>(gait_);

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

fpt_t DriveCtrlImpl::GetStepHeight() const { return step_height_; }
State DriveCtrlImpl::GetState() const { return state_; }
Gait DriveCtrlImpl::GetGait() const { return gait_; }

SdVector3f const &DriveCtrlImpl::GetPosDes() const { return pos_; }
SdVector3f const &DriveCtrlImpl::GetRpyDes() const { return rpy_; }
SdVector3f const &DriveCtrlImpl::GetLvelDes() const { return lvel_; }
SdVector3f const &DriveCtrlImpl::GetAvelDes() const { return avel_; }

}  // namespace sdquadx::drive
