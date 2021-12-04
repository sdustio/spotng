#include "drive/drive_ctrl_impl.h"

#include "math/interpolate.h"
#include "sdquadx/consts.h"

namespace sdquadx::drive {
DriveCtrlImpl::DriveCtrlImpl(options::DriveMode mode, fpt_t dt) : mode_(mode), dt_(dt) {}

DriveCtrlImpl::DriveCtrlImpl(fpt_t dt) : DriveCtrlImpl(options::DriveMode::kAuto, dt) {}

bool DriveCtrlImpl::CmdtoDesData() {
  lvel_ = {Deadband(twist_.lvel_x, consts::drive::kMinVelX, consts::drive::kMaxVelX),
           Deadband(twist_.lvel_y, consts::drive::kMinVelY, consts::drive::kMaxVelY), 0};

  pos_ = {0., 0., Deadband(pose_.height, consts::drive::kMinVarHeight, consts::drive::kMaxVarHeight)};

  avel_ = {0., 0, Deadband(twist_.avel_z, consts::drive::kMinRateY, consts::drive::kMaxRateY)};

  rpy_ = {Deadband(pose_.roll, consts::drive::kMinAngleR, consts::drive::kMaxAngleR),
          Deadband(pose_.pitch, consts::drive::kMinAngleP, consts::drive::kMaxAngleP),
          Deadband(pose_.yaw, consts::drive::kMinAngleY, consts::drive::kMaxAngleY)};

  // TODO(Michael Ding) 根据 Drive Mode 进行参数修正。比如 自动档无视 state，
  // state 和 move 是否冲突等等
  return true;
}

bool DriveCtrlImpl::UpdateTwist(Twist const &twist) {
  math::interpolate_linear(twist_.lvel_x, twist_.lvel_x, twist.lvel_x, consts::drive::kFilter);
  math::interpolate_linear(twist_.lvel_y, twist_.lvel_y, twist.lvel_y, consts::drive::kFilter);
  math::interpolate_linear(twist_.lvel_z, twist_.lvel_z, twist.lvel_z, consts::drive::kFilter);

  return true;
}

bool DriveCtrlImpl::UpdatePose(Pose const &pose) {
  math::interpolate_linear(pose_.height, pose_.height, pose.height, consts::drive::kFilter);
  math::interpolate_linear(pose_.roll, pose_.roll, pose.roll, consts::drive::kFilter);
  math::interpolate_linear(pose_.pitch, pose_.pitch, pose.pitch, consts::drive::kFilter);
  math::interpolate_linear(pose_.yaw, pose_.yaw, pose.yaw, consts::drive::kFilter);

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
  step_height_ = Deadband(height, consts::drive::kMinStepHeight, consts::drive::kMaxStepHeight);
  return true;
}

fpt_t DriveCtrlImpl::GetDuration() const { return dt_; }
fpt_t DriveCtrlImpl::GetStepHeight() const { return step_height_; }
State DriveCtrlImpl::GetState() const { return state_; }
Gait DriveCtrlImpl::GetGait() const { return gait_; }

SdVector3f const &DriveCtrlImpl::GetPosDes() const { return pos_; }
SdVector3f const &DriveCtrlImpl::GetRpyDes() const { return rpy_; }
SdVector3f const &DriveCtrlImpl::GetLvelDes() const { return lvel_; }
SdVector3f const &DriveCtrlImpl::GetAvelDes() const { return avel_; }

fpt_t DriveCtrlImpl::Deadband(fpt_t v, fpt_t minVal, fpt_t maxVal) {
  if (v < consts::drive::kDeadbandRegion && v > -consts::drive::kDeadbandRegion) {
    return 0.0;
  } else {
    return (v / 2) * (maxVal - minVal);
  }
}
}  // namespace sdquadx::drive
