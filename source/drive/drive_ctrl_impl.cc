#include "sdrobot/consts.h"
#include "drive/drive_ctrl_impl.h"

namespace sdrobot::drive
{
  DriveCtrlImpl::DriveCtrlImpl(DriveMode mode, fpt_t dt) : mode_(mode),
                                                           dt_(dt)
  {
  }

  DriveCtrlImpl::DriveCtrlImpl(fpt_t dt) : DriveCtrlImpl(DriveMode::kAuto, dt)
  {
  }

  bool DriveCtrlImpl::CmdtoDesData()
  {
    lvel_ = {
        Deadband(twist_.linear_vel_x, consts::drive::kMinVelX, consts::drive::kMaxVelX),
        Deadband(twist_.linear_vel_y, consts::drive::kMinVelY, consts::drive::kMaxVelY),
        0};

    pos_ = {
        dt_ * lvel_[0],
        dt_ * lvel_[1],
        Deadband(twist_.variant_height, consts::drive::kMinVarHeight, consts::drive::kMaxVarHeight)};

    avel_ = {
        0., 0, Deadband(twist_.angular_vel_z, consts::drive::kMinRateY, consts::drive::kMaxRateY)};

    rpy_ = {
        0.,
        Deadband(twist_.variant_pitch, consts::drive::kMinAngleP, consts::drive::kMaxAngleP),
        dt_ * avel_[2]};


    // TODO 根据 Drive Mode 进行参数修正。比如 自动档无视 state， state 和 move 是否冲突等等
    return true;
  }

  bool DriveCtrlImpl::UpdateTwist(Twist const &twist)
  {
    twist_.variant_height = twist.variant_height;
    twist_.linear_vel_x = twist_.linear_vel_x * (1.0 - consts::drive::kFilter) + twist.linear_vel_x * consts::drive::kFilter;
    twist_.linear_vel_y = twist_.linear_vel_y * (1.0 - consts::drive::kFilter) + twist.linear_vel_y * consts::drive::kFilter;
    twist_.angular_vel_z = twist_.angular_vel_z * (1.0 - consts::drive::kFilter) + twist.angular_vel_z * consts::drive::kFilter;
    twist_.variant_pitch = twist_.variant_pitch * (1.0 - consts::drive::kFilter) + twist.variant_pitch * consts::drive::kFilter;
    return true;
  }

  bool DriveCtrlImpl::UpdateState(State const &state)
  {
    state_ = state;
    return true;
  }

  bool DriveCtrlImpl::UpdateGait(Gait const &gait)
  {
    gait_ = gait;
    return true;
  }

  bool DriveCtrlImpl::UpdateStepHeight(fpt_t const height)
  {
    step_height_ = Deadband(height, consts::drive::kMinStepHeight, consts::drive::kMaxStepHeight);;
    return true;
  }

  fpt_t DriveCtrlImpl::GetDuration() const
  {
    return dt_;
  }
  fpt_t DriveCtrlImpl::GetStepHeight() const
  {
    return step_height_;
  }
  State DriveCtrlImpl::GetState() const
  {
    return state_;
  }
  Gait DriveCtrlImpl::GetGait() const
  {
    return gait_;
  }

  SdVector3f const &DriveCtrlImpl::GetPosDes() const
  {
    return pos_;
  }
  SdVector3f const &DriveCtrlImpl::GetRpyDes() const
  {
    return rpy_;
  }
  SdVector3f const &DriveCtrlImpl::GetLvelDes() const
  {
    return lvel_;
  }
  SdVector3f const &DriveCtrlImpl::GetAvelDes() const
  {
    return avel_;
  }

  fpt_t DriveCtrlImpl::Deadband(fpt_t v, fpt_t minVal, fpt_t maxVal)
  {
    if (v < consts::drive::kDeadbandRegion && v > -consts::drive::kDeadbandRegion)
    {
      return 0.0;
    }
    else
    {
      return (v / 2) * (maxVal - minVal);
    }
  }
}
