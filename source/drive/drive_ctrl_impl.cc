#include "sdrobot/params.h"
#include "drive/drive_ctrl_impl.h"

namespace sdrobot::drive
{
  DriveCtrlImpl::DriveCtrlImpl(DriveMode mode, fpt_t dt) : mode_(mode),
                                                           dt_(dt)
  {
  }

  DriveCtrlImpl::DriveCtrlImpl(fpt_t dt) : DriveCtrlImpl(DriveMode::kAutoAll, dt)
  {
  }

  bool DriveCtrlImpl::CmdtoDesData()
  {
    vel_ = {
        Deadband(cmd_.move_x, params::drive::kMinVelX, params::drive::kMaxVelX),
        Deadband(cmd_.move_y, params::drive::kMinVelY, params::drive::kMaxVelY),
        0};

    pos_ = {
        dt_ * vel_[0],
        dt_ * vel_[1],
        Deadband(cmd_.variant_height, params::drive::kMinVarHeight, params::drive::kMaxVarHeight)};

    vel_rpy_ = {
        0., 0, Deadband(cmd_.turn_rate, params::drive::kMinRateY, params::drive::kMaxRateY)};

    pos_rpy_ = {
        0.,
        Deadband(cmd_.angle_pitch, params::drive::kMinAngleP, params::drive::kMaxAngleP),
        dt_ * vel_rpy_[2]};

    step_height_ = cmd_.step_height;

    state_ = cmd_.state;
    gait_ = cmd_.gait;

    // TODO 根据 Drive Mode 进行参数修正。比如 自动档无视 state，gait， state 和 move 是否冲突等等
    return true;
  }

  bool DriveCtrlImpl::UpdateDriveCmd(DriveCmd const &cmd)
  {
    cmd_.state = cmd.state;
    cmd_.gait = cmd.gait;
    cmd_.step_height = Deadband(cmd.step_height, params::drive::kMinStepHeight, params::drive::kMaxStepHeight);

    cmd_.variant_height = cmd.variant_height;
    cmd_.move_x = cmd_.move_x * (1.0 - params::drive::kFilter) + cmd.move_x * params::drive::kFilter;
    cmd_.move_y = cmd_.move_y * (1.0 - params::drive::kFilter) + cmd.move_y * params::drive::kFilter;
    cmd_.turn_rate = cmd_.turn_rate * (1.0 - params::drive::kFilter) + cmd.turn_rate * params::drive::kFilter;
    cmd_.angle_pitch = cmd_.angle_pitch * (1.0 - params::drive::kFilter) + cmd.angle_pitch * params::drive::kFilter;
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
  SdVector3f const &DriveCtrlImpl::GetPosRpyDes() const
  {
    return pos_rpy_;
  }
  SdVector3f const &DriveCtrlImpl::GetVelDes() const
  {
    return vel_;
  }
  SdVector3f const &DriveCtrlImpl::GetVelRpyDes() const
  {
    return vel_rpy_;
  }

  fpt_t DriveCtrlImpl::Deadband(fpt_t v, fpt_t minVal, fpt_t maxVal)
  {
    if (v < params::drive::kDeadbandRegion && v > -params::drive::kDeadbandRegion)
    {
      return 0.0;
    }
    else
    {
      return (v / 2) * (maxVal - minVal);
    }
  }
}
