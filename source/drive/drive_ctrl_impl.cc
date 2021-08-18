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
    vel_ = {
        Deadband(cmd_.move_x, consts::drive::kMinVelX, consts::drive::kMaxVelX),
        Deadband(cmd_.move_y, consts::drive::kMinVelY, consts::drive::kMaxVelY),
        0};

    pos_ = {
        dt_ * vel_[0],
        dt_ * vel_[1],
        Deadband(cmd_.variant_height, consts::drive::kMinVarHeight, consts::drive::kMaxVarHeight)};

    avel_ = {
        0., 0, Deadband(cmd_.turn_rate, consts::drive::kMinRateY, consts::drive::kMaxRateY)};

    rpy_ = {
        0.,
        Deadband(cmd_.angle_pitch, consts::drive::kMinAngleP, consts::drive::kMaxAngleP),
        dt_ * avel_[2]};

    step_height_ = cmd_.step_height;

    state_ = cmd_.state;
    gait_ = cmd_.gait;

    // TODO 根据 Drive Mode 进行参数修正。比如 自动档无视 state， state 和 move 是否冲突等等
    return true;
  }

  bool DriveCtrlImpl::UpdateDriveCmd(DriveCmd const &cmd)
  {
    cmd_.state = cmd.state;
    cmd_.gait = cmd.gait;
    cmd_.step_height = Deadband(cmd.step_height, consts::drive::kMinStepHeight, consts::drive::kMaxStepHeight);

    cmd_.variant_height = cmd.variant_height;
    cmd_.move_x = cmd_.move_x * (1.0 - consts::drive::kFilter) + cmd.move_x * consts::drive::kFilter;
    cmd_.move_y = cmd_.move_y * (1.0 - consts::drive::kFilter) + cmd.move_y * consts::drive::kFilter;
    cmd_.turn_rate = cmd_.turn_rate * (1.0 - consts::drive::kFilter) + cmd.turn_rate * consts::drive::kFilter;
    cmd_.angle_pitch = cmd_.angle_pitch * (1.0 - consts::drive::kFilter) + cmd.angle_pitch * consts::drive::kFilter;
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
  SdVector3f const &DriveCtrlImpl::GetVelDes() const
  {
    return vel_;
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
