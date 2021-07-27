#include "params.h"
#include "drive/drive_ctrl_impl.h"

namespace sdrobot::drive
{
  DriveCtrlImpl::DriveCtrlImpl(DriveMode mode, double dt) : mode_(mode),
                                                            dt_(dt)
  {
  }

  bool DriveCtrlImpl::CmdtoDesData()
  {
    vel_ = {
        Deadband(cmd_.move_x, params::cmd::kMinVelX, params::cmd::kMaxVelX),
        Deadband(cmd_.move_y, params::cmd::kMinVelY, params::cmd::kMaxVelY),
        0};

    pos_ = {
        dt_ * vel_[0],
        dt_ * vel_[1],
        Deadband(cmd_.variant_height, params::cmd::kMinVarHeight, params::cmd::kMaxVarHeight)};

    vel_rpy_ = {
        0., 0, Deadband(cmd_.turn_rate, params::cmd::kMinRateY, params::cmd::kMaxRateY)};

    pos_rpy_ = {
        0.,
        Deadband(cmd_.angle_pitch, params::cmd::kMinAngleP, params::cmd::kMaxAngleP),
        dt_ * vel_rpy_[2]};

    // TODO Drive Mode
    return true;
  }

  bool DriveCtrlImpl::UpdateDriveCmd(DriveCmd const &cmd)
  {
    // state_ 可能被内部重置。
    // 只有在传入的 state 发生变化时才更新 state_
    // 否则我们应当保持 state_ 为内部的重置值

    if (cmd_.state != cmd.state)
      state_ = cmd_.state = cmd.state;
    gait_ = cmd_.gait = cmd.gait;

    step_height_ = cmd_.step_height = Deadband(cmd.step_height, params::cmd::kMinStepHeight, params::cmd::kMaxStepHeight);

    cmd_.variant_height = cmd.variant_height;
    cmd_.move_x = cmd_.move_x * (1.0 - params::cmd::kFilter) + cmd.move_x * params::cmd::kFilter;
    cmd_.move_y = cmd_.move_y * (1.0 - params::cmd::kFilter) + cmd.move_y * params::cmd::kFilter;
    cmd_.turn_rate = cmd_.turn_rate * (1.0 - params::cmd::kFilter) + cmd.turn_rate * params::cmd::kFilter;
    cmd_.angle_pitch = cmd_.angle_pitch * (1.0 - params::cmd::kFilter) + cmd.angle_pitch * params::cmd::kFilter;
    return true;
  }

  double DriveCtrlImpl::GetDuration() const
  {
    return dt_;
  }
  double DriveCtrlImpl::GetStepHeight() const
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

  Array3f const &DriveCtrlImpl::GetPosDes() const
  {
    return pos_;
  }
  Array3f const &DriveCtrlImpl::GetPosRpyDes() const
  {
    return pos_rpy_;
  }
  Array3f const &DriveCtrlImpl::GetVelDes() const
  {
    return vel_;
  }
  Array3f const &DriveCtrlImpl::GetVelRpyDes() const
  {
    return vel_rpy_;
  }

  double DriveCtrlImpl::Deadband(double v, double minVal, double maxVal)
  {
    if (v < params::cmd::kDeadbandRegion && v > -params::cmd::kDeadbandRegion)
    {
      return 0.0;
    }
    else
    {
      return (v / 2) * (maxVal - minVal);
    }
  }

  DriveCtrl::Ptr DriveCtrl::Build(double dt)
  {
    return std::make_unique<DriveCtrlImpl>(DriveMode::kAutoAll, dt);
  }
  DriveCtrl::Ptr DriveCtrl::Build(DriveMode mode, double dt)
  {
    return std::make_unique<DriveCtrlImpl>(mode, dt);
  }
  DriveCtrl::SharedPtr DriveCtrl::BuildShared(double dt)
  {
    return std::make_shared<DriveCtrlImpl>(DriveMode::kAutoAll, dt);
  }
  DriveCtrl::SharedPtr DriveCtrl::BuildShared(DriveMode mode, double dt)
  {
    return std::make_shared<DriveCtrlImpl>(mode, dt);
  }
}
