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
        Deadband(cmd_.move_x, CmdLimits::min_vel_x, CmdLimits::max_vel_x),
        Deadband(cmd_.move_y, CmdLimits::min_vel_y, CmdLimits::max_vel_y),
        0};

    pos_ = {
        dt_ * vel_[0],
        dt_ * vel_[1],
        Deadband(cmd_.variant_height, CmdLimits::min_var_height, CmdLimits::max_var_height)};

    vel_rpy_ = {
        0., 0, Deadband(cmd_.turn_rate, CmdLimits::min_rate_y, CmdLimits::max_rate_y)};

    pos_rpy_ = {
        0.,
        Deadband(cmd_.angle_pitch, CmdLimits::min_angle_p, CmdLimits::max_angle_p),
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

    step_height_ = cmd_.step_height = Deadband(cmd.step_height, CmdLimits::min_step_height, CmdLimits::max_step_height);

    cmd_.variant_height = cmd.variant_height;
    cmd_.move_x = cmd_.move_x * (1.0 - CmdLimits::filter) + cmd.move_x * CmdLimits::filter;
    cmd_.move_y = cmd_.move_y * (1.0 - CmdLimits::filter) + cmd.move_y * CmdLimits::filter;
    cmd_.turn_rate = cmd_.turn_rate * (1.0 - CmdLimits::filter) + cmd.turn_rate * CmdLimits::filter;
    cmd_.angle_pitch = cmd_.angle_pitch * (1.0 - CmdLimits::filter) + cmd.angle_pitch * CmdLimits::filter;
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
    if (v < CmdLimits::deadband_region && v > -CmdLimits::deadband_region)
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
