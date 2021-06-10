#include "sd/controllers/state_cmd.h"

namespace sd::ctrl
{
  bool StateCmd::Update(double mv_x, double mv_y, double tr, double pa, robot::Mode m)
  {
    cmd_mode_ = m;
    if (cmd_mode_ == robot::Mode::Stand || cmd_mode_ == robot::Mode::RecoveryStand)
    {
      mv_x = 0.0;
      mv_y = 0.0;
    }
    cmd_mv_x_ = cmd_mv_x_ * (1.0 - CmdLimits::filter) + mv_x * CmdLimits::filter;
    cmd_mv_y_ = cmd_mv_y_ * (1.0 - CmdLimits::filter) + mv_y * CmdLimits::filter;
    cmd_tr_ = cmd_tr_ * (1.0 - CmdLimits::filter) + tr * CmdLimits::filter;
    cmd_pa_ = cmd_pa_ * (1.0 - CmdLimits::filter) + pa * CmdLimits::filter;
    return true;
  }

  bool StateCmd::CmdtoStateData()
  {

    state_des_.setZero();
    state_des_(StateIdx::vel_x) = Deadband(cmd_mv_x_, CmdLimits::min_vel_x, CmdLimits::max_vel_x);
    state_des_(StateIdx::vel_y) = Deadband(cmd_mv_y_, CmdLimits::min_vel_y, CmdLimits::max_vel_y);
    state_des_(StateIdx::vel_z) = 0.0;
    state_des_(StateIdx::pos_x) = dt_ * state_des_(StateIdx::vel_x);
    state_des_(StateIdx::pos_y) = dt_ * state_des_(StateIdx::vel_y);
    state_des_(StateIdx::pos_z) = 0.26;
    state_des_(StateIdx::rate_r) = 0.0;
    state_des_(StateIdx::rate_p) = 0.0;
    state_des_(StateIdx::rate_y) = Deadband(cmd_tr_, CmdLimits::min_rate_y, CmdLimits::max_rate_y);
    state_des_(StateIdx::angle_r) = 0.0;
    state_des_(StateIdx::angle_p) = Deadband(cmd_pa_, CmdLimits::min_angle_p, CmdLimits::max_angle_p);
    state_des_(StateIdx::angle_y) = dt_ * state_des_(StateIdx::rate_y);

    return true;
  }

  double StateCmd::Deadband(double v, double minVal, double maxVal)
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
}