#pragma once

#include "sd/dynamics/state.h"

namespace sd::dynamics
{
  template <typename T>
  class DesiredStateCmd
  {

  public:
    explicit DesiredStateCmd(float dt) : dt_(dt){state_des_ = Vec12<T>::Zero();}

    bool UpdateCmd(float mv_x, float mv_y, float tr, float pa, Mode m)
    {
      cmd_mode_ = m;
      if (cmd_mode_ == Mode::Stand || cmd_mode_ == Mode::RecoveryStand)
      {
        mv_x = 0.0;
        mv_y = 0.0;
      }
      cmd_mv_x_ = cmd_mv_x_ * (T(1) - CmdLimits::filter) + mv_x * CmdLimits::filter;
      cmd_mv_y_ = cmd_mv_y_ * (T(1) - CmdLimits::filter) + mv_y * CmdLimits::filter;
      cmd_tr_ = cmd_tr_ * (T(1) - CmdLimits::filter) + tr * CmdLimits::filter;
      cmd_pa_ = cmd_pa_ * (T(1) - CmdLimits::filter) + pa * CmdLimits::filter;
      return true;
    }
    const Vec12<T>& GetStateDes() const {return state_des_;}

  private:
    bool CmdtoStateData()
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

    float Deadband(float v, float minVal, float maxVal)
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

    float cmd_mv_x_;
    float cmd_mv_y_;
    float cmd_tr_;
    float cmd_pa_;
    Mode cmd_mode_;

    // Dynamics matrix for discrete time approximation
    StateVec<T> state_des_;
    T dt_;
  };

}
