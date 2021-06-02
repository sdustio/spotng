#pragma once

#include "sd/dynamics/state.h"

namespace sd::ctrl
{
  template <typename T>
  class StateCmd
  {

  public:
    explicit StateCmd(float dt) : dt_(dt){state_des_ = Vec12<T>::Zero();}

    bool UpdateCmd(float mv_x, float mv_y, float tr, float pa, dynamics::Mode m)
    {
      cmd_mode_ = m;
      if (cmd_mode_ == dynamics::Mode::Stand || cmd_mode_ == dynamics::Mode::RecoveryStand)
      {
        mv_x = 0.0;
        mv_y = 0.0;
      }
      cmd_mv_x_ = cmd_mv_x_ * (T(1) - dynamics::CmdLimits::filter) + mv_x * dynamics::CmdLimits::filter;
      cmd_mv_y_ = cmd_mv_y_ * (T(1) - dynamics::CmdLimits::filter) + mv_y * dynamics::CmdLimits::filter;
      cmd_tr_ = cmd_tr_ * (T(1) - dynamics::CmdLimits::filter) + tr * dynamics::CmdLimits::filter;
      cmd_pa_ = cmd_pa_ * (T(1) - dynamics::CmdLimits::filter) + pa * dynamics::CmdLimits::filter;
      return true;
    }
    const Vec12<T>& GetStateDes() const {return state_des_;}

  private:
    bool CmdtoStateData()
    {

      state_des_.setZero();
      state_des_(dynamics::StateIdx::vel_x) = Deadband(cmd_mv_x_, dynamics::CmdLimits::min_vel_x, dynamics::CmdLimits::max_vel_x);
      state_des_(dynamics::StateIdx::vel_y) = Deadband(cmd_mv_y_, dynamics::CmdLimits::min_vel_y, dynamics::CmdLimits::max_vel_y);
      state_des_(dynamics::StateIdx::vel_z) = 0.0;
      state_des_(dynamics::StateIdx::pos_x) = dt_ * state_des_(dynamics::StateIdx::vel_x);
      state_des_(dynamics::StateIdx::pos_y) = dt_ * state_des_(dynamics::StateIdx::vel_y);
      state_des_(dynamics::StateIdx::pos_z) = 0.26;
      state_des_(dynamics::StateIdx::rate_r) = 0.0;
      state_des_(dynamics::StateIdx::rate_p) = 0.0;
      state_des_(dynamics::StateIdx::rate_y) = Deadband(cmd_tr_, dynamics::CmdLimits::min_rate_y, dynamics::CmdLimits::max_rate_y);
      state_des_(dynamics::StateIdx::angle_r) = 0.0;
      state_des_(dynamics::StateIdx::angle_p) = Deadband(cmd_pa_, dynamics::CmdLimits::min_angle_p, dynamics::CmdLimits::max_angle_p);
      state_des_(dynamics::StateIdx::angle_y) = dt_ * state_des_(dynamics::StateIdx::rate_y);

      return true;
    }

    float Deadband(float v, float minVal, float maxVal)
    {
      if (v < dynamics::CmdLimits::deadband_region && v > -dynamics::CmdLimits::deadband_region)
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
    dynamics::Mode cmd_mode_;

    // Dynamics matrix for discrete time approximation
    dynamics::StateVec<T> state_des_;
    T dt_;
  };

}
