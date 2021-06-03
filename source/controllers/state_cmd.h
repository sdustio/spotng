#pragma once

#include "sd/robot/interface.h"

namespace sd::ctrl
{
  struct StateIdx
  {
    constexpr static int pos_x = 0;   // X position
    constexpr static int pos_y = 1;   // Y position
    constexpr static int pos_z = 2;   // Z position
    constexpr static int angle_r = 3; // Roll angle
    constexpr static int angle_p = 4; // Pitch angle
    constexpr static int angle_y = 5; // Yaw angle
    constexpr static int vel_x = 6;   // X linear velocity
    constexpr static int vel_y = 7;   // Y linear velocity
    constexpr static int vel_z = 8;   // Z linear velocity
    constexpr static int rate_r = 9;  // Roll rate
    constexpr static int rate_p = 10; // Pitch rate
    constexpr static int rate_y = 11; // Yaw rate
  };

  struct CmdLimits
  {
    constexpr static float max_angle_r = 0.4;
    constexpr static float min_angle_r = -0.4;
    constexpr static float max_angle_p = 0.4;
    constexpr static float min_angle_p = -0.4;
    constexpr static float max_vel_x = 3.0;
    constexpr static float min_vel_x = -3.0;
    constexpr static float max_vel_y = 2.0;
    constexpr static float min_vel_y = -2.0;
    constexpr static float max_rate_y = 2.5;
    constexpr static float min_rate_y = -2.5;
    constexpr static float deadband_region = 0.075;
    constexpr static float filter = 0.1;
  };

  template <typename T>
  class StateCmd
  {

  public:
    explicit StateCmd(float dt) : dt_(dt){state_des_ = Vec12<T>::Zero();}

    bool Update(float mv_x, float mv_y, float tr, float pa, robot::Mode m)
    {
      cmd_mode_ = m;
      if (cmd_mode_ == robot::Mode::Stand || cmd_mode_ == robot::Mode::RecoveryStand)
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
    robot::Mode cmd_mode_;

    // Dynamics matrix for discrete time approximation
    Vec12<T> state_des_;
    T dt_;
  };

}
