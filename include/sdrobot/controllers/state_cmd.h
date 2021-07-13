#pragma once

#include <memory>

#include "sdrobot/robot/interface.h"

namespace sdrobot::ctrl
{
  struct StateIdx
  {
    constexpr static size_t pos_x = 0;   // X delta position
    constexpr static size_t pos_y = 1;   // Y delta position
    constexpr static size_t pos_z = 2;   // Z delta position
    constexpr static size_t angle_r = 3; // Roll angle
    constexpr static size_t angle_p = 4; // Pitch angle
    constexpr static size_t angle_y = 5; // Yaw angle
    constexpr static size_t vel_x = 6;   // X linear velocity
    constexpr static size_t vel_y = 7;   // Y linear velocity
    constexpr static size_t vel_z = 8;   // Z linear velocity
    constexpr static size_t rate_r = 9;  // Roll rate
    constexpr static size_t rate_p = 10; // Pitch rate
    constexpr static size_t rate_y = 11; // Yaw rate
  };

  struct CmdLimits
  {
    constexpr static double max_angle_r = 0.4;
    constexpr static double min_angle_r = -0.4;
    constexpr static double max_angle_p = 0.4;
    constexpr static double min_angle_p = -0.4;
    constexpr static double max_vel_x = 3.0;
    constexpr static double min_vel_x = -3.0;
    constexpr static double max_vel_y = 2.0;
    constexpr static double min_vel_y = -2.0;
    constexpr static double max_rate_y = 2.5;
    constexpr static double min_rate_y = -2.5;
    constexpr static double deadband_region = 0.075;
    constexpr static double filter = 0.1;
  };

  class StateCmd
  {

  public:
    explicit StateCmd(double dt) : dt_(dt) { state_des_ = Vector12d::Zero(); }

    bool Update(double mv_x, double mv_y, double tr, double pa, robot::Mode m);
    bool SetMode(robot::Mode mode) {
      cmd_mode_ = mode;
      return true;
      }

    bool CmdtoStateData();
    const Vector12d &GetStateDes() const { return state_des_; }
    robot::Mode GetMode() const { return cmd_mode_; }

  private:
    double Deadband(double v, double minVal, double maxVal);

    double cmd_mv_x_;
    double cmd_mv_y_;
    double cmd_tr_;
    double cmd_pa_;
    robot::Mode cmd_mode_;

    // Dynamics matrix for discrete time approximation
    Vector12d state_des_;
    double dt_;
  };

  using StateCmdPtr = std::shared_ptr<StateCmd>;

}
