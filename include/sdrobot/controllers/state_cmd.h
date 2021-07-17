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

  enum class Gait : uint8_t{
    Trot,
    SlowTrot,
    FlyingTrot,
    Walk,
    Bound,
    Pronk
  };

  class StateCmd
  {

  public:
    explicit StateCmd(double dt) : dt_(dt) { state_des_ = Vector12::Zero(); }

    bool Update(double mv_x, double mv_y, double tr, double pa, robot::Mode m);

    bool CmdtoStateData();

    const Vector12 &GetStateDes() const { return state_des_; }

    bool SetMode(robot::Mode mode)
    {
      cmd_mode_ = mode;
      return true;
    }

    robot::Mode GetMode() const { return cmd_mode_; }

    Gait GetGait() const {return cmd_gait_;}

    double GetStepHeight() const {return cmd_step_height_;}

  private:
    double Deadband(double v, double minVal, double maxVal);

    double cmd_move_x_;
    double cmd_move_y_;
    double cmd_turn_rate_;
    double cmd_pitch_angle_;
    double cmd_step_height_ = 0.1; //step height
    double cmd_height_variation_;
    robot::Mode cmd_mode_;
    Gait cmd_gait_ = Gait::Trot;

    // Dynamics matrix for discrete time approximation
    Vector12 state_des_;
    double dt_;
  };

  using StateCmdPtr = std::shared_ptr<StateCmd>;

}
