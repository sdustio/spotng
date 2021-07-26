#pragma once

#include "sdrobot/drive.h"

namespace sdrobot::drive
{
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
    constexpr static double max_var_height = 0.2;
    constexpr static double min_var_height = -0.2;
    constexpr static double max_step_height = 0.2;
    constexpr static double min_step_height = 0.05;
    constexpr static double deadband_region = 0.075;
    constexpr static double filter = 0.1;
  };

  class DriveCtrlImpl : public DriveCtrl
  {
  public:
    DriveCtrlImpl(DriveMode mode, double dt);

    bool UpdateDriveCmd(DriveCmd const &cmd) override;
    bool CmdtoDesData() override;

    double GetDuration() const override;
    double GetStepHeight() const override;
    State GetState() const override;
    Gait GetGait() const override;

    Array3f const &GetPosDes() const override;
    Array3f const &GetPosRpyDes() const override;
    Array3f const &GetVelDes() const override;
    Array3f const &GetVelRpyDes() const override;

  private:
    double Deadband(double v, double minVal, double maxVal);

    DriveMode mode_;
    double dt_;
    DriveCmd cmd_;

    double step_height_ = 0.1;
    State state_ = State::Init;
    Gait gait_ = Gait::Trot;
    Array3f pos_ = {};
    Array3f pos_rpy_ = {};
    Array3f vel_ = {};
    Array3f vel_rpy_ = {};
  };

}
