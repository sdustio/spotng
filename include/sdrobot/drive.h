#pragma once

#include <memory>

#include "sdrobot/export.h"
#include "sdrobot/options.h"

namespace sdrobot::drive
{
  enum class State : uint8_t
  {
    Init,
    RecoveryStand,
    Locomotion,
    BalanceStand
  };

  enum class Gait : uint8_t
  {
    Trot,
    SlowTrot,
    FlyingTrot,
    Walk,
    Bound,
    Pronk
  };

  struct SDROBOT_EXPORT DriveCmd
  {
    double move_x;
    double move_y;
    double turn_rate;
    double angle_pitch;
    State state;
    Gait gait;
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

  class SDROBOT_EXPORT DriveCtrl
  {
  public:
    DriveCtrl(double dt, DriveMode mode);
    explicit DriveCtrl(double dt);

    bool CmdtoDesData();

    bool GetDriveCmd(DriveCmd &cmd) const;
    bool UpdateDriveCmd(DriveCmd const &cmd);

    bool GetPosDes(Array3f &pos) const;
    bool GetPosRpyDes(Array3f &pos_rpy) const;
    bool GetVelDes(Array3f &vel) const;
    bool GetVelRpyDes(Array3f &vel_rpy) const;
  };

  using DriveCtrlPtr = std::shared_ptr<DriveCtrl>;
}
