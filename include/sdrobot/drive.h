#pragma once

#include <memory>

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

  class SDROBOT_EXPORT DriveCtrl
  {
  public:
    using Ptr = std::unique_ptr<DriveCtrl>;
    using SharedPtr = std::shared_ptr<DriveCtrl>;

    virtual ~DriveCtrl() = 0;

    virtual bool CmdtoDesData() = 0;

    virtual bool GetDriveCmd(DriveCmd &cmd) const = 0;
    virtual bool UpdateDriveCmd(DriveCmd const &cmd) = 0;

    virtual bool GetDuration(double &dt) const = 0;

    virtual bool GetPosDes(Array3f &pos) const = 0;
    virtual bool GetPosRpyDes(Array3f &pos_rpy) const = 0;
    virtual bool GetVelDes(Array3f &vel) const = 0;
    virtual bool GetVelRpyDes(Array3f &vel_rpy) const = 0;
  };

}
