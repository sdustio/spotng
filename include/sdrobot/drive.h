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
    double variant_height;
    double step_height;
    State state;
    Gait gait;
  };

  class SDROBOT_EXPORT DriveCtrl
  {
  public:
    using Ptr = std::unique_ptr<DriveCtrl>;
    using SharedPtr = std::shared_ptr<DriveCtrl>;

    static Ptr Build(double dt);
    static Ptr Build(DriveMode mode, double dt);
    static SharedPtr BuildShared(double dt);
    static SharedPtr BuildShared(DriveMode mode, double dt);

    virtual ~DriveCtrl() = 0;

    virtual bool UpdateDriveCmd(DriveCmd const &cmd) = 0;
    virtual bool CmdtoDesData() = 0;

    virtual double GetDuration() const = 0;
    virtual double GetStepHeight() const = 0;
    virtual State GetState() const = 0;
    virtual Gait GetGait() const = 0;

    virtual Array3f const &GetPosDes() const = 0;
    virtual Array3f const &GetPosRpyDes() const = 0;
    virtual Array3f const &GetVelDes() const = 0;
    virtual Array3f const &GetVelRpyDes() const = 0;
  };

}
