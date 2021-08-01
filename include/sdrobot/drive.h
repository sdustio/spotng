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
    fptype move_x;
    fptype move_y;
    fptype turn_rate;
    fptype angle_pitch;
    fptype variant_height;
    fptype step_height;
    State state;
    Gait gait;
  };

  class SDROBOT_EXPORT DriveCtrl
  {
  public:
    using Ptr = std::unique_ptr<DriveCtrl>;
    using SharedPtr = std::shared_ptr<DriveCtrl>;

    virtual ~DriveCtrl() = 0;

    virtual bool UpdateDriveCmd(DriveCmd const &cmd) = 0;
    virtual bool CmdtoDesData() = 0;

    virtual fptype GetDuration() const = 0;
    virtual fptype GetStepHeight() const = 0;
    virtual State GetState() const = 0;
    virtual Gait GetGait() const = 0;

    virtual SdVector3f const &GetPosDes() const = 0;
    virtual SdVector3f const &GetPosRpyDes() const = 0;
    virtual SdVector3f const &GetVelDes() const = 0;
    virtual SdVector3f const &GetVelRpyDes() const = 0;
  };

}
