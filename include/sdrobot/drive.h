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
    Bound
  };

  struct SDROBOT_EXPORT DriveCmd
  {
    fpt_t move_x;
    fpt_t move_y;
    fpt_t turn_rate;
    fpt_t angle_pitch;
    fpt_t variant_height;
    fpt_t step_height;
    State state;
    Gait gait;
  };

  class SDROBOT_EXPORT DriveCtrl
  {
  public:
    using Ptr = std::unique_ptr<DriveCtrl>;
    using SharedPtr = std::shared_ptr<DriveCtrl>;
    using ConstPtr = std::unique_ptr<DriveCtrl const>;
    using ConstSharedPtr = std::shared_ptr<DriveCtrl const>;

    virtual ~DriveCtrl() = 0;

    virtual bool UpdateDriveCmd(DriveCmd const &cmd) = 0;
    virtual bool CmdtoDesData() = 0;

    virtual fpt_t GetDuration() const = 0;
    virtual fpt_t GetStepHeight() const = 0;
    virtual State GetState() const = 0;
    virtual Gait GetGait() const = 0;

    virtual SdVector3f const &GetPosDes() const = 0;
    virtual SdVector3f const &GetPosRpyDes() const = 0;
    virtual SdVector3f const &GetVelDes() const = 0;
    virtual SdVector3f const &GetVelRpyDes() const = 0;
  };

}
