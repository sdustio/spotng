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
    fpt_t move_x = 0.;
    fpt_t move_y = 0.;
    fpt_t turn_rate = 0.;
    fpt_t angle_pitch = 0.;
    fpt_t variant_height = 0.;
    fpt_t step_height = 0.1;
    State state = State::Init;
    Gait gait = Gait::Trot;
  };

  class SDROBOT_EXPORT DriveCtrl
  {
  public:
    using Ptr = std::unique_ptr<DriveCtrl>;
    using SharedPtr = std::shared_ptr<DriveCtrl>;
    using ConstSharedPtr = std::shared_ptr<DriveCtrl const>;

    virtual ~DriveCtrl() = default;

    virtual bool UpdateDriveCmd(DriveCmd const &cmd) = 0;
    virtual bool CmdtoDesData() = 0;

    virtual fpt_t GetDuration() const = 0;
    virtual fpt_t GetStepHeight() const = 0;
    virtual State GetState() const = 0;
    virtual Gait GetGait() const = 0;

    virtual SdVector3f const &GetPosDes() const = 0;
    virtual SdVector3f const &GetRpyDes() const = 0;
    virtual SdVector3f const &GetVelDes() const = 0;
    virtual SdVector3f const &GetAvelDes() const = 0;
  };

}
