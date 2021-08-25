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

  struct SDROBOT_EXPORT Twist
  {
    fpt_t linear_vel_x = 0.;
    fpt_t linear_vel_y = 0.;
    fpt_t linear_vel_z = 0.; // reserved, unused
    fpt_t angular_vel_x = 0.; // reserved, unused, roll vel
    fpt_t angular_vel_y = 0.; // reserved, unused, pitch vel
    fpt_t angular_vel_z = 0.; // yaw vel
    fpt_t variant_pitch = 0.;
    fpt_t variant_height = 0.;
  };

  class SDROBOT_EXPORT DriveCtrl
  {
  public:
    using Ptr = std::unique_ptr<DriveCtrl>;
    using SharedPtr = std::shared_ptr<DriveCtrl>;
    using ConstSharedPtr = std::shared_ptr<DriveCtrl const>;

    virtual ~DriveCtrl() = default;

    virtual bool UpdateTwist(Twist const &twist) = 0;
    virtual bool UpdateState(State const &state) = 0;
    virtual bool UpdateGait(Gait const &gait) = 0;
    virtual bool UpdateStepHeight(fpt_t const height) = 0;

    virtual bool CmdtoDesData() = 0;

    virtual fpt_t GetDuration() const = 0;
    virtual fpt_t GetStepHeight() const = 0;
    virtual State GetState() const = 0;
    virtual Gait GetGait() const = 0;

    virtual SdVector3f const &GetPosDes() const = 0;
    virtual SdVector3f const &GetRpyDes() const = 0;
    virtual SdVector3f const &GetLvelDes() const = 0;
    virtual SdVector3f const &GetAvelDes() const = 0;
  };

}
