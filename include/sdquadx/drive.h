#pragma once

#include <memory>

#include "sdquadx/options.h"

namespace sdquadx::drive {
enum class State : uint8_t { Init, RecoveryStand, Locomotion, BalanceStand };

enum class Gait : uint8_t { Trot, SlowTrot, FlyingTrot, Walk, Bound };

struct SDQUADX_EXPORT Twist {
  fpt_t lvel_x = 0.;
  fpt_t lvel_y = 0.;
  fpt_t lvel_z = 0.;      // reserved, unused
  fpt_t avel_x = 0.;      // reserved, unused, roll vel
  fpt_t avel_y = 0.;      // reserved, unused, pitch vel
  fpt_t avel_z = 0.;      // yaw vel
  fpt_t var_pitch = 0.;   // variant pitch
  fpt_t var_height = 0.;  // variant height
};

class SDQUADX_EXPORT DriveCtrl {
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

}  // namespace sdquadx::drive
